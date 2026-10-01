/*
 * The model behind tests/idf-stub/, and the two IDF globals the port compares
 * against.
 *
 * Every function here is one IDF would provide on a chip, with one difference
 * worth naming: nothing in this file blocks, fails on its own, or has a
 * random element. A radio that answered differently on the second run would
 * make the check a coin toss, and the failures a bench cannot arrange (a
 * driver that will not start, a netif that will not be created) are switches
 * the test throws instead.
 */

#include "host_esp32_net_model.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include <string.h>

/*
 * IDF's netif is opaque to the port - that is what a pointer to it is - so the
 * model gives it a body here and the port cannot tell the difference. The
 * address itself lives in the model, not in the object, because on a chip it
 * comes from DHCP rather than from anything the port wrote.
 */
struct esp_netif_obj {
    int role; /* which helper made it */
};

/* The event bases, defined once: the port compares the base it is handed with
 * the one it registered for, by address, exactly as IDF does. */
const char *IP_EVENT = "IP_EVENT";
const char *WIFI_EVENT = "WIFI_EVENT";

#define AK_HOST_NET_PORT 5555
#define AK_HOST_NET_REGISTRATIONS 4
#define AK_HOST_NET_BUFFERS 4
#define AK_HOST_NET_BUFFER_BYTES 2048
#define AK_HOST_NET_SENT_BYTES 2048
#define AK_HOST_NET_CLIENT_BYTES 256

typedef struct {
    const char       *base;
    int               id;
    esp_event_handler_t handler;
    void             *argument;
} net_registration_t;

typedef struct {
    uint8_t  bytes[AK_HOST_NET_BUFFER_BYTES];
    unsigned head;
    unsigned count;
    unsigned created;
} net_buffer_t;

static net_registration_t events[AK_HOST_NET_REGISTRATIONS];
static unsigned event_count;

static net_buffer_t buffers[AK_HOST_NET_BUFFERS];

static int wifi_inited;
static int wifi_storage = -1;
static int wifi_mode = -1;
static int wifi_config_if = -1;
static unsigned wifi_starts;
static unsigned wifi_connects;
static char sta_ssid[33];
static char sta_password[65];
static int  sta_authmode = -1;
static char ap_ssid[33];
static char ap_password[65];
static int  ap_ssid_len = -1;
static int  ap_max_connection = -1;
static int  ap_authmode = -1;

static struct esp_netif_obj netif_sta_obj;
static struct esp_netif_obj netif_ap_obj;
static struct esp_netif_obj netif_generic_obj;
static unsigned netif_sta_count;
static unsigned netif_ap_count;
static unsigned ip_reads;
static uint32_t netif_ip;

static int refuse_wifi_init;
static int refuse_wifi_start;
static int refuse_netif;
static int refuse_socket;
static int refuse_bind;
static int refuse_listen;
static int refuse_accept;

static unsigned tasks;
static TaskFunction_t task_body;
static const char *task_name = "";
static unsigned task_stack;
static unsigned task_priority;

static unsigned sockets;
static int socket_reuseaddr = -1;
static int socket_bound_port = -1;
static int socket_bound_any;
static int socket_backlog = -1;
static unsigned closes;

static const uint8_t *client_bytes;
static unsigned client_length;
static unsigned client_at;
static int client_done;

static void (*busy_hook)(void);

static uint8_t sent[AK_HOST_NET_SENT_BYTES];
static unsigned sent_bytes;
static unsigned selects;
static unsigned select_timeouts;
static unsigned select_wait_ms;

static jmp_buf *escape_env;

#define AK_HOST_NET_CLIENT_FD 5

void ak_host_net_reset(void)
{
    memset(events, 0, sizeof events);
    event_count = 0u;
    memset(buffers, 0, sizeof buffers);

    wifi_inited = 0;
    wifi_storage = -1;
    wifi_mode = -1;
    wifi_config_if = -1;
    wifi_starts = 0u;
    wifi_connects = 0u;
    sta_ssid[0] = '\0';
    sta_password[0] = '\0';
    sta_authmode = -1;
    ap_ssid[0] = '\0';
    ap_password[0] = '\0';
    ap_ssid_len = -1;
    ap_max_connection = -1;
    ap_authmode = -1;

    netif_sta_count = 0u;
    netif_ap_count = 0u;
    ip_reads = 0u;
    netif_ip = 0u;

    refuse_wifi_init = 0;
    refuse_wifi_start = 0;
    refuse_netif = 0;
    refuse_socket = 0;
    refuse_bind = 0;
    refuse_listen = 0;
    refuse_accept = 0;

    tasks = 0u;
    task_body = 0;
    task_name = "";
    task_stack = 0u;
    task_priority = 0u;

    sockets = 0u;
    socket_reuseaddr = -1;
    socket_bound_port = -1;
    socket_bound_any = 0;
    socket_backlog = -1;
    closes = 0u;

    client_bytes = 0;
    client_length = 0u;
    client_at = 0u;
    client_done = 0;
    busy_hook = 0;
    sent_bytes = 0u;
    selects = 0u;
    select_timeouts = 0u;
    select_wait_ms = 0u;
    escape_env = NULL;
}

/* --- IDF's system layer --------------------------------------------------- */

void ak_host_idf_error_check(esp_err_t err, int line)
{
    (void)line;
    if (err != ESP_OK) {
        ak_host_idf_log("ak-net", "ESP_ERROR_CHECK(%d) would have restarted "
                                  "the board", (int)err);
    }
}

/* --- the event loop ------------------------------------------------------ */

esp_err_t esp_event_loop_create_default(void)
{
    return ESP_OK;
}

esp_err_t esp_event_handler_register(esp_event_base_t base, int32_t id,
                                     esp_event_handler_t handler, void *argument)
{
    if (event_count >= AK_HOST_NET_REGISTRATIONS) {
        return ESP_FAIL;
    }
    events[event_count].base = base;
    events[event_count].id = (int)id;
    events[event_count].handler = handler;
    events[event_count].argument = argument;
    event_count++;
    return ESP_OK;
}

void ak_host_net_raise(const char *base, int id)
{
    for (unsigned i = 0u; i < event_count; i++) {
        if (events[i].base == base && events[i].id == id) {
            events[i].handler(events[i].argument, events[i].base, id, NULL);
        }
    }
}

/* --- the radio ----------------------------------------------------------- */

esp_err_t esp_wifi_init(const wifi_init_config_t *config)
{
    (void)config;
    if (refuse_wifi_init) {
        return ESP_FAIL;
    }
    wifi_inited = 1;
    return ESP_OK;
}

esp_err_t esp_wifi_set_storage(wifi_storage_t storage)
{
    wifi_storage = (int)storage;
    return ESP_OK;
}

esp_err_t esp_wifi_set_mode(wifi_mode_t mode)
{
    wifi_mode = (int)mode;
    return ESP_OK;
}

esp_err_t esp_wifi_set_config(wifi_interface_t interface, wifi_config_t *config)
{
    wifi_config_if = (int)interface;

    if (interface == WIFI_IF_STA) {
        memcpy(sta_ssid, config->sta.ssid, 32u);
        sta_ssid[32] = '\0';
        memcpy(sta_password, config->sta.password, 64u);
        sta_password[64] = '\0';
        sta_authmode = (int)config->sta.threshold.authmode;
    } else {
        memcpy(ap_ssid, config->ap.ssid, 32u);
        ap_ssid[32] = '\0';
        memcpy(ap_password, config->ap.password, 64u);
        ap_password[64] = '\0';
        ap_ssid_len = (int)config->ap.ssid_len;
        ap_max_connection = (int)config->ap.max_connection;
        ap_authmode = (int)config->ap.authmode;
    }
    return ESP_OK;
}

esp_err_t esp_wifi_start(void)
{
    if (refuse_wifi_start) {
        return ESP_FAIL;
    }
    wifi_starts++;
    return ESP_OK;
}

esp_err_t esp_wifi_connect(void)
{
    wifi_connects++;
    return ESP_OK;
}

/* --- the netif ----------------------------------------------------------- */

esp_err_t esp_netif_init(void)
{
    return ESP_OK;
}

esp_netif_t *esp_netif_create_default_wifi_sta(void)
{
    if (refuse_netif) {
        return NULL;
    }
    netif_sta_count++;
    return &netif_sta_obj;
}

esp_netif_t *esp_netif_create_default_wifi_ap(void)
{
    if (refuse_netif) {
        return NULL;
    }
    netif_ap_count++;
    return &netif_ap_obj;
}

esp_netif_t *esp_netif_new(const esp_netif_config_t *config)
{
    (void)config;
    return &netif_generic_obj;
}

esp_err_t esp_netif_attach(esp_netif_t *netif, void *io_driver)
{
    (void)netif;
    (void)io_driver;
    return ESP_OK;
}

esp_err_t esp_netif_get_ip_info(esp_netif_t *netif, esp_netif_ip_info_t *info)
{
    if (netif == NULL || info == NULL) {
        return ESP_FAIL;
    }
    ip_reads++;
    info->ip.addr = netif_ip;
    info->netmask.addr = 0x00FFFFFFu;
    info->gw.addr = netif_ip;
    return ESP_OK;
}

void ak_host_net_set_address(unsigned a, unsigned b, unsigned c, unsigned d)
{
    netif_ip = (a & 0xFFu) | ((b & 0xFFu) << 8) | ((c & 0xFFu) << 16) |
               ((d & 0xFFu) << 24);
}

/* --- the stream buffers -------------------------------------------------- */

static net_buffer_t *buffer_of(StreamBufferHandle_t handle)
{
    for (unsigned i = 0u; i < AK_HOST_NET_BUFFERS; i++) {
        if (handle == (StreamBufferHandle_t)&buffers[i]) {
            return &buffers[i];
        }
    }
    return NULL;
}

StreamBufferHandle_t xStreamBufferCreate(size_t buffer_size, size_t trigger)
{
    (void)trigger;
    if (buffer_size > AK_HOST_NET_BUFFER_BYTES) {
        return NULL;
    }
    for (unsigned i = 0u; i < AK_HOST_NET_BUFFERS; i++) {
        if (!buffers[i].created) {
            buffers[i].created = 1u;
            buffers[i].head = 0u;
            buffers[i].count = 0u;
            return (StreamBufferHandle_t)&buffers[i];
        }
    }
    return NULL;
}

size_t xStreamBufferSend(StreamBufferHandle_t handle, const void *data,
                         size_t length, TickType_t wait)
{
    (void)wait;
    net_buffer_t *buffer = buffer_of(handle);
    const uint8_t *bytes = data;

    if (buffer == NULL) {
        return 0u;
    }
    size_t put = 0u;
    while (put < length && buffer->count < AK_HOST_NET_BUFFER_BYTES) {
        unsigned at = (buffer->head + buffer->count) % AK_HOST_NET_BUFFER_BYTES;
        buffer->bytes[at] = bytes[put];
        buffer->count++;
        put++;
    }
    return put;
}

size_t xStreamBufferReceive(StreamBufferHandle_t handle, void *data,
                            size_t length, TickType_t wait)
{
    (void)wait;
    net_buffer_t *buffer = buffer_of(handle);
    uint8_t *bytes = data;

    if (buffer == NULL) {
        return 0u;
    }
    size_t got = 0u;
    while (got < length && buffer->count > 0u) {
        bytes[got] = buffer->bytes[buffer->head];
        buffer->head = (buffer->head + 1u) % AK_HOST_NET_BUFFER_BYTES;
        buffer->count--;
        got++;
    }
    return got;
}

BaseType_t xStreamBufferReset(StreamBufferHandle_t handle)
{
    net_buffer_t *buffer = buffer_of(handle);
    if (buffer == NULL) {
        return pdFALSE;
    }
    buffer->head = 0u;
    buffer->count = 0u;
    return pdTRUE;
}

/* --- tasks --------------------------------------------------------------- */

BaseType_t xTaskCreate(TaskFunction_t task, const char *name,
                       unsigned stack_depth, void *argument,
                       unsigned priority, TaskHandle_t *created)
{
    (void)argument;
    if (task == 0) {
        return pdFALSE;
    }
    tasks++;
    task_body = task;
    task_name = name;
    task_stack = stack_depth;
    task_priority = priority;
    if (created != NULL) {
        *created = (TaskHandle_t)&tasks;
    }
    return pdTRUE;
}

void vTaskDelete(TaskHandle_t task)
{
    (void)task;
}

void ak_host_net_run_task(jmp_buf *escape)
{
    escape_env = escape;
    if (task_body != 0) {
        task_body(NULL);
    }
    /* Only reached if the port's task returned, and the real one returns in
     * exactly one place: the listener it could not open, where it logs and
     * deletes itself. 99 is that path, and the check reads it back. */
    longjmp(*escape, 99);
}

/* --- the socket ---------------------------------------------------------- */

static int with_client(int fd)
{
    return fd == AK_HOST_NET_CLIENT_FD;
}

int ak_host_net_socket(int domain, int type, int protocol)
{
    (void)domain;
    (void)type;
    (void)protocol;
    if (refuse_socket) {
        return -1;
    }
    sockets++;
    return 3;
}

int ak_host_net_setsockopt(int fd, int level, int option, const void *value,
                           socklen_t length)
{
    (void)fd;
    (void)level;
    (void)length;
    if (option == SO_REUSEADDR && value != NULL) {
        socket_reuseaddr = *(const int *)value;
    }
    return 0;
}

int ak_host_net_bind(int fd, const struct sockaddr *address, socklen_t length)
{
    (void)fd;
    (void)length;
    if (refuse_bind) {
        return -1; /* somebody else holds the port */
    }
    const struct sockaddr_in *in = (const struct sockaddr_in *)address;

    /* The port hands over a network-order port, as lwip wants; this undoes it
     * so the check can compare with the number the console prints. */
    socket_bound_port = (int)htons(in->sin_port);
    socket_bound_any = (in->sin_addr.s_addr == INADDR_ANY);
    return 0;
}

int ak_host_net_listen(int fd, int backlog)
{
    (void)fd;
    if (refuse_listen) {
        return -1;
    }
    socket_backlog = backlog;
    return 0;
}

int ak_host_net_accept(int fd, struct sockaddr *address, socklen_t *length)
{
    (void)fd;
    (void)address;
    (void)length;

    if (refuse_accept) {
        /* A client that went away between the select and the call - *once*,
         * because the port's answer is a `continue` and a model that failed
         * every call would spin in that loop for ever, which is a test that
         * hangs rather than a port that is wrong. */
        refuse_accept = 0;
        return -1;
    }

    if (client_done) {
        /* The loop has finished with the first client and is waiting for a
         * second, which is where the test takes its task back. */
        if (escape_env != NULL) {
            longjmp(*escape_env, 1);
        }
        return -1;
    }
    return AK_HOST_NET_CLIENT_FD;
}

int ak_host_net_select(int nfds, fd_set *readable, fd_set *writable,
                       fd_set *except, struct timeval *timeout)
{
    (void)nfds;
    (void)writable;
    (void)except;

    selects++;
    if (timeout != NULL) {
        select_wait_ms = (unsigned)(timeout->tv_sec * 1000L + timeout->tv_usec / 1000L);
    }

    /* The flight loop is the side that is free here: whatever it writes now
     * has to go out on the socket in the same pass. */
    if (busy_hook != NULL) {
        busy_hook();
    }

    if (selects == 1u) {
        /* A quiet client: the pass times out, and the loop's answer still has
         * to leave. */
        select_timeouts++;
        return 0;
    }
    if (readable != NULL) {
        FD_ZERO(readable);
        FD_SET(AK_HOST_NET_CLIENT_FD, readable);
    }
    return 1;
}

ssize_t ak_host_net_recv(int fd, void *buffer, size_t length, int flags)
{
    (void)flags;
    if (!with_client(fd)) {
        return -1;
    }
    unsigned left = client_length - client_at;
    if (left == 0u) {
        client_done = 1;
        return 0; /* the client hung up */
    }
    /* Bounded by this file's own array sizes, so the cast back to the counter's
     * type is exact. */
    size_t take = left < length ? left : length;
    memcpy(buffer, client_bytes + client_at, take);
    client_at += (unsigned)take;
    return (int)take;
}

ssize_t ak_host_net_send(int fd, const void *buffer, size_t length, int flags)
{
    (void)flags;
    if (!with_client(fd)) {
        return -1;
    }
    const uint8_t *bytes = buffer;
    size_t take = length;
    if (sent_bytes + take > AK_HOST_NET_SENT_BYTES) {
        take = AK_HOST_NET_SENT_BYTES - sent_bytes;
    }
    memcpy(sent + sent_bytes, bytes, take);
    sent_bytes += (unsigned)take;
    return (int)take;
}

int ak_host_net_close(int fd)
{
    (void)fd;
    closes++;
    return 0;
}

uint32_t ak_host_net_htonl(uint32_t host)
{
    return ((host & 0x000000FFu) << 24) | ((host & 0x0000FF00u) << 8) |
           ((host & 0x00FF0000u) >> 8) | ((host & 0xFF000000u) >> 24);
}

uint16_t ak_host_net_htons(uint16_t host)
{
    return (uint16_t)(((host & 0x00FFu) << 8) | ((host & 0xFF00u) >> 8));
}

/* --- what the test reads back -------------------------------------------- */

int ak_host_net_wifi_inited(void) { return wifi_inited; }
int ak_host_net_storage(void) { return wifi_storage; }
int ak_host_net_mode(void) { return wifi_mode; }
unsigned ak_host_net_wifi_starts(void) { return wifi_starts; }
unsigned ak_host_net_connects(void) { return wifi_connects; }
int ak_host_net_config_if(void) { return wifi_config_if; }
const char *ak_host_net_sta_ssid(void) { return sta_ssid; }
const char *ak_host_net_sta_password(void) { return sta_password; }
int ak_host_net_sta_authmode(void) { return sta_authmode; }
const char *ak_host_net_ap_ssid(void) { return ap_ssid; }
const char *ak_host_net_ap_password(void) { return ap_password; }
int ak_host_net_ap_ssid_len(void) { return ap_ssid_len; }
int ak_host_net_ap_max_connection(void) { return ap_max_connection; }
int ak_host_net_ap_authmode(void) { return ap_authmode; }
unsigned ak_host_net_netif_sta(void) { return netif_sta_count; }
unsigned ak_host_net_netif_ap(void) { return netif_ap_count; }
unsigned ak_host_net_ip_reads(void) { return ip_reads; }
unsigned ak_host_net_registrations(void) { return event_count; }
unsigned ak_host_net_tasks(void) { return tasks; }
const char *ak_host_net_task_name(void) { return task_name; }
unsigned ak_host_net_task_stack(void) { return task_stack; }
unsigned ak_host_net_task_priority(void) { return task_priority; }
unsigned ak_host_net_sockets(void) { return sockets; }
int ak_host_net_reuseaddr(void) { return socket_reuseaddr; }
int ak_host_net_bound_port(void) { return socket_bound_port; }
int ak_host_net_bound_any(void) { return socket_bound_any; }
int ak_host_net_backlog(void) { return socket_backlog; }
unsigned ak_host_net_closes(void) { return closes; }
const void *ak_host_net_sent(void) { return sent; }
unsigned ak_host_net_sent_bytes(void) { return sent_bytes; }
unsigned ak_host_net_selects(void) { return selects; }
unsigned ak_host_net_select_timeouts(void) { return select_timeouts; }
unsigned ak_host_net_select_wait_ms(void) { return select_wait_ms; }

void ak_host_net_client_send(const void *bytes, unsigned length)
{
    client_bytes = bytes;
    client_length = length;
    client_at = 0u;
    client_done = 0;
}

void ak_host_net_on_busy(void (*hook)(void))
{
    busy_hook = hook;
}

void ak_host_net_refuse_wifi_init(int refuse) { refuse_wifi_init = refuse; }
void ak_host_net_refuse_wifi_start(int refuse) { refuse_wifi_start = refuse; }
void ak_host_net_refuse_netif(int refuse) { refuse_netif = refuse; }
void ak_host_net_refuse_socket(int refuse) { refuse_socket = refuse; }
void ak_host_net_refuse_bind(int refuse) { refuse_bind = refuse; }
void ak_host_net_refuse_listen(int refuse) { refuse_listen = refuse; }
void ak_host_net_refuse_accept(int refuse) { refuse_accept = refuse; }
