#include "esp.h"

#include "esp_eth.h"
#include "esp_eth_mac_openeth.h"
#include "esp_eth_netif_glue.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "ak_console.h"
#include "ak_text.h"

#include <string.h>

/*
 * The network, which is the reason this target exists.
 *
 * The firmware's flight loop is a plain superloop that never blocks, so the
 * socket lives in a task of its own and the two exchange bytes through stream
 * buffers: the task never blocks the loop, and the loop never blocks on a
 * client that has gone quiet.
 *
 * A chip does not have "the network" - it has a radio and possibly a MAC - so
 * which one carries those bytes is a build choice (main/Kconfig.projbuild), and
 * this file is the only place the two are told apart. Above them everything is
 * identical: the same netif, the same lwip, the same socket, the same port, the
 * same protocol. QEMU, which is where this is verified, has no radio, so that
 * check runs the Ethernet medium; a board in a field has no PHY, so it runs
 * Wi-Fi. Either way the core is told whether it has a network and whether
 * anybody is on it, and neither answer depends on which medium it is.
 */

#define AK_NET_PORT 5555

static StreamBufferHandle_t net_rx;
static StreamBufferHandle_t net_tx;
static volatile int net_ready;
static volatile int net_connected;
static esp_netif_t *net_netif;
/* Which role the radio ended up in, once it has been started: the build choice
 * says "wifi", and this says whether that radio is a station or the network
 * itself. Nothing above this file cares - the socket and the protocol are the
 * same either way - but the console has to say which, because "no address" from
 * an access point and from a station are different problems. */
/* Which of the two Wi-Fi roles came up, for the medium line on the console.
 * Compiled only into the Wi-Fi build: on the Ethernet build there is no role
 * to name, and a variable that nothing can set is a warning rather than
 * information. */
#if CONFIG_AK_NET_WIFI
static const char *net_role;
#endif

/*
 * The credentials, as parameters rather than as build configuration. The
 * values live here and the table points at them, so `set wifi_ssid`,
 * `save` and the config protocol all work the way they do for every other
 * setting - one configuration path, which is the decision the survey landed on
 * (docs/22-esp32-survey.md) and the reason the Kconfig entries are gone.
 */
static char     net_wifi_ssid[33];  /* what to join; empty means none */
static char     net_wifi_pass[64];  /* its password, and the AP's; secret */
static char     net_ap_ssid[33] = "AerialKit";
static uint32_t net_wifi_mode;      /* 0 auto, 1 access point, 2 station only */

unsigned ak_esp_net_params(ak_param_t *items, unsigned count);

unsigned ak_esp_net_params(ak_param_t *items, unsigned count)
{
    count = ak_params_add_text(items, count, "wifi_ssid",
                               "network to join; empty means be one",
                               net_wifi_ssid, 32u, 0u, "", AK_PARAM_GROUP_NETWORK);
    count = ak_params_add_text(items, count, "wifi_pass",
                               "WPA2 password, station and access point",
                               net_wifi_pass, 63u, AK_PARAM_SECRET, "", AK_PARAM_GROUP_NETWORK);
    count = ak_params_add_text(items, count, "wifi_ap_ssid",
                               "the name this aircraft broadcasts",
                               net_ap_ssid, 32u, 0u, "AerialKit", AK_PARAM_GROUP_NETWORK);
    count = ak_params_add_u32(items, count, "wifi_mode",
                              "0 auto, 1 access point, 2 join only",
                              &net_wifi_mode, 0u, 2u, AK_PARAM_GROUP_NETWORK);
    return count;
}

/* Says where the firmware ended up listening, in the firmware's own console
 * output. A network that is up but unreachable is the same as no network, and
 * the address is the only way to tell which one this is - so the address is
 * printed when it arrives rather than checked once and guessed at. */
static void net_report_address(void)
{
    esp_netif_ip_info_t info;
    if (net_netif == NULL || esp_netif_get_ip_info(net_netif, &info) != ESP_OK ||
        info.ip.addr == 0) {
        ak_console_printf("net: %s, listening on %d, no address yet\n",
                          ak_esp_net_medium(), AK_NET_PORT);
        return;
    }

    ak_console_printf("net: %s, %u.%u.%u.%u:%d\n", ak_esp_net_medium(),
                      (unsigned)((info.ip.addr >> 0) & 0xFFu),
                      (unsigned)((info.ip.addr >> 8) & 0xFFu),
                      (unsigned)((info.ip.addr >> 16) & 0xFFu),
                      (unsigned)((info.ip.addr >> 24) & 0xFFu),
                      AK_NET_PORT);
}

static void net_on_ip(void *argument, esp_event_base_t base, int32_t id,
                      void *data)
{
    (void)argument;
    (void)data;
    if (base == IP_EVENT &&
        (id == IP_EVENT_ETH_GOT_IP || id == IP_EVENT_STA_GOT_IP)) {
        net_report_address();
    }
}

const char *ak_esp_net_medium(void)
{
#if CONFIG_AK_NET_WIFI
    return net_role != 0 ? net_role : "wifi";
#else
    return "ethernet";
#endif
}

static void net_server_task(void *argument)
{
    (void)argument;

    int listener = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listener < 0) {
        ESP_LOGE("ak-net", "no socket");
        vTaskDelete(NULL);
        return;
    }

    int reuse = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof reuse);

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(AK_NET_PORT);
    if (bind(listener, (struct sockaddr *)&address, sizeof address) != 0 ||
        listen(listener, 1) != 0) {
        ESP_LOGE("ak-net", "cannot listen on %d", AK_NET_PORT);
        close(listener);
        vTaskDelete(NULL);
        return;
    }

    net_ready = 1;
    net_report_address();
    for (;;) {
        struct sockaddr_in peer;
        socklen_t peer_length = sizeof peer;
        int client = accept(listener, (struct sockaddr *)&peer, &peer_length);
        if (client < 0) {
            continue;
        }
        net_connected = 1;
        xStreamBufferReset(net_rx);
        xStreamBufferReset(net_tx);

        for (;;) {
            uint8_t buffer[128];
            fd_set readable;
            FD_ZERO(&readable);
            FD_SET(client, &readable);
            /* 2000 *microseconds* - a timeval's field is microseconds, not
             * milliseconds - so a pass with a silent client still leaves within
             * two of them. Short and bounded on purpose: an answer the flight
             * loop wrote must not wait for the client to say something, and the
             * wait must not be a block. */
            struct timeval timeout = { 0, 2000 };
            if (select(client + 1, &readable, NULL, NULL, &timeout) > 0) {
                int got = recv(client, buffer, sizeof buffer, 0);
                if (got <= 0) {
                    break;
                }
                xStreamBufferSend(net_rx, buffer, (size_t)got, 0);
            }

            size_t pending;
            while ((pending = xStreamBufferReceive(net_tx, buffer,
                                                   sizeof buffer, 0)) > 0) {
                if (send(client, buffer, pending, 0) < 0) {
                    break;
                }
            }
        }

        net_connected = 0;
        close(client);
    }
}

/*
 * Wi-Fi, as a station.
 *
 * Station only. The aircraft joins a network somebody else runs; it never
 * creates one. That is the difference between a port that is reachable from
 * the machine with the configurator on it and a port that is its own little
 * island, and it is also the reason nothing new is listening on the air.
 */
#if CONFIG_AK_NET_WIFI
static void net_on_wifi(void *argument, esp_event_base_t base, int32_t id,
                        void *data)
{
    (void)argument;
    (void)data;

    /*
     * A router that reboots, or an aircraft carried out of range and back,
     * leaves a station that is up and associated with nothing. IDF does not
     * reconnect by itself, so without this the firmware would report "no
     * address" for the rest of the flight - and the socket would be gone with
     * it, because lwip closes the client's sockets when the link drops.
     */
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        net_connected = 0;
        (void)esp_wifi_connect();
    }
}

/* The shortest password WPA2 will take. Anything less and the access point is
 * either open or absent, and this board is not allowed to be either: the
 * protocol has no authentication of its own, so the *link* is the only thing
 * standing between a passer-by and a running aircraft. */
#define AK_NET_AP_PASSWORD_MIN 8u

/*
 * A string into a fixed field, always terminated, and the length it actually
 * copied.
 *
 * This was `strncpy(dst, src, sizeof dst - 1)` in four places, which is the
 * same thing only because the config was zeroed first - and the compiler says
 * so on every build with -Wall, four times. It also cannot answer the question
 * the access point's `ssid_len` asks, which is how the name *after* the copy
 * came to be able to disagree with the name that was copied: a name of 32
 * characters is truncated into a 32-byte field (31 characters and a terminator)
 * and the length field said 32.
 */
static unsigned net_copy(char *to, unsigned size, const char *from)
{
    unsigned i = 0u;

    while (i + 1u < size && from[i] != '\0') {
        to[i] = from[i];
        i++;
    }
    to[i] = '\0';
    return i;
}

static int net_start_station(void)
{
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();

    if (netif == NULL) {
        ESP_LOGW("ak-net", "no Wi-Fi netif");
        return 0;
    }

    wifi_config_t wifi;
    memset(&wifi, 0, sizeof wifi);
    net_copy((char *)wifi.sta.ssid, sizeof wifi.sta.ssid, net_wifi_ssid);
    net_copy((char *)wifi.sta.password, sizeof wifi.sta.password, net_wifi_pass);
    /* WPA2 or better. An open network is a network anybody can command an
     * aircraft over. */
    wifi.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK ||
        esp_wifi_set_config(WIFI_IF_STA, &wifi) != ESP_OK ||
        esp_wifi_start() != ESP_OK) {
        ESP_LOGW("ak-net", "the station did not start");
        return 0;
    }

    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                               net_on_wifi, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, net_on_ip, NULL);

    net_netif = netif;
    net_role = "wifi station";

    /* Association takes seconds and completes on the event loop. The socket
     * does not wait for it: binding to INADDR_ANY needs no address, so the
     * listener is ready the moment the address arrives. */
    esp_wifi_connect();
    return 1;
}

/*
 * And as an access point, which is the mode a board with no network of its own
 * to join comes up in: the aircraft is its own network, at a known address,
 * with nothing to type at the field beyond the password somebody set on the
 * bench. It is *not* open - see AK_NET_AP_PASSWORD_MIN - and a board told to
 * be an access point without a usable password says so and stays off the air
 * rather than coming up as one anybody can command.
 */
static int net_start_access_point(void)
{
    if (ak_strlen(net_wifi_pass) < AK_NET_AP_PASSWORD_MIN) {
        ak_console_write("net:       no password for the access point - set "
                         "wifi_pass (at least 8 characters) and 'save'\r\n");
        ESP_LOGW("ak-net", "access point refused: wifi_pass too short");
        return 0;
    }

    esp_netif_t *netif = esp_netif_create_default_wifi_ap();
    if (netif == NULL) {
        ESP_LOGW("ak-net", "no Wi-Fi netif");
        return 0;
    }

    wifi_config_t wifi;
    memset(&wifi, 0, sizeof wifi);
    net_copy((char *)wifi.ap.ssid, sizeof wifi.ap.ssid, net_ap_ssid);
    net_copy((char *)wifi.ap.password, sizeof wifi.ap.password, net_wifi_pass);
    /* The length is what was copied, not what was asked for: they differ for
     * any name longer than the field, and a broadcast that claims bytes it does
     * not hold is a network nobody can join. */
    wifi.ap.ssid_len = (uint8_t)ak_strlen((char *)wifi.ap.ssid);
    wifi.ap.max_connection = 4u;
    wifi.ap.authmode = WIFI_AUTH_WPA2_PSK;

    if (esp_wifi_set_mode(WIFI_MODE_AP) != ESP_OK ||
        esp_wifi_set_config(WIFI_IF_AP, &wifi) != ESP_OK ||
        esp_wifi_start() != ESP_OK) {
        ESP_LOGW("ak-net", "the access point did not start");
        return 0;
    }

    esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, net_on_ip,
                               NULL);
    net_netif = netif;
    net_role = "wifi access point";
    return 1;
}

static int net_start_wifi(void)
{
    /* What the parameters say: join a network, be one, or decide from whether
     * a network name was ever stored. The third case is the one that matters
     * on a bench - a board nobody has configured should still be reachable. */
    int station = net_wifi_mode == 2u ||
                  (net_wifi_mode == 0u && net_wifi_ssid[0] != '\0');
    int access_point = net_wifi_mode == 1u ||
                       (net_wifi_mode == 0u && net_wifi_ssid[0] == '\0');

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&init) != ESP_OK) {
        ESP_LOGW("ak-net", "the Wi-Fi driver did not initialise");
        return 0;
    }

    /* The credentials are parameters, and the driver has no reason to write
     * them anywhere: flash is for the configuration record `save` owns, and a
     * controller that rewrites its own flash on every boot is one that wears
     * it out. */
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    if (station && net_start_station()) {
        return 1;
    }
    if (access_point) {
        return net_start_access_point();
    }

    /* Told to join, with nothing to join: said out loud rather than left as a
     * board that quietly never gets an address, which is the failure this
     * whole file exists to avoid. */
    ESP_LOGW("ak-net", "no Wi-Fi network configured");
    ak_console_write("net:       no network to join - set wifi_ssid and "
                     "wifi_pass, or wifi_mode 1 for an access point\r\n");
    return 0;
}
#else
/*
 * The emulated Ethernet MAC, which is what QEMU has and what the automated
 * check therefore runs on.
 */
static int net_start_openeth(void)
{
    /* The netif is built from IDF's defaults rather than from a helper that
     * creates one, because it is not the only netif this file can build. */
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *netif = esp_netif_new(&netif_config);
    if (netif == NULL) {
        ESP_LOGW("ak-net", "no ethernet netif");
        return 0;
    }

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_openeth(&mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_dp83848(&phy_config);
    if (mac == NULL || phy == NULL) {
        ESP_LOGW("ak-net", "no emulated MAC");
        return 0;
    }

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth = NULL;
    if (esp_eth_driver_install(&eth_config, &eth) != ESP_OK) {
        ESP_LOGW("ak-net", "the ethernet driver did not install");
        return 0;
    }

    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth);
    if (glue == NULL || esp_netif_attach(netif, glue) != ESP_OK ||
        esp_eth_start(eth) != ESP_OK) {
        ESP_LOGW("ak-net", "the netif did not attach");
        return 0;
    }
    net_netif = netif;

    /* The address arrives long after this call returns, on an event, which is
     * where the console hears about it. */
    esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, net_on_ip, NULL);
    return 1;
}
#endif

/*
 * Bring the link up and put an address on it.
 *
 * The netif wants an event loop before anything can be attached to it, and the
 * DHCP client inside it is what turns the medium into an address the host can
 * reach. Both are IDF's. What is ours is which medium, and the decision to keep
 * going when it fails: a flight controller that will not boot without a network
 * is one that cannot be flown without one, so a failed network leaves the
 * aircraft flying with a console and no configurator.
 */
#ifdef AK_HOST_NET
/*
 * The host build's way to ask again.
 *
 * Like the SPI, I2C and ADC ports, this one keeps process-global state: the
 * stream buffers, whether the socket is listening, and whether a client is
 * connected. A board starts the network once; a test that wants a *failed*
 * start after a successful one has to be able to begin again, or it would be
 * checking the port's memory of the first attempt. Compiled only for the host
 * build (the Makefile passes -DAK_HOST_NET on this object alone).
 */
void ak_esp_net_forget_for_host(void)
{
    net_rx = 0;
    net_tx = 0;
    net_ready = 0;
    net_connected = 0;
    net_netif = 0;
}
#endif

void ak_esp_net_start(void)
{
    net_rx = xStreamBufferCreate(512, 1);
    net_tx = xStreamBufferCreate(1024, 1);
    if (net_rx == NULL || net_tx == NULL) {
        return;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if CONFIG_AK_NET_WIFI
    if (!net_start_wifi()) {
        return;
    }
#else
    if (!net_start_openeth()) {
        return;
    }
#endif

    /* Separate task, and a small stack: this one only ever waits on a socket. */
    xTaskCreate(net_server_task, "ak-net", 4096, NULL, 4, NULL);
}

int ak_esp_net_poll_rx(uint8_t *byte)
{
    if (net_rx == NULL) {
        return 0;
    }
    return xStreamBufferReceive(net_rx, byte, 1, 0) == 1 ? 1 : 0;
}

unsigned ak_esp_net_write(const uint8_t *data, unsigned length)
{
    if (net_tx == NULL || !net_connected) {
        return 0;
    }
    return (unsigned)xStreamBufferSend(net_tx, data, (size_t)length, 0);
}

int ak_esp_net_connected(void)
{
    return net_connected;
}

int ak_esp_net_ready(void)
{
    return net_ready;
}

void ak_esp_net_report(ak_printf_fn out)
{
    esp_netif_ip_info_t info;
    if (!net_ready) {
        out("net:       %s, not listening\n", ak_esp_net_medium());
        return;
    }
    if (net_netif != NULL &&
        esp_netif_get_ip_info(net_netif, &info) == ESP_OK &&
        info.ip.addr != 0) {
        out("net:       %s, %u.%u.%u.%u:%d, %s\n", ak_esp_net_medium(),
            (unsigned)((info.ip.addr >> 0) & 0xFFu),
            (unsigned)((info.ip.addr >> 8) & 0xFFu),
            (unsigned)((info.ip.addr >> 16) & 0xFFu),
            (unsigned)((info.ip.addr >> 24) & 0xFFu), AK_NET_PORT,
            net_connected ? "a client is attached" : "no client");
        return;
    }
    out("net:       %s, listening on %d, no address\n", ak_esp_net_medium(),
        AK_NET_PORT);
}
