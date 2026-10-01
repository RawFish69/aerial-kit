/*
 * The ESP32's network, in the configuration a board is in and this machine
 * cannot be.
 *
 * `src/arch/esp32/net.c` builds the medium at compile time. QEMU's ESP32 has
 * no radio, so every automated run of that file has been the *Ethernet* build:
 * the emulated MAC, an address from QEMU's DHCP, a real socket, and
 * `docs/evidence/esp32-network.txt` as the transcript. Everything under
 * `#if CONFIG_AK_NET_WIFI` - which role the radio is put in, what the
 * credentials are set to, the WPA2 floor, the access point that refuses to be
 * open, the reconnect after a router reboots - has been *compiled* and never
 * run, on this machine or anywhere else, because the owner's ESP32 board is
 * one of the two answers this work is parked on.
 *
 * So this file runs it: the real net.c as the Wi-Fi build, against the stand-in
 * IDF headers in tests/idf-stub and the model in tests/host_esp32_net_model.c.
 * What it cannot say: that a radio does what it was told. What it can say is
 * what the firmware *tells* the radio, which is where the decisions are - and
 * the socket half below runs the port's own server task, one client at a time,
 * with the flight loop on the other side of the two stream buffers.
 */

#include "host_esp32_net_model.h"
#include "host_esp32_output_model.h" /* where the stand-in IDF log's lines live */

#include "ak_params.h"
#include "esp.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "tests.h"

#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* The console's text, through the seam the report takes rather than the board's
 * sink - the same way the log tests capture a dump. */
static char captured[512];
static unsigned captured_len;

/* The port's four parameters, in the same table a conversation with it would
 * use: the credentials are set by name, through the path a person's `set` takes
 * rather than by reaching into the port's own buffers. */
static ak_param_t items[8];
static ak_params_t table;
static int table_ready;

static int capture_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(captured + captured_len, sizeof captured - captured_len,
                      fmt, args);
    va_end(args);
    if (n > 0) {
        captured_len += (unsigned)n;
    }
    return n;
}

static void capture_reset(void)
{
    captured[0] = '\0';
    captured_len = 0u;
}

static int capture_has(const char *text)
{
    return strstr(captured, text) != NULL;
}

/* Run the port's socket task far enough for the listener to be open, then take
 * it back - the same hand-back the client check makes, with nothing to serve.
 *
 * Until this has happened once the port's own report says "not listening",
 * because nothing has bound the port yet; a check that wants the other lines
 * has to let the task run. net_ready is a static in net.c and lives for the
 * process, so the check that wants "not listening" comes before any of these. */
static void listen_once(void)
{
    jmp_buf escape;
    if (setjmp(escape) == 0) {
        ak_host_net_run_task(&escape);
    }
}

/* What the test writes while the socket task is waiting on select(), i.e. as
 * the flight loop would when a command arrives. */
static int answered;
static void answer_while_waiting(void)
{
    /* Once: what is being checked is that an answer the flight loop wrote while
     * the client was quiet went out on the same pass, not how many times a
     * hook can be called. */
    if (!answered) {
        answered = 1;
        (void)ak_esp_net_write((const uint8_t *)"aa 55 01 status", 15u);
    }
}

/* The host build's way to ask the port to start again: see the seam in
 * src/arch/esp32/net.c. A board starts its network once; a test that wants a
 * *failed* start after a successful one has to forget the first. */
void ak_esp_net_forget_for_host(void);

/* Whether the port said this at any point. The last line is not enough: a
 * failure that is *reported* can be followed by another line, because the port
 * carries on to the next question - which is what a firmware that does not give
 * up on a bench mistake looks like. */
static int log_says(const char *needle)
{
    for (unsigned i = 0; i < ak_host_idf_log_count(); i++) {
        if (strstr(ak_host_idf_log_text(i), needle) != NULL) {
            return 1;
        }
    }
    return 0;
}

static void start_fresh(void)
{
    ak_esp_net_forget_for_host();
    ak_host_net_reset();
    ak_host_idf_reset();   /* the log lines the stub records: the checks read them */

    /* The table is registered *once*, the way it is at boot, and that matters:
     * a parameter's default is its value when it is registered, so a table
     * rebuilt after a check had changed something would record the changed
     * value as the default and quieten the reset below. */
    if (!table_ready) {
        ak_params_init(&table, items, ak_esp_net_params(items, 0u));
        table_ready = 1;
    }

    /* The port's settings are statics in net.c with no reset of their own, so
     * every check starts from the defaults - which is what a board has at boot,
     * and without which a mode left set by the previous check is the mode this
     * one silently tests. That is not hypothetical: one of these checks passed
     * while testing the access point instead of the station. */
    ak_params_reset(&table);
    capture_reset();
}

static void put(const char *name, const char *value)
{
    char message[64];
    (void)ak_params_set(&table, name, value, message, sizeof message);
}

/* --- the parameters, which are the credentials --------------------------- */

static void test_the_credentials_are_parameters(void)
{
    start_fresh();

    expect("the network's four settings are in the parameter table",
           table.count == 4u && strcmp(items[0].name, "wifi_ssid") == 0 &&
               strcmp(items[1].name, "wifi_pass") == 0 &&
               strcmp(items[2].name, "wifi_ap_ssid") == 0 &&
               strcmp(items[3].name, "wifi_mode") == 0);

    /* The password is served *over the config port*, which is the thing
     * anybody with the address can read - so it is the one setting in the
     * table that has to be marked secret. */
    expect("and the password is the one marked secret",
           (items[1].flags & AK_PARAM_SECRET) != 0u &&
               (items[0].flags & AK_PARAM_SECRET) == 0u);

    expect("the access point a board nobody configured broadcasts has a name",
           strcmp(items[2].def_text, "AerialKit") == 0);
    expect("and the mode defaults to deciding from what was saved",
           items[3].def.u == 0u && items[3].max.u == 2u);

    /* And what a console does to them actually reaches the driver, which is
     * the next check's business - here, only that the table and the port's
     * buffers are the same storage. */
    put("wifi_ssid", "labnet");
    expect("setting the network name is remembered by the table",
           ak_params_find(&table, "wifi_ssid") != NULL);
}

/* --- which role the radio plays ------------------------------------------ */

static void test_a_board_with_credentials_joins_the_network(void)
{
    start_fresh();
    put("wifi_ssid", "labnet");
    put("wifi_pass", "hunter2hunter2");

    ak_esp_net_start();

    expect("a board with a network to join starts as a station",
           ak_host_net_mode() == WIFI_MODE_STA &&
               ak_host_net_netif_sta() == 1u && ak_host_net_netif_ap() == 0u);
    expect("and it is the credentials it was given that the radio gets",
           strcmp(ak_host_net_sta_ssid(), "labnet") == 0 &&
               strcmp(ak_host_net_sta_password(), "hunter2hunter2") == 0 &&
               ak_host_net_config_if() == WIFI_IF_STA);

    /* An open network is a network anybody can command an aircraft over, and
     * the protocol has no authentication of its own. So the station will not
     * accept anything weaker than WPA2, whatever the access point offers. */
    expect("the station refuses to associate with less than WPA2",
           ak_host_net_sta_authmode() == (int)WIFI_AUTH_WPA2_PSK);

    /* Flash is for the configuration record `save` owns. A driver that writes
     * its own copy on every boot is a controller wearing out its own flash. */
    expect("the driver is told not to keep the credentials in flash",
           ak_host_net_wifi_inited() && ak_host_net_storage() == WIFI_STORAGE_RAM);

    expect("and the socket task is up as soon as the radio is",
           ak_host_net_tasks() == 1u &&
               strcmp(ak_host_net_task_name(), "ak-net") == 0 &&
               ak_host_net_task_stack() == 4096u && ak_esp_net_ready() == 0);

    /* `ready` is 0 until the *socket* is listening, which happens in the task;
     * that the radio started is the task's starting being asked for, not a
     * claim that anything is reachable. */
    expect("the station asks to associate, and the medium says which role",
           ak_host_net_connects() == 1u &&
               strcmp(ak_esp_net_medium(), "wifi station") == 0);
    /* Two registrations: the address arriving, and the link dropping. Without
     * either, the console never says where the board is and the board never
     * rejoins a network that rebooted - which are the two failures this whole
     * file exists to avoid. */
    expect("and it asks to hear about the address and about losing the link",
           ak_host_net_registrations() == 2u);
}

static void test_a_board_without_one_becomes_one(void)
{
    start_fresh();
    put("wifi_mode", "1");            /* access point, explicitly */
    put("wifi_pass", "aircraftpass");

    ak_esp_net_start();

    expect("a board told to be a network brings one up",
           ak_host_net_mode() == WIFI_MODE_AP &&
               ak_host_net_netif_ap() == 1u && ak_host_net_config_if() == WIFI_IF_AP);
    expect("under the name it was given, with a real length beside it",
           strcmp(ak_host_net_ap_ssid(), "AerialKit") == 0 &&
               ak_host_net_ap_ssid_len() == 9 &&
               strcmp(ak_host_net_ap_password(), "aircraftpass") == 0);
    expect("and with room for the bench and nothing more",
           ak_host_net_ap_max_connection() == 4 &&
               ak_host_net_ap_authmode() == (int)WIFI_AUTH_WPA2_PSK);
    expect("the medium reports the role it actually came up in",
           strcmp(ak_esp_net_medium(), "wifi access point") == 0);

    /* And the name a person can actually type into that setting: the field
     * takes 31 characters and a terminator, so a 32-character name is cut - and
     * the length that goes to the radio has to be the length of what is
     * *there*. It used to be the length of what was asked for, which is a
     * network broadcasting bytes it does not hold. */
    start_fresh();
    put("wifi_mode", "1");
    put("wifi_pass", "aircraftpass");
    put("wifi_ap_ssid", "ABCDEFGHIJKLMNOPQRSTUVWXYZ012345");

    ak_esp_net_start();

    expect("an access point name too long for the field is cut and says so",
           strcmp(ak_host_net_ap_ssid(), "ABCDEFGHIJKLMNOPQRSTUVWXYZ01234") == 0 &&
               ak_host_net_ap_ssid_len() == 31);
}

static void test_an_open_aircraft_is_refused(void)
{
    /* Anybody who can reach the port can fly the aircraft: the protocol has no
     * authentication of its own, so the *link* is the only thing standing
     * between a passer-by and a running aircraft. */
    start_fresh();
    put("wifi_mode", "1");            /* be a network */
    put("wifi_pass", "short");        /* seven characters */

    ak_esp_net_start();

    expect("an access point without a real password does not come up at all",
           ak_host_net_netif_ap() == 0u && ak_host_net_wifi_starts() == 0u &&
               ak_esp_net_ready() == 0);
    expect("and it says why rather than failing quietly",
           log_says("too short"));

    /* And the radio is left exactly as it was: it is not put into a mode it
     * cannot serve, which is what "stays off the air" has to mean in practice -
     * a radio set to access point with no usable password is a network
     * somebody can join. */
    expect("the radio is initialised and then left alone",
           ak_host_net_wifi_inited() && ak_host_net_mode() == -1 &&
               ak_host_net_wifi_starts() == 0u);
}

static void test_told_to_join_with_nothing_to_join(void)
{
    uint8_t byte = 0u;

    start_fresh();
    put("wifi_mode", "2");            /* join only, and no name was saved */
    /* A real driver refuses to start a station with no SSID to join; the model
     * has to be told to, because otherwise the port's own decision looks like
     * a success and the branch below is never reached. */
    ak_host_net_refuse_wifi_start(1);

    ak_esp_net_start();

    /* This is the failure the file exists to avoid: a board that is powered,
     * looks alive, and is reachable by nobody. It says so instead. */
    expect("a board told to join a network it was never given says so",
           log_says("no Wi-Fi network configured"));
    expect("and it stops there rather than half coming up",
           ak_host_net_wifi_starts() == 0u && ak_esp_net_ready() == 0 &&
               ak_esp_net_poll_rx(&byte) == 0 &&
               ak_esp_net_write((const uint8_t *)"x", 1u) == 0u);
}

/*
 * The station's netif and the access point's radio: the two calls the *other*
 * role makes, so the two failure branches the test above leaves. A station that
 * cannot create a netif and an AP whose radio will not start are the same
 * promise as the rest of this file - the firmware says so and carries on with
 * the console.
 */
static void test_the_other_roles_failures(void)
{
    start_fresh();
    put("wifi_mode", "2");
    put("wifi_ssid", "labnet");
    put("wifi_pass", "aircraftpass");
    ak_host_net_refuse_netif(1);

    ak_esp_net_start();

    expect("a station with no netif says so and stops",
           log_says("no Wi-Fi netif") && ak_esp_net_ready() == 0);

    start_fresh();
    put("wifi_mode", "1");
    put("wifi_pass", "aircraftpass");
    ak_host_net_refuse_wifi_start(1);

    ak_esp_net_start();

    expect("an access point whose radio will not start says so",
           log_says("the access point did not start") &&
               ak_esp_net_ready() == 0);
}

/*
 * And the listener itself refusing, which on a board is a port another service
 * holds: the task has to end rather than spin, and say why. A client that goes
 * away between the select and the accept is the other half - the loop carries
 * on, which is what one failed accept is for.
 */
static void test_a_port_that_cannot_be_listened_on(void)
{
    start_fresh();
    put("wifi_ssid", "labnet");
    put("wifi_pass", "hunter2hunter2");
    ak_host_net_refuse_bind(1);
    ak_esp_net_start();
    listen_once();
    expect("a port somebody else holds is a listener that does not start",
           log_says("cannot listen") && ak_esp_net_ready() == 0);

    start_fresh();
    put("wifi_ssid", "labnet");
    put("wifi_pass", "hunter2hunter2");
    ak_host_net_refuse_listen(1);
    ak_esp_net_start();
    listen_once();
    expect("and the same for a listen the system refuses",
           log_says("cannot listen"));

    start_fresh();
    put("wifi_ssid", "labnet");
    put("wifi_pass", "hunter2hunter2");
    ak_host_net_refuse_accept(1);
    ak_esp_net_start();
    listen_once();
    expect("an accept that fails leaves the task running for the next client",
           ak_host_net_sockets() == 1u);
}

static void test_a_radio_that_will_not_start(void)
{
    start_fresh();
    ak_host_net_refuse_wifi_init(1);

    ak_esp_net_start();

    /* A flight controller that will not boot without a network is one that
     * cannot be flown without one. The console is what is left, and the
     * aircraft flies on it. */
    expect("a driver that will not initialise leaves the firmware running",
           ak_host_net_wifi_inited() == 0 && ak_host_net_tasks() == 0u &&
               ak_esp_net_ready() == 0 && ak_esp_net_connected() == 0);

    start_fresh();
    put("wifi_mode", "1");
    put("wifi_pass", "aircraftpass");
    ak_host_net_refuse_netif(1);

    ak_esp_net_start();
    expect("and so does a netif that cannot be created",
           ak_host_net_netif_ap() == 0u && ak_host_net_wifi_starts() == 0u);
}

/* --- the address, and the events that produce it ------------------------- */

static void test_the_address_arrives_on_an_event(void)
{
    start_fresh();
    put("wifi_ssid", "labnet");
    put("wifi_pass", "hunter2hunter2");
    ak_host_net_set_address(10u, 0u, 0u, 5u);

    unsigned reads_before = ak_host_net_ip_reads();
    ak_esp_net_start();

    /* The address is not there when the socket is bound - binding to
     * INADDR_ANY needs no address - so it arrives on an event, and the
     * handler's job is to read it off the netif. */
    ak_host_net_raise(IP_EVENT, IP_EVENT_STA_GOT_IP);
    expect("the address arriving makes the port read it off the netif",
           ak_host_net_ip_reads() > reads_before);

    /* The line itself is checked through the report path - the console's `net`
     * command calls the same function the event handler does - because this
     * binary's console sink is the F405's UART and does not record text. */
    listen_once();
    capture_reset();
    ak_esp_net_report(capture_printf);
    expect("the console says which medium, which address and which port",
           capture_has("10.0.0.5:5555") && capture_has("wifi station") &&
               capture_has("no client"));
}

/* Before any check has brought the port up. This comes first in the run order
 * because "not listening" is a state the port leaves and never returns to. */
static void test_the_report_before_anything_starts(void)
{
    start_fresh();
    capture_reset();
    ak_esp_net_report(capture_printf);
    expect("before anything starts the report says so, and which medium",
           capture_has("not listening") && capture_has("wifi"));
}

static void test_the_report_says_what_is_missing(void)
{
    /* A link that is up with no address is the failure a board at a field is
     * actually in when the configurator cannot find it, and it is not the same
     * thing as no network at all. */
    start_fresh();
    put("wifi_ssid", "labnet");
    put("wifi_pass", "hunter2hunter2");
    ak_esp_net_start();                 /* no address set: DHCP has not answered */
    listen_once();
    capture_reset();
    ak_esp_net_report(capture_printf);
    expect("a link that is up with no address says that, not silence",
           capture_has("no address") && capture_has("listening"));
}

static void test_a_router_that_reboots_is_joined_again(void)
{
    start_fresh();
    put("wifi_ssid", "labnet");
    put("wifi_pass", "hunter2hunter2");
    ak_esp_net_start();

    unsigned connects = ak_host_net_connects();
    ak_host_net_raise(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED);

    /* IDF does not reconnect by itself. Without this the firmware would report
     * "no address" for the rest of the flight after any dropout - an aircraft
     * carried out of range and back, a router that rebooted - and the
     * configurator would never find it again. */
    expect("a station that loses the network asks to join it again",
           ak_host_net_connects() == connects + 1u);
    expect("and it does not claim to have a client while it is off the air",
           ak_esp_net_connected() == 0);
}

/* --- the socket, which is what all of the above is for ------------------- */

static void test_the_server_task_serves_a_client(void)
{
    static const uint8_t request[15] = {
        'a', 'a', 0x55, 0x01, 0x08, 0x1a, 0xcc, 0x10,
        0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00,
    };
    jmp_buf escape;
    int stopped_at;
    uint8_t got[32];

    start_fresh();
    answered = 0;
    put("wifi_ssid", "labnet");
    put("wifi_pass", "hunter2hunter2");
    ak_host_net_client_send(request, sizeof request);
    ak_host_net_on_busy(answer_while_waiting);

    ak_esp_net_start();
    stopped_at = setjmp(escape);
    if (stopped_at == 0) {
        ak_host_net_run_task(&escape);
        return; /* run_task always leaves through `escape` */
    }
    expect("the test took its task back at the accept it was waiting in",
           stopped_at == 1);

    expect("the port listens on the port the console advertises",
           ak_host_net_sockets() == 1u && ak_host_net_bound_port() == 5555 &&
               ak_host_net_bound_any() == 1);
    expect("one client at a time, and the address is reusable after a reboot",
           ak_host_net_backlog() == 1 && ak_host_net_reuseaddr() == 1);

    /* A quiet client: the first pass through select() timed out, and the
     * answer the loop wrote during it still went out. */
    expect("a pass that no client spoke during still sends what the loop wrote",
           ak_host_net_select_timeouts() == 1u &&
               ak_host_net_sent_bytes() == 15u &&
               memcmp(ak_host_net_sent(), "aa 55 01 status", 15u) == 0);
    expect("and each pass is bounded by a two-millisecond wait, not a block",
           ak_host_net_select_wait_ms() == 2u && ak_host_net_selects() >= 3u);

    /* And the other direction: every byte the client sent is in the buffer the
     * flight loop drains, in order and with nothing added. */
    unsigned n = 0u;
    while (n < sizeof got && ak_esp_net_poll_rx(&got[n])) {
        n++;
    }
    expect("every byte the client sent reached the flight loop, in order",
           n == sizeof request && memcmp(got, request, sizeof request) == 0);

    /* The client hanging up is noticed rather than waited on: the task closed
     * it and went back to accept(), which is where this test took its task
     * back. */
    expect("a client that hangs up is closed and not waited for",
           ak_host_net_closes() == 1u && ak_esp_net_connected() == 0);
}

static void test_a_socket_that_cannot_be_made(void)
{
    jmp_buf escape;
    int stopped_at;

    start_fresh();
    put("wifi_ssid", "labnet");
    put("wifi_pass", "hunter2hunter2");
    ak_host_net_refuse_socket(1);

    ak_esp_net_start();
    stopped_at = setjmp(escape);
    if (stopped_at == 0) {
        ak_host_net_run_task(&escape);
    }

    /* No socket means no listener, so the aircraft is up with a console and
     * no configurator - and it does not sit in a loop with no way out. */
    expect("a listener that cannot be opened ends the task rather than looping",
           ak_host_net_sockets() == 0u && ak_host_net_selects() == 0u &&
               stopped_at == 99);
}

void test_arch_esp32_net(void)
{
    test_the_credentials_are_parameters();
    test_a_board_with_credentials_joins_the_network();
    test_a_board_without_one_becomes_one();
    test_an_open_aircraft_is_refused();
    test_told_to_join_with_nothing_to_join();
    test_a_radio_that_will_not_start();
    test_the_report_before_anything_starts();
    test_the_address_arrives_on_an_event();
    test_the_report_says_what_is_missing();
    test_a_router_that_reboots_is_joined_again();
    test_the_server_task_serves_a_client();
    test_a_socket_that_cannot_be_made();
    /* Last: these start a network of their own. */
    test_the_other_roles_failures();
    test_a_port_that_cannot_be_listened_on();
}
