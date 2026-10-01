#ifndef AK_HOST_ESP32_NET_MODEL_H
#define AK_HOST_ESP32_NET_MODEL_H

#include <setjmp.h>
#include <stdint.h>

/*
 * The ESP32's network, run on the host against a modelled IDF and a modelled
 * client.
 *
 * `src/arch/esp32/net.c` is the reason that target exists, and it is the one
 * file on it that had never been run anywhere this machine can see: QEMU's
 * ESP32 has no radio, so the end-to-end check builds the *Ethernet* medium and
 * everything under `#if CONFIG_AK_NET_WIFI` - the two roles, the credentials,
 * the WPA2 floor, the address, the reconnect after a router reboots - has only
 * ever been compiled. That is the same hole the ESP32's ADC, its buses and the
 * fitted half of a board file each had, and the same one `net.c` had for the
 * same reason: the part that does not run is the part a bench has to be driven
 * to, and a bench session is expensive.
 *
 * So the real file is compiled as the Wi-Fi build against stand-in IDF headers
 * (tests/idf-stub/) and this model behind them: a radio that records what it
 * was asked to do, a netif whose address the test can set, an event loop the
 * test raises events through, stream buffers that are really buffers, and a
 * socket that plays a client - one that goes quiet, then sends a frame, then
 * hangs up, which is the sequence the task's loop exists to survive.
 */

void ak_host_net_reset(void);

/* What the port asked the radio to do. `-1` means "never asked". */
int      ak_host_net_wifi_inited(void);
int      ak_host_net_storage(void);      /* WIFI_STORAGE_RAM, one hopes */
int      ak_host_net_mode(void);         /* WIFI_MODE_STA or WIFI_MODE_AP */
unsigned ak_host_net_wifi_starts(void);
unsigned ak_host_net_connects(void);
int      ak_host_net_config_if(void);    /* WIFI_IF_STA / WIFI_IF_AP / -1 */
const char *ak_host_net_sta_ssid(void);
const char *ak_host_net_sta_password(void);
int      ak_host_net_sta_authmode(void);
const char *ak_host_net_ap_ssid(void);
const char *ak_host_net_ap_password(void);
int      ak_host_net_ap_ssid_len(void);
int      ak_host_net_ap_max_connection(void);
int      ak_host_net_ap_authmode(void);

/* The netif: how many were made, in which role, and how often the port has
 * read the address off one - which is what the line it prints is built from. */
unsigned ak_host_net_netif_sta(void);
unsigned ak_host_net_netif_ap(void);
unsigned ak_host_net_ip_reads(void);
unsigned ak_host_net_registrations(void);

/* Failures a bench cannot arrange on purpose. */
void ak_host_net_refuse_wifi_init(int refuse);
void ak_host_net_refuse_wifi_start(int refuse);
void ak_host_net_refuse_netif(int refuse);
void ak_host_net_refuse_socket(int refuse);
/* And the listener itself refusing: the socket is made and the bind or the
 * listen is what fails, which on a board is a port another service already
 * holds. */
void ak_host_net_refuse_bind(int refuse);
void ak_host_net_refuse_listen(int refuse);
/* Or an accept that fails once with the loop still running, which is a client
 * that went away between the select and the call. */
void ak_host_net_refuse_accept(int refuse);

/* The address the netif reports: 0.0.0.0 means DHCP has not answered yet. */
void ak_host_net_set_address(unsigned a, unsigned b, unsigned c, unsigned d);

/* Raise what a chip's radio raises: the address arriving, the link dropping. */
void ak_host_net_raise(const char *base, int id);

/* The socket task, as the port created it. */
unsigned    ak_host_net_tasks(void);
const char *ak_host_net_task_name(void);
unsigned    ak_host_net_task_stack(void);
unsigned    ak_host_net_task_priority(void);

/*
 * Run that task's body, and come back through `escape` when the model reaches
 * its second `accept()`.
 *
 * The task is a `for (;;)`: it never returns, so a test that called it would
 * never see its own next line. Waiting for a *second* client is the point at
 * which the loop has provably finished with the first one, and it is where
 * this hands control back. `longjmp` out of it is deliberate - the alternative
 * is a threaded test that hangs when the port is wrong, which is the failure
 * mode the check exists to prevent.
 */
void ak_host_net_run_task(jmp_buf *escape);

/* The socket the task built, and what the model saw done to it. */
unsigned ak_host_net_sockets(void);
int      ak_host_net_reuseaddr(void);   /* the option it was set with, or -1 */
int      ak_host_net_bound_port(void);  /* in host order, or -1 */
int      ak_host_net_bound_any(void);
int      ak_host_net_backlog(void);
unsigned ak_host_net_closes(void);

/* The client at the other end. `send` is what it sends; when those bytes run
 * out it hangs up, which is what makes the task's inner loop end. */
void ak_host_net_client_send(const void *bytes, unsigned length);

/* Called inside `select()`, i.e. while the flight loop is the side that is
 * free - the moment a person at the console would type. What the port's own
 * write path does with it is the check. */
void ak_host_net_on_busy(void (*hook)(void));

/* What the port pushed at the client, and what the socket was told to do. */
const void *ak_host_net_sent(void);
unsigned    ak_host_net_sent_bytes(void);
unsigned    ak_host_net_selects(void);
unsigned    ak_host_net_select_timeouts(void);   /* passes that timed out */
unsigned    ak_host_net_select_wait_ms(void);    /* the timeout it asked for */

#endif /* AK_HOST_ESP32_NET_MODEL_H */
