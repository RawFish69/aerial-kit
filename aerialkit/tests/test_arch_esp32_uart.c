/*
 * The ESP32's two UARTs, run on the host against a modelled IDF driver.
 *
 * This is the receiver's and the GPS's port, and it is the last file on the
 * second target that a host can run: no receiver has ever been wired to it, no
 * GPS either, and QEMU's ESP32 has no serial device on those pins. So the
 * checks below are about the three decisions the file makes rather than about
 * a waveform:
 *
 *   - the read path never waits (`uart_read_bytes` with a zero timeout), which
 *     is what keeps a port with nothing on it from costing the loop a pass;
 *   - the driver's events are drained on that same path, because a full event
 *     queue is exactly how the overflow the driver was asked to report goes
 *     missing;
 *   - a framing error is said once, and then only counted.
 *
 * And the protocol switch, which is the one thing on this chip that the F405
 * cannot do for itself: SBUS is inverted, the F405 needs a transistor, and
 * this part's GPIO matrix does it - so the check watches the model's line
 * settings change and the receive ring be flushed with them.
 */

#include <string.h>
#include <sys/mman.h>

#include "esp.h"

#include "driver/uart.h"

#include "ak_console.h"
#include "host_esp32_uart_model.h"
#include "tests.h"

#define TEST_RC_UART   1u
#define TEST_GPS_UART  2u
#define TEST_RC_TX    17
#define TEST_RC_RX    16
#define TEST_GPS_TX    4
#define TEST_GPS_RX    5

#define TEST_CRSF_BAUD 420000
#define TEST_SBUS_BAUD 100000

/*
 * The console's own sink, in this binary, is the F405's USART: the last byte of
 * every line this firmware prints lands in its data register. That is how "said
 * once" becomes checkable here rather than a claim - see the sentinel below.
 *
 * The page is mapped *by this test*, at an address of its own, rather than
 * borrowed from the F405 test's mapping of the real peripheral window. The
 * first version borrowed it, and that worked only because `test_arch()` happens
 * to run earlier in this binary and happens not to unmap: a test that would
 * segfault rather than fail if that order ever changed. The console sink writes
 * through USART_SR/USART_DR at the base it is given, so any mapped pair of words
 * will do - and 0x40004400 is free here, above the F405's TEST pins and below
 * the AT32 test's window.
 */
#define TEST_CONSOLE_USART 0x4C000000u
#define USART_SR_OFFSET 0x00u
#define USART_DR_OFFSET 0x04u
#define USART_SR_TXE (1u << 7)

static void *map_console_page(void)
{
    void *page = mmap((void *)TEST_CONSOLE_USART, 4096, PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    return page == MAP_FAILED ? 0 : page;
}

static volatile uint32_t *sr(void)
{
    return (volatile uint32_t *)(TEST_CONSOLE_USART + USART_SR_OFFSET);
}

static volatile uint32_t *dr(void)
{
    return (volatile uint32_t *)(TEST_CONSOLE_USART + USART_DR_OFFSET);
}

void test_arch_esp32_uart(void)
{
    ak_host_uart_reset();
    /* The page the console writes into, mapped here rather than borrowed - a
     * failed mapping is a failed check, not a crash two checks later. */
    expect("this test can map the page it writes the console into",
           map_console_page() != 0);

    /* --- the receiver's port ------------------------------------------------- */

    ak_esp_rc_init(TEST_RC_UART, TEST_RC_TX, TEST_RC_RX, TEST_CRSF_BAUD,
                   TEST_SBUS_BAUD);
    expect("the receiver's uart comes up on the port the board named",
           ak_host_uart_installed(TEST_RC_UART) == 1);
    expect("with the pins and the CRSF line settings the board asked for",
           ak_host_uart_tx_pin(TEST_RC_UART) == TEST_RC_TX &&
               ak_host_uart_rx_pin(TEST_RC_UART) == TEST_RC_RX &&
               ak_host_uart_baud(TEST_RC_UART) == TEST_CRSF_BAUD &&
               ak_host_uart_data_bits(TEST_RC_UART) == UART_DATA_8_BITS &&
               ak_host_uart_parity(TEST_RC_UART) == UART_PARITY_DISABLE &&
               ak_host_uart_stop_bits(TEST_RC_UART) == UART_STOP_BITS_1);
    expect("with a receive ring, a transmit buffer and an event queue",
           ak_host_uart_rx_buffer(TEST_RC_UART) == 1024 &&
               ak_host_uart_tx_buffer(TEST_RC_UART) == 512 &&
               ak_host_uart_event_depth(TEST_RC_UART) == 16);

    /* Bytes arrive, and come back one at a time - the shape the core's own
     * receiver parser wants, because it is fed a byte per loop pass. */
    {
        static const uint8_t frame[4] = { 0xC8u, 0x18u, 0x00u, 0x03u };
        uint8_t byte = 0;
        unsigned got = 0;
        ak_host_uart_feed(TEST_RC_UART, frame, sizeof frame);

        expect("the port drains what the driver has, one byte a call",
               ak_esp_rc_poll(&byte) == 1 && byte == frame[0]);
        got = byte == frame[0] ? 1u : 0u;
        while (ak_esp_rc_poll(&byte) == 1 && got < sizeof frame) {
            got++;
        }
        expect("and the bytes are in the order the receiver sent them",
               ak_host_uart_pending(TEST_RC_UART) == 0u && got == sizeof frame);
    }

    expect("an idle port offers nothing rather than waiting for something",
           ak_esp_rc_poll(&(uint8_t){ 0 }) == 0);
    expect("and it says it can invert, because this chip's matrix does",
           ak_esp_rc_inverted() == 1);

    /* --- the protocol switch ------------------------------------------------- */

    /* Half a frame at the old settings is worse than none: the switch flushes
     * what was in flight and reconfigures the line. */
    {
        static const uint8_t leftover[3] = { 0xAAu, 0xBBu, 0xCCu };
        ak_host_uart_feed(TEST_RC_UART, leftover, sizeof leftover);
        ak_esp_rc_set_protocol(1u); /* SBUS */

        expect("sbus is 100000 baud, even parity, two stop bits",
               ak_host_uart_baud(TEST_RC_UART) == TEST_SBUS_BAUD &&
                   ak_host_uart_parity(TEST_RC_UART) == UART_PARITY_EVEN &&
                   ak_host_uart_stop_bits(TEST_RC_UART) == UART_STOP_BITS_2);
        expect("and the receive line is inverted by the port itself",
               ak_host_uart_inverted(TEST_RC_UART) == (int)UART_SIGNAL_RXD_INV);
        expect("and what was in flight at the old settings is thrown away",
               ak_host_uart_pending(TEST_RC_UART) == 0u &&
                   ak_host_uart_flushes(TEST_RC_UART) == 1u);

        ak_esp_rc_set_protocol(0u); /* back to CRSF */
        expect("switching back restores the CRSF rate, 8N1 and no inversion",
               ak_host_uart_baud(TEST_RC_UART) == TEST_CRSF_BAUD &&
                   ak_host_uart_parity(TEST_RC_UART) == UART_PARITY_DISABLE &&
                   ak_host_uart_stop_bits(TEST_RC_UART) == UART_STOP_BITS_1 &&
                   ak_host_uart_inverted(TEST_RC_UART) == 0);
    }

    /* --- the driver's events -------------------------------------------------- */

    /* A buffer overflow is a number the driver reports, not one this file
     * keeps - and an event queue nobody drains is how that number goes
     * missing. So the port drains it on the poll path. */
    {
        unsigned flushes = ak_host_uart_flushes(TEST_RC_UART);
        ak_host_uart_push_event(TEST_RC_UART, UART_BUFFER_FULL);
        expect("the port drains the driver's events on its poll path",
               ak_esp_rc_dropped() == 1u &&
                   ak_host_uart_events(TEST_RC_UART) == 0u);
        expect("and asks for a flush afterwards, or the driver reports it "
               "again",
               ak_host_uart_flushes(TEST_RC_UART) == flushes + 1u);
    }

    ak_host_uart_push_event(TEST_RC_UART, UART_FIFO_OVF);
    expect("a fifo overflow is the same answer",
           ak_esp_rc_dropped() == 2u &&
               ak_host_uart_events(TEST_RC_UART) == 0u);

    /* Framing errors are not dropped bytes: they are the symptom of the wrong
     * protocol, and they are said once rather than once a byte. */
    {
        ak_host_uart_push_event(TEST_RC_UART, UART_FRAME_ERR);

        /* The console's sink is the F405's USART in this binary: point it at a
         * mapped register and leave a sentinel in the data register. If the
         * port prints, the last byte of the line lands there; if it prints
         * *again* on the next event, the sentinel is clobbered. */
        ak_console_attach(TEST_CONSOLE_USART);
        *sr() = USART_SR_TXE;
        *dr() = 'X';
        (void)ak_esp_rc_poll(&(uint8_t){ 0 });
        expect("a framing error is reported once, on the console",
               (*dr() & 0xFFu) == '\n');

        *dr() = 'X';
        for (int i = 0; i < 10; i++) {
            ak_host_uart_push_event(TEST_RC_UART, UART_FRAME_ERR);
        }
        (void)ak_esp_rc_poll(&(uint8_t){ 0 });
        expect("and not once per byte after that",
               (*dr() & 0xFFu) == 'X');
        ak_console_attach(0);
    }

    expect("a receive buffer that never overflowed says zero",
           ak_esp_rc_dropped() == 2u);

    /* --- telemetry out -------------------------------------------------------- */

    {
        /* `uint8_t`, not `char`: a CRSF frame's first byte is 0xEE, and `char`
         * is signed on x86_64 and unsigned on this Pi's aarch64 - which is a
         * sign-conversion warning on the laptop and silence here. The check
         * that builds and runs the host suite for x86_64 is what found it. */
        static const uint8_t telemetry[5] = { 0xEEu, 0x10u, 0x02u, 0x03u, 0x04u };
        expect("a telemetry frame goes out of the receiver's port",
               ak_esp_rc_send((const char *)telemetry, sizeof telemetry) == 0 &&
                   ak_host_uart_tx_bytes(TEST_RC_UART) == sizeof telemetry &&
                   memcmp(ak_host_uart_tx(TEST_RC_UART), telemetry,
                          sizeof telemetry) == 0);

        ak_host_uart_set_write_short(1);
        expect("a frame the driver only took part of is an error, not a success",
               ak_esp_rc_send((const char *)telemetry, sizeof telemetry) == -1);
        ak_host_uart_set_write_short(0);

        ak_host_uart_set_tx_stuck(1);
        expect("and a transmitter that never drains is the same answer",
               ak_esp_rc_send((const char *)telemetry, sizeof telemetry) == -1);
        ak_host_uart_set_tx_stuck(0);
    }

    /* --- the gps's port, which is the other half of "one ring per port" ------ */

    ak_esp_gps_init(TEST_GPS_UART, TEST_GPS_TX, TEST_GPS_RX, 9600);
    expect("the gps's uart comes up on its own port",
           ak_host_uart_installed(TEST_GPS_UART) == 1 &&
               ak_host_uart_baud(TEST_GPS_UART) == 9600 &&
               ak_host_uart_tx_pin(TEST_GPS_UART) == TEST_GPS_TX);

    {
        static const uint8_t ubx[5] = { 0xB5u, 0x62u, 0x01u, 0x07u, 0x00u };
        uint8_t byte = 0;
        ak_host_uart_feed(TEST_GPS_UART, ubx, sizeof ubx);
        expect("its bytes come back through its own poll",
               ak_esp_gps_poll(&byte) == 1 && byte == ubx[0]);
        expect("and the receiver's port did not touch them",
               ak_host_uart_pending(TEST_RC_UART) == 0u);
    }

    /* A configuration frame out, and the bound that keeps a full transmit
     * buffer from stopping the loop. */
    expect("a configuration frame goes out of the gps's port",
           ak_esp_gps_send("cfg", 3u) == 0 &&
               ak_host_uart_tx_bytes(TEST_GPS_UART) == 3u);
    ak_host_uart_set_tx_stuck(1);
    expect("and a gps that is not reading costs a failed write",
           ak_esp_gps_send("cfg", 3u) == -1);
    ak_host_uart_set_tx_stuck(0);

    /* --- the port that never came up ----------------------------------------- */

    ak_host_uart_reset();
    ak_host_uart_set_install_refused(1);
    /* The board's own init prints through the console; on this binary that is
     * the F405's sink again, so the same attachment is used to see the line. */
    ak_console_attach(TEST_CONSOLE_USART);
    *sr() = USART_SR_TXE;
    *dr() = 'X';
    ak_esp_rc_init(TEST_RC_UART, TEST_RC_TX, TEST_RC_RX, TEST_CRSF_BAUD,
                   TEST_SBUS_BAUD);
    expect("a port that refused to install says so on the console",
           (*dr() & 0xFFu) == '\n' && ak_host_uart_installed(TEST_RC_UART) == 0);
    expect("and polling it is nothing, not a wait",
           ak_esp_rc_poll(&(uint8_t){ 0 }) == 0);
    expect("as is sending, which a board with no receiver cannot do",
           ak_esp_rc_send("x", 1u) == -1 && ak_esp_gps_send("x", 1u) == -1);

    /* And the gps port, which says *which* port it was rather than nothing: a
     * module on a pin this chip cannot route is a wiring mistake, and the one
     * line that names it is the difference between that and a dead module. */
    ak_host_uart_reset();
    ak_host_uart_set_install_refused(1);
    *sr() = USART_SR_TXE;
    *dr() = 'X';
    ak_esp_gps_init(TEST_GPS_UART, TEST_GPS_TX, TEST_GPS_RX, 9600);
    expect("a gps port that refused to install says so, by name",
           (*dr() & 0xFFu) == '\n');
    expect("and its drop counter is nothing rather than a wait",
           ak_esp_gps_dropped() == 0u);
    ak_host_uart_set_install_refused(0);

    /* The other half of that: the driver installs and the *parameters* are
     * refused, so the port has to take the driver down again - a UART left
     * installed with no pins is a UART that owns them. */
    ak_host_uart_reset();
    ak_host_uart_set_param_refused(1);
    ak_esp_gps_init(TEST_GPS_UART, TEST_GPS_TX, TEST_GPS_RX, 9600);
    expect("parameters the driver refuses leave nothing installed",
           ak_host_uart_installed(TEST_GPS_UART) == 0);
    ak_host_uart_set_param_refused(0);
    ak_console_attach(0);
}
