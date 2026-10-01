#ifndef AK_CORE_AK_BOARD_H
#define AK_CORE_AK_BOARD_H

#include <stdint.h>

#include "ak_console.h"
#include "ak_boot.h"
#include "ak_bus.h"
#include "ak_output.h"
#include "ak_flashlog.h"
#include "ak_params.h"

/*
 * What the portable core is allowed to ask of a board.
 *
 * Everything in src/core/ goes through this, plus console.h and time.h. No
 * portable file may include an MCU header; that is the rule that keeps the
 * ESP32 port a port instead of a rewrite.
 */

void ak_board_init(void);

const char *ak_board_name(void);

/*
 * Which aircraft this board is built into, as the `airframe` parameter's number:
 * 0 is a quad-X, 7 is the single-motor elevon wing, and ak_mixer.h's table is
 * the whole list.
 *
 * A board answer rather than a core one because it is a fact about the *header*.
 * An airframe's mix needs a number of motors and a number of servos, and the
 * arming gate refuses when the board cannot drive them all - so a board with two
 * motor pads and two servo pads cannot fly the quad-X that is every other
 * board's default, and the refusal nobody can clear is the one that has to be
 * answered where the pads are known.
 *
 * It is the parameter's *default*, not an override: a saved configuration still
 * wins, and `defaults` restores this.
 */
uint32_t ak_board_default_airframe(void);

/* Status pin. No assumption about which way is "on" leaks out of the board. */
void ak_board_led_set(int on);
void ak_board_led_toggle(void);
int  ak_board_led_state(void);

/* Human-readable one-line summary of the clocks the board ended up at. */
const char *ak_board_clock_summary(void);

/* Saved configuration. Returns the number of bytes read into buf, 0 when the
 * board holds nothing saved, and -1 on an error. The write returns 0 on
 * success. A board with no storage at all returns 0 and -1, so the console can
 * say so rather than pretend. */
int ak_board_config_read(void *buf, uint32_t len);
int ak_board_config_write(const void *buf, uint32_t len);

/* Console receive, polled: 1 and the byte when one has arrived, 0 otherwise. */
int ak_board_console_poll_rx(char *byte);

/*
 * How long the core drives the console before the banner, in the diagnostic
 * build only (`make EXTRA_CFLAGS=-DAK_BOOT_USB_PUMP_MS=5000`).
 *
 * A board whose console door is *polled* - the F405's USB is, and its UART is
 * the other door on the same console - has no console at all until something
 * calls the poll above, and the only caller is the main loop. So from the
 * moment the port is brought up until the loop starts, a host sees a device
 * that pulled up D+ and then answered nothing: no descriptors, no port, and
 * every byte the firmware printed sitting in a ring nobody drains. That is not
 * a symptom of anything; it is what every boot looks like from the host side.
 * It is also why a fault before the main loop is invisible on the bench - and
 * it is the reason the F405's first power-on could be read neither from the
 * console nor from the LED.
 *
 * N milliseconds of poll here, after the millisecond tick exists and before
 * the banner is printed, is enough for a host to enumerate and open the port.
 * Bytes printed after that leave on their own: the write path pushes each
 * packet to the endpoint FIFO, so once the device is configured the transmit
 * side needs no further polling. The default is 0, which compiles to nothing.
 */
#ifndef AK_BOOT_USB_PUMP_MS
#define AK_BOOT_USB_PUMP_MS 0
#endif

/*
 * The USB trace, in the diagnostic build only
 * (`make EXTRA_CFLAGS=-DAK_USB_TRACE=1`).
 *
 * The F405 on this bench has a problem that has now outlived four attempted
 * fixes: it puts a full-speed device on the bus, the host resets it, and the
 * device-descriptor read comes back `error -71` on every boot. Register by
 * register the driver agrees with ST's reference - the speed field, the
 * endpoint-0 packet size, the STUPCNT reload, the endpoint-0 FIFO register and
 * its depth, the turnaround time, the global IN NAK release - and the host
 * tests model every one of those writes and stay green. So the disagreement is
 * not about what is written. It is about what the board does afterwards, and
 * the only instrument that can see that is the board.
 *
 * The console cannot be it: its two doors are the USB port that will not
 * enumerate and a UART that needs an adapter this bench does not have. The LED
 * is what is left, and this is the smallest set of numbers that separates the
 * hypotheses:
 *
 *   the system clock   because a USB device needs a 48 MHz PHY clock that the
 *                      D+ pull-up does *not* need, so a device whose crystal
 *                      the clock layer could not use still attaches perfectly
 *                      and then fails every transaction. `docs/02-hardware.md`
 *                      bounds the crystal at 4-26 MHz from the ROM bootloader
 *                      enumerating, which is not the same as 8 MHz.
 *   the setup count    did the core ever hand the stack a setup packet, or is
 *                      the receive path the end that is dead
 *   the send count     did the stack ever answer one, or is it the transmit end
 *
 * A setup count of zero with a good clock is a broken receive path; a setup
 * count above zero with a good clock and a send count to match is a stack that
 * ran correctly and a bus that did not, which points back at the clock or the
 * phy. Those are different repairs, and this is what tells them apart.
 *
 * The report is blinked on the status pin by the core at ten seconds, and the
 * arch fills the struct - the clock is the arch's to know, since no core file
 * may include an arch header. A port that has not implemented this fails to
 * link, which is the honest answer for a build flag only the F405 target sets.
 *
 * Four numbers and no more, one per group blinked: the clock as a frequency
 * rather than as a yes/no, because `sysclk_hz` already says whether the crystal
 * was usable (16 MHz *is* the fallback) and a second field saying so again would
 * be a number nobody reads. `enum_done` was collected for a while and dropped
 * for the same reason - it fires on the same bus reset that `resets` counts, so
 * it was never the number that decided anything.
 */
#ifndef AK_USB_TRACE
#define AK_USB_TRACE 0
#endif

#if AK_USB_TRACE
typedef struct {
    uint32_t sysclk_hz;  /* what the clock layer settled at: 168 MHz, or 16 */
    uint16_t resets;     /* bus resets the core saw */
    uint16_t setups;     /* setup packets handed to the stack */
    uint16_t sends;      /* control transfers the stack started */

    /* The second reading, for the question the first one left open: a core
     * that sees resets and is handed no SETUP packets has stopped somewhere
     * between the wire and the stack, and these say where. See the note on the
     * counters in src/arch/stm32f405/usb.c for what each one answers. */
    uint32_t passes;         /* how many times the poll ran */
    uint32_t rx_entries;     /* entries taken off the receive status queue */
    uint32_t pktsts_setup;   /* ... of which the core called a SETUP */
    uint32_t pktsts_data;    /* ... an OUT data packet */
    uint32_t pktsts_other;   /* ... the end of a transfer, or a global NAK */
    uint32_t gintsts_or;     /* every global-status bit the poll ever saw */

    /* And the registers, read at report time rather than accumulated: what the
     * core thinks its own state is, in the core's own encoding. A host cannot
     * guess any of these, and all three are read back exactly as they are so
     * the decoding stays on the side of the wire that has the reference. */
    uint32_t pllcfgr;        /* RCC_PLLCFGR: where the 48 MHz for USB comes from */
    uint32_t dsts;           /* OTG_DSTS: the speed the core settled at */
    uint32_t dctl;           /* OTG_DCTL: soft disconnect, and the global NAKs */
} ak_usb_trace_t;

/* The counters as they stand, plus what the clock layer ended up at. */
void ak_usb_trace_get(ak_usb_trace_t *out);

/*
 * And the same four numbers written down where a host can read them back with
 * no eyes involved - the ROM bootloader's DFU upload, which reads memory over
 * the wire the image was flashed on.
 *
 * Returns 0 when the record is in flash, -1 when the board has nowhere to put
 * it or the place it has is not blank. Which page that is, and why it is that
 * one, belongs to the board: it is a fact about this part's flash map and no
 * other board's.
 *
 * The *format* is the board's too, and that is the awkward half: the reader is
 * a program on a host, so the layout has to be written down somewhere a host
 * can find it rather than only in the code that writes it. It is in
 * docs/05-bringup.md, section 6c, beside the command that reads it.
 */
int ak_board_trace_save(void);
#endif

/*
 * The board's network, if it has one. Bytes in and bytes out, exactly like the
 * console - the core does not know whether they arrived on a UART, on Wi-Fi or
 * on the emulated Ethernet a test rig uses, and the protocol served over it is
 * the same code either way.
 *
 * A board with no network says so with ready() == 0 and the core never opens a
 * protocol on it; a board with a network and nobody connected says so with
 * connected() == 0. Those are different answers, and the console reports both.
 */
int  ak_board_net_ready(void);
int  ak_board_net_connected(void);
/* 1 and the byte, or 0 when nothing is waiting. Never blocks. */
int  ak_board_net_poll_rx(char *byte);
void ak_board_net_write(const char *data, unsigned len);
void ak_board_net_report(ak_printf_fn out);

/*
 * Parameters the board itself has to offer - a network name, a password, an
 * antenna choice - in the same table as everything else, so there is one
 * configuration path rather than a second one beside it. Called while the
 * table is being built, before anything is loaded, and returns the first free
 * index like the other registration helpers.
 */
unsigned ak_board_param_table(ak_param_t *items, unsigned count);

/*
 * Bring the board's network up. Deliberately *not* part of ak_board_init: on
 * the ESP32 the credentials are parameters, so the network cannot start until
 * the saved configuration has been loaded into them - and starting it early
 * would be a board that comes up on the wrong network or none.
 */
void ak_board_net_start(void);

/*
 * A block of RAM that startup does not clear, for records that should outlive
 * a reset - the fault record is one, the long log is another. Returns the
 * address and sets `bytes`, or returns 0 on a board that has none (in which
 * case the core uses ordinary memory and the log simply does not survive).
 *
 * What it does *not* survive is losing power. A log across a power cycle wants
 * flash, and saying so here is better than letting a bench session assume a
 * battery-backed something that is not there.
 */
void *ak_board_retained_ram(unsigned *bytes);

/*
 * Flash for the blackbox, or 0 for a board whose log is RAM-sized. The core
 * writes records into it through ak_flashlog.c and never erases anything on
 * its own: an erase stops the CPU for about a second on the STM32, which the
 * aircraft cannot afford and the bench cannot even notice. See
 * src/core/flight/ak_flashlog.h.
 */
const ak_flashlog_store_t *ak_board_log_store(void);

void ak_board_reboot(void);

/*
 * Hand the part to its ROM bootloader, for a board that can get there from
 * software - 0 on a board that cannot, and on one that can the call does not
 * return, because the ROM takes the part over.
 *
 * It exists because the alternative on some boards is a jumper: the wing's
 * flight controller enters DFU with a button *and* a solder joint, and the
 * firmware that was replaced could reach the ROM by command. The F405 has a
 * BOOT0 button, which is how its image was written, and the software path is
 * the one that does not need somebody at the board.
 *
 * The ESP32 answers 0, and that is not a gap: its bootloader is entered by the
 * hardware and by `idf.py`, not by the running firmware.
 */
int ak_board_enter_bootloader(void);

/* What the board knows about itself, for the preflight check. The core cannot
 * ask the arch layer directly - that is the port boundary - so anything the
 * core wants to verify arrives through here. */
uint32_t ak_board_console_port(void);
/* Where console output is actually going, which is not necessarily the same
 * thing - and comparing the two is how a silent board gets caught at boot
 * rather than at the bench. */
uint32_t ak_board_console_attached_port(void);
int      ak_board_clock_ok(void);
uint32_t ak_board_clock_sysclk_hz(void);
uint32_t ak_board_clock_apb1_hz(void);
uint32_t ak_board_output_dshot_hz(void);
uint32_t ak_board_output_dshot_period(void);
uint32_t ak_board_output_ccr_zero(void);
uint32_t ak_board_output_ccr_one(void);
uint32_t ak_board_output_frames_sent(void);

/* Motors and servos. The board owns the pins, the timers and the DMA; the
 * flight core only says what the outputs should be doing. */
void ak_board_output_init(void);
/* Whether this board drives motors and servos at all. A board that does not is
 * not a broken one - it is a port in progress - and the preflight check asks
 * rather than assuming. */
int  ak_board_output_ready(void);
/*
 * How many motors and how many servos this board actually drives.
 *
 * The flight core compares it with what the selected airframe's mixer needs and
 * refuses to arm when the board cannot drive it - a wing's two motors and two
 * elevons on a board with two outputs, or a quadrotor's four motors on one with
 * two, is a crash with a plausible number in it. Every board answers; a board
 * that does not is a board whose aircraft will not arm, which is the safe
 * direction for a number nobody has stated.
 */
void ak_board_output_shape(unsigned *motors, unsigned *servos);
void ak_board_output_write(const ak_output_frame_t *frame);
void ak_board_output_set_rate(uint32_t khz);
void ak_board_output_report(ak_printf_fn out);

/* Receiver input. The board owns the UART; the bytes come out one at a time. */
void ak_board_rc_init(void);
/* Which protocol those bytes are. The core owns the choice - it is a
 * parameter - but the board has to hear about it, because CRSF and SBUS do not
 * even run at the same line settings: 420000 baud 8N1 against 100000 baud
 * 8E2. The board reconfigures its port and says nothing. */
void ak_board_rc_set_protocol(uint32_t protocol);
int  ak_board_rc_poll(uint8_t *byte);
uint32_t ak_board_rc_dropped(void);
/* Whether the receive path inverts the signal, which is what SBUS is: it idles
 * low and starts with a high bit, the opposite of a UART. An F4 cannot invert
 * its own pins - the part has no such bit - so a board without an inverter in
 * front of the receiver pin sees a stream of framing errors and would report
 * them as a wrong baud rate. It is asked rather than assumed, so the console
 * can say which of those two it is. */
int  ak_board_rc_inverted(void);
/* The receiver's transmit line, for telemetry: CRSF frames the flight
 * controller sends back up the same wire. 0 on success, -1 when this board has
 * no way to speak - which is the honest answer for a port wired for SBUS,
 * since SBUS is a one-way protocol. */
int  ak_board_rc_send(const char *data, unsigned len);

/* The GPS's UART. Same shape as the receiver's, on its own port. */
void ak_board_gps_init(void);
int  ak_board_gps_poll(uint8_t *byte);
uint32_t ak_board_gps_dropped(void);
int  ak_board_gps_send(const char *data, unsigned len);

/*
 * The flight pack, measured through whatever divider the board is drawn with.
 *
 * The board reports volts *at its own ADC pin* and nothing else. The ratio,
 * the cell count and the thresholds stay in the flight core, because they are
 * the pilot's to set and they are the same arithmetic on every board - and
 * because a second board with a different divider is then a pin and a pair of
 * resistors rather than a second copy of a state machine.
 *
 * A board with no way to measure a battery says so with ready() == 0, and a
 * reading that is not available arrives as a negative voltage rather than as
 * zero: zero is also what a divider with no pack on it reads, and the console
 * should be able to tell "nothing plugged in" from "nothing to read it with".
 */
void  ak_board_battery_init(void);
int   ak_board_battery_ready(void);
float ak_board_battery_pin_volts(void);
/* Raw counts, the reference and the full scale - the hardware half of the
 * answer, so somebody with a multimeter can disagree with the firmware. */
void  ak_board_battery_report(ak_printf_fn out);

/* The IMU's bus, a loopback check for the wires under it, and its bring-up.
 * The bus is a plain read/write/delay interface, so a driver never learns
 * which SPI port it is on - or that it is SPI at all. */
void ak_board_imu_init(void);
const ak_bus_t *ak_board_imu_bus(void);
/* The barometer's bus, or null. A separate bus even when the part shares the
 * SPI wires with the IMU, because it has its own chip select - and a board
 * with no barometer fitted returns null, which the console says out loud. */
const ak_bus_t *ak_board_baro_bus(void);

/*
 * The rangefinder's bus, or null.
 *
 * A second bus on the same I2C port as the barometer and nothing else: same
 * two wires, different address, which is what an I2C bus is *for*. It is a
 * separate answer rather than a parameter of the barometer's bus because the
 * two parts are fitted independently - a board with a rangefinder and no
 * barometer is a legitimate machine, and so is the other way round - and
 * because a driver that had to be told which of two devices on one bus it was
 * talking to would be a driver with an address in it, which is exactly what
 * ak_bus.h exists to keep out.
 */
const ak_bus_t *ak_board_range_bus(void);
void ak_board_spi_loopback(ak_printf_fn out);

#endif /* AK_CORE_AK_BOARD_H */
