#include "arch.h"

#include "ak_board.h"

/*
 * USB, as a virtual serial port - the console the goal's M0 asks for.
 *
 * The reason this exists at all: AerialKit has no USB stack, so the moment it
 * boots, the board vanishes from the host it is plugged into and its console
 * exists only on PA2/PA3. On a bench with no USB-TTL adapter that means an LED
 * and no words, which is where the first flash ended. With this, the board
 * comes up as /dev/ttyACM0 on the machine it is already plugged into.
 *
 * Polled, like the console's UART and for the same reason: one person types at
 * it, and an interrupt for that is an interrupt to get wrong. The cost is that
 * a host can outrun us between calls - the receive FIFO absorbs that - and the
 * benefit is that everything happens in a known place, in a known order.
 *
 * The controller is OTG_FS (RM0090 33). Only the device side is implemented:
 * this board is never a host. Endpoints in use: 0 control, 0x82 notification
 * (which the CDC class requires and nothing here sends on), 0x81 bulk IN for
 * the console's output, 0x02 bulk OUT for what a person types. Both bulk
 * directions are live: the banner leaves by 0x81, typing arrives on 0x02, and
 * ak_usb_read() is where the console picks it up.
 *
 * Nothing here has run on the board yet. It compiles, and tests/test_arch.c
 * executes it against a mapped register block - enumeration, the descriptors,
 * both bulk directions and the requests a serial driver makes - so what it does
 * is pinned as behaviour rather than as constants. Whether the silicon agrees
 * is the next bench question, and the host's kernel log is the instrument that
 * answers it.
 */

/*
 * The receive FIFO, behind a seam for the host build.
 *
 * This is the one register in the file whose *reads* have an order: a setup
 * packet is two words, a data packet as many as its length needs, and reading
 * them is what makes the core drop its queue entry. A page of memory cannot
 * model that - every read of it returns the same word - and the first version
 * of the host test fed the driver setup packets with the same four bytes in
 * both words, which is a packet no host ever sends. The test passed because the
 * driver and the test agreed with each other, and the bench disagreed:
 * `device descriptor read/64, error -32` is a device stalling every request it
 * was sent, which is what a request decoded from the wrong byte looks like.
 */
#ifdef AK_HOST_USB
#include "host_usb_model.h"
#define otg_fifo_read() host_usb_fifo_read()
#else
static uint32_t otg_fifo_read(void)
{
    return OTG_FIFO(0u);
}
#endif

/* FIFO budget, in 32-bit words. OTG_FS has 320 words of shared FIFO RAM:
 * 128 for everything the host sends plus each IN endpoint's transmit buffer. */
#define AK_USB_RX_WORDS   128u /* receive: shared by all OUT endpoints */
#define AK_USB_EP0_WORDS   16u /* endpoint 0's transmit buffer */
#define AK_USB_EP1_WORDS   64u /* the console's, so a line can go out whole */
#define AK_USB_EP2_WORDS   16u /* the notification endpoint CDC requires */

#define AK_USB_MPS_EP0 64u
#define AK_USB_MPS_BULK 64u
#define AK_USB_MPS_NOTIFY 8u

/* The vendor and product the ST virtual COM port uses, so a Linux host binds
 * cdc_acm to it without a rule or a driver of our own. */
#define AK_USB_VID 0x0483u
#define AK_USB_PID 0x5740u

/* --- the descriptors ------------------------------------------------------ */

typedef struct __attribute__((packed)) {
    uint8_t  length;
    uint8_t  type;
    uint16_t bcd_usb;
    uint8_t  device_class;
    uint8_t  device_subclass;
    uint8_t  device_protocol;
    uint8_t  max_packet0;
    uint16_t id_vendor;
    uint16_t id_product;
    uint16_t bcd_device;
    uint8_t  manufacturer;
    uint8_t  product;
    uint8_t  serial;
    uint8_t  configurations;
} ak_usb_device_descriptor_t;

static const ak_usb_device_descriptor_t device_descriptor = {
    .length = 18u,
    .type = 0x01u,
    .bcd_usb = 0x0200u,
    .device_class = 0x02u, /* communications */
    .device_subclass = 0x00u,
    .device_protocol = 0x00u,
    .max_packet0 = AK_USB_MPS_EP0,
    .id_vendor = AK_USB_VID,
    .id_product = AK_USB_PID,
    .bcd_device = 0x0100u,
    .manufacturer = 1u,
    .product = 2u,
    .serial = 3u,
    .configurations = 1u,
};

/*
 * One configuration, two interfaces: a CDC control interface with the three
 * functional descriptors the class requires, and a data interface with the
 * bulk pair. Sixty-seven bytes, and wTotalLength has to match or the host
 * walks off the end of it - which is why the length is asserted below rather
 * than trusted.
 */
static const uint8_t configuration_descriptor[] = {
    /* configuration */
    0x09, 0x02, 67u, 0x00, 2u, 1u, 0x00, 0x80u, 0x32u,
    /* interface 0: CDC control, ACM */
    0x09, 0x04, 0u, 0x00u, 1u, 0x02u, 0x02u, 0x01u, 0u,
    /* header functional */
    0x05, 0x24, 0x00, 0x10, 0x01,
    /* call management */
    0x05, 0x24, 0x01, 0x00, 0x01,
    /* abstract control management: line coding and control */
    0x04, 0x24, 0x02, 0x02,
    /* union: control interface 0 and data interface 1 */
    0x05, 0x24, 0x06, 0x00, 0x01,
    /* notification endpoint, 0x82 in */
    0x07, 0x05, 0x82u, 0x03u, AK_USB_MPS_NOTIFY, 0x00u, 0x0Au,
    /* interface 1: CDC data */
    0x09, 0x04, 1u, 0x00u, 2u, 0x0Au, 0x00u, 0x00u, 0u,
    /* bulk in, 0x81: the console's output */
    0x07, 0x05, 0x81u, 0x02u, AK_USB_MPS_BULK, 0x00u, 0x00u,
    /* bulk out, 0x02: what a person types */
    0x07, 0x05, 0x02u, 0x02u, AK_USB_MPS_BULK, 0x00u, 0x00u,
};

_Static_assert(sizeof configuration_descriptor == 67u,
               "the configuration descriptor and its declared length disagree");
_Static_assert(sizeof device_descriptor == 18u, "device descriptor is not 18 bytes");

/*
 * String descriptors, in the UTF-16LE the class asks for. Index 0 is the
 * language list; 1 to 3 name the product.
 *
 * Each bLength is written as the same macro the array is asserted against,
 * because bLength is the length of the *whole descriptor* and not of the text
 * inside it - a distinction nothing in the language enforces, since the byte is
 * just a number. Three of these four agreed with their arrays and the serial
 * did not: it declared 12 over 14 bytes.
 *
 * That is a host walking two bytes off the end of a descriptor it was handed.
 * It is also invisible here, because a host asks for string index 3 only after
 * it has read the device and configuration descriptors, and this board has
 * never got that far - so the one descriptor that was wrong is the one
 * descriptor no test or bench reading could have reached. Asserted at compile
 * time for that reason: the failure this pins down is silent at runtime until
 * the day everything else works.
 *
 * The first spelling of these asserts compared sizeof against `array[0]`, which
 * is not a constant expression for a const object in C - it does not compile.
 * A check that has to be removed to build is not a check, so the macro carries
 * the number and both the byte and the assertion read it from there.
 */
#define AK_USB_STR_LANGUAGE_LEN      4u
#define AK_USB_STR_MANUFACTURER_LEN 20u
#define AK_USB_STR_PRODUCT_LEN      30u
#define AK_USB_STR_SERIAL_LEN       14u

static const uint8_t string_language[] = {
    AK_USB_STR_LANGUAGE_LEN, 0x03u, 0x09u, 0x04u,
};
static const uint8_t string_manufacturer[] = {
    AK_USB_STR_MANUFACTURER_LEN, 0x03u,
    'A', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0, 'K', 0, 'i', 0, 't', 0,
};
static const uint8_t string_product[] = {
    AK_USB_STR_PRODUCT_LEN, 0x03u,
    'A', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0, 'K', 0, 'i', 0,
    't', 0, ' ', 0, 'F', 0, '4', 0, '0', 0, '5', 0,
};
static const uint8_t string_serial[] = {
    AK_USB_STR_SERIAL_LEN, 0x03u,
    'A', 0, 'K', 0, '0', 0, '0', 0, '0', 0, '1', 0,
};

_Static_assert(sizeof string_language == AK_USB_STR_LANGUAGE_LEN,
               "string_language bLength disagrees with its array");
_Static_assert(sizeof string_manufacturer == AK_USB_STR_MANUFACTURER_LEN,
               "string_manufacturer bLength disagrees with its array");
_Static_assert(sizeof string_product == AK_USB_STR_PRODUCT_LEN,
               "string_product bLength disagrees with its array");
_Static_assert(sizeof string_serial == AK_USB_STR_SERIAL_LEN,
               "string_serial bLength disagrees with its array");

/* --- the transmit path ---------------------------------------------------- */

/*
 * A ring for the console's bytes, because a console line is longer than the
 * FIFO and a write must never block the flight loop. Dropped bytes are counted
 * and the count is visible: a console that loses output silently is worse than
 * one that says it did.
 */
/* Two kilobytes, and the number is chosen for one fact: the firmware prints
 * its banner, its selftest and its preflight verdict in the first few
 * milliseconds, while the host is still enumerating - so the first second of
 * output has nowhere to go and waits in this ring. At 512 bytes the beginning
 * of it (the banner, the part worth reading) would be the part dropped. The
 * cost is two kilobytes of a 128 KB part. */
#define AK_USB_TX_CAPACITY 2048u

static char     tx_buffer[AK_USB_TX_CAPACITY];
static unsigned tx_head;
static unsigned tx_tail;
static uint32_t tx_dropped;
static int      usb_configured;
static int      usb_ready;
/*
 * Set from init until the first poll: the device is held off the bus for as
 * long as nothing is driving it. See the note at the end of `ak_usb_init()`.
 */
static int      usb_detached;
static uint8_t  control_in[AK_USB_MPS_EP0];
static unsigned control_in_length;
/*
 * Where the rest of a control IN that did not fit one packet lives.
 *
 * The packet buffer above is one packet long, so the bytes after the first
 * packet are not in it - they are still in the table the request was answered
 * from, and every one of those is static. Without this, the second packet of a
 * long descriptor was taken from the packet buffer at an offset past its end
 * with a length of zero: the host got 64 bytes of a 67-byte configuration
 * descriptor and nothing else, which is a device whose configuration the host
 * cannot parse. No test had ever asked for a descriptor longer than a packet.
 */
static const uint8_t *control_in_source;
static uint8_t  setup[8];

/*
 * The diagnostic's counters, and the one event each of them answers for.
 * Compiled out of a flight image: the macro is nothing, so the counters cost
 * neither a byte of RAM nor an instruction there. See the note in ak_board.h
 * for what the numbers are for and why the LED is where they come out.
 *
 * Three of them are the four numbers of the first reading: how many bus resets
 * the core saw, how many SETUP packets it handed to the stack, how many control
 * transfers the stack started. The rest answer the question that reading left
 * open, which is *where* between the wire and the stack a packet stops:
 *
 *   passes       how many times the poll ran. With the millisecond clock this
 *                is the loop's rate, and a rate too slow to catch a receive
 *                would look exactly like a wire that never delivered one.
 *   rx_entries   entries taken off the core's receive status queue, of any
 *                kind - so this counts *every* packet the core decoded,
 *                including ones this stack has no use for.
 *   pktsts_*     the same entries split by the packet-status field the core
 *                labels them with. A SETUP is 6 and an OUT data packet is 2.
 *                This is the pair that separates "the core never decoded a
 *                packet" (all three zero) from "the core decoded packets and
 *                this file dropped them" (entries counted, setups not).
 *   gintsts_or   the bitwise OR of every global-status snapshot the poll took.
 *                A one in it says that event happened at least once, which is
 *                the cheapest way to find out which of the core's events have
 *                ever fired at all.
 */
#if AK_USB_TRACE
static struct {
    uint16_t resets;
    uint16_t setups;
    uint16_t sends;
    uint32_t passes;
    uint32_t rx_entries;
    uint32_t pktsts_setup;
    uint32_t pktsts_data;
    uint32_t pktsts_other;
    uint32_t gintsts_or;
} usb_trace;

#define AK_USB_TRACE_NOTE(field) (usb_trace.field++)
/* For the one counter that is a set of events rather than a tally of them. */
#define AK_USB_TRACE_OR(field, value) (usb_trace.field |= (uint32_t)(value))
#else
#define AK_USB_TRACE_NOTE(field) ((void)0)
#define AK_USB_TRACE_OR(field, value) ((void)0)
#endif

static unsigned tx_used(void)
{
    return (tx_head - tx_tail + AK_USB_TX_CAPACITY) % AK_USB_TX_CAPACITY;
}

/* Push what fits, one 32-bit word at a time: the FIFO takes words, not bytes,
 * and the last word of an odd-length packet is padded with zeros. */
static void ep1_push(void)
{
    if (!usb_configured) {
        return;
    }
    unsigned used = tx_used();
    if (used == 0u || (OTG_DIEPCTL(1u) & OTG_DEPCTL_EPENA) != 0u) {
        return;
    }

    unsigned count = used > AK_USB_MPS_BULK ? AK_USB_MPS_BULK : used;
    /* Size and enable first, then the bytes: see control_send(). */
    OTG_DIEPTSIZ(1u) = count | (1u << 19); /* one packet, this many bytes */
    OTG_DIEPCTL(1u) |= OTG_DEPCTL_CNAK | OTG_DEPCTL_EPENA;
    for (unsigned at = 0; at < count; at += 4u) {
        uint32_t word = 0u;
        for (unsigned byte = 0; byte < 4u && at + byte < count; byte++) {
            word |= (uint32_t)(uint8_t)tx_buffer[(tx_tail + at + byte) %
                                                 AK_USB_TX_CAPACITY]
                    << (8u * byte);
        }
        OTG_FIFO(1u) = word;
    }
    tx_tail = (tx_tail + count) % AK_USB_TX_CAPACITY;
}

unsigned ak_usb_write(const char *data, unsigned len)
{
    unsigned taken = 0;

    for (unsigned i = 0; i < len; i++) {
        unsigned next = (tx_head + 1u) % AK_USB_TX_CAPACITY;
        if (next == tx_tail) {
            tx_dropped += len - i;
            break;
        }
        tx_buffer[tx_head] = data[i];
        tx_head = next;
        taken++;
    }
    ep1_push();
    return taken;
}

int ak_usb_ready(void)
{
    return usb_ready;
}

uint32_t ak_usb_dropped(void)
{
    return tx_dropped;
}

/* --- the receive path ----------------------------------------------------- */

/*
 * What a person types. A ring for the same reason the transmit side has one:
 * the poll runs once a pass, and a host sends a burst - a pasted line - of more
 * than the one 64-byte packet it can put on the bus at a time. Two hundred and
 * fifty-six bytes is a paste rather than a terminal buffer, and a byte that
 * will not fit is counted rather than silently lost.
 */
#define AK_USB_RX_CAPACITY 256u

static char     rx_buffer[AK_USB_RX_CAPACITY];
static unsigned rx_head;
static unsigned rx_tail;
static uint32_t rx_dropped;
static int      setup_pending; /* a setup packet is in `setup`, unread */

static void rx_push(char byte)
{
    unsigned next = (rx_head + 1u) % AK_USB_RX_CAPACITY;

    if (next == rx_tail) {
        rx_dropped++;
        return;
    }
    rx_buffer[rx_head] = byte;
    rx_head = next;
}

/* One byte the console typed at, or no byte at all. Polled, like every other
 * input on this board: the caller is the console's own reader. */
int ak_usb_read(char *byte)
{
    if (rx_head == rx_tail) {
        return 0;
    }
    *byte = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1u) % AK_USB_RX_CAPACITY;
    return 1;
}

uint32_t ak_usb_rx_dropped(void)
{
    return rx_dropped;
}

/* --- enumeration ---------------------------------------------------------- */

/* Hand the buffer back to the host on endpoint 0: arm the transfer, and let
 * the XFRC interrupt - which is a bit in DIEPINT0, which this poll reads - say
 * when it has been taken. */
static void control_send(const uint8_t *data, unsigned length)
{
    unsigned count = length > AK_USB_MPS_EP0 ? AK_USB_MPS_EP0 : length;

    AK_USB_TRACE_NOTE(sends);
    control_in_source = data;
    for (unsigned i = 0; i < count; i++) {
        control_in[i] = data[i];
    }
    control_in_length = length;

    /* Size, then enable, then the bytes - RM0090's order for an IN transfer
     * and the reference driver's (`USB_EPStartXfer`). Filling the FIFO of an
     * endpoint that is not yet enabled is not a sequence the core promises to
     * honour. */
    OTG_DIEPTSIZ0 = OTG_DIEPTSIZ0_XFRSIZ(count) | OTG_DIEPTSIZ0_PKTCNT1;
    OTG_DIEPCTL0 |= OTG_DEPCTL_CNAK | OTG_DEPCTL_EPENA;
    for (unsigned at = 0; at < count; at += 4u) {
        uint32_t word = 0u;
        for (unsigned byte = 0; byte < 4u && at + byte < count; byte++) {
            word |= (uint32_t)control_in[at + byte] << (8u * byte);
        }
        OTG_FIFO(0u) = word;
    }
}

static void control_stall(void)
{
    OTG_DIEPCTL0 |= OTG_DEPCTL_STALL;
    OTG_DOEPCTL0 |= OTG_DEPCTL_STALL;
}

/* The line coding a serial driver sets when it opens the port, and what it is
 * answered with: seven bytes - 9600 baud little-endian, one stop bit, no
 * parity, eight data bits. Nothing here has a baud rate to change (this is a
 * full-speed device whose console runs at the loop's pace), so the bytes are
 * kept only so that a request with a data stage has somewhere to put it. */
static uint8_t  line_coding[7] = { 0x80u, 0x25u, 0x00u, 0x00u,
                                   0x00u, 0x00u, 0x08u };
static unsigned control_out_pending; /* bytes of a data stage still to come */

/*
 * Arm endpoint 0's OUT side, and this is worth reading twice.
 *
 * STUPCNT is how many SETUP packets the core will hand over before the endpoint
 * has to be armed again. ST's own driver programs three here every time it
 * comes back from a request, and a device that never writes the field is a
 * device that answers the first descriptor request and then stops hearing
 * anything: the core has nothing left to give it. On a bench that is a board
 * that enumerates half way with no error reported anywhere. XFRSIZ covers the
 * setup packets the count allows and PKTCNT is one packet, as the reference
 * driver has it (USB_EP0_OutStart in stm32f4xx_ll_usb.c).
 */
static void ep0_out_arm(void)
{
    OTG_DOEPTSIZ0 = OTG_DOEPTSIZ0_XFRSIZ(3u * 8u) | OTG_DOEPTSIZ0_PKTCNT1 |
                    OTG_DOEPTSIZ0_STUPCNT(3u);
    OTG_DOEPCTL0 |= OTG_DEPCTL_CNAK | OTG_DEPCTL_EPENA;
}

/* The bulk OUT endpoint, armed one packet at a time. The core clears EPENA when
 * the packet lands and the poll arms it again; while it is not armed the
 * endpoint NAKs, so a re-arm that arrives late costs the host a retry rather
 * than a byte. */
static void ep2_arm(void)
{
    OTG_DOEPTSIZ(2u) = OTG_DOEPTSIZ_XFRSIZ(AK_USB_MPS_BULK) |
                       OTG_DOEPTSIZ_PKTCNT(1u);
    OTG_DOEPCTL(2u) |= OTG_DEPCTL_CNAK | OTG_DEPCTL_EPENA;
}

/*
 * One entry from the receive status queue.
 *
 * The status word and the bytes do not come from the same place, and getting
 * that wrong reads like a wiring fault: what says whose packet it is and how
 * long it is comes from GRXSTSP, and the bytes it describes are then popped one
 * thirty-two-bit word at a time out of FIFO 0. This file read the status out of
 * the FIFO first, which eats the first word of every payload as if it were a
 * status - on a bus where the host does not complain, it just never receives
 * what it asked for.
 *
 * Reading the bytes is also what makes the core drop the entry, so every entry
 * that has bytes behind it is read head to tail.
 */
static void receive_packet(void)
{
    uint32_t status = OTG_GRXSTSP;
    unsigned endpoint = status & OTG_GRXSTS_EPNUM;
    unsigned count = (status & OTG_GRXSTS_BCNT) >> 4;
    unsigned kind = (status & OTG_GRXSTS_PKTSTS) >> 17;
    unsigned words = (count + 3u) / 4u;

    AK_USB_TRACE_NOTE(rx_entries);
    if (kind == OTG_GRXSTS_SETUP) {
        AK_USB_TRACE_NOTE(pktsts_setup);
    } else if (kind == OTG_GRXSTS_DATA) {
        AK_USB_TRACE_NOTE(pktsts_data);
    } else {
        AK_USB_TRACE_NOTE(pktsts_other);
    }

    if (kind == OTG_GRXSTS_SETUP) {
        /* A setup packet is eight bytes whatever the count says, and the
         * answer to it is written in do_setup(), from the poll loop. */
        for (unsigned at = 0; at < 2u; at++) {
            uint32_t word = otg_fifo_read();
            setup[at * 4u + 0u] = (uint8_t)(word & 0xFFu);
            setup[at * 4u + 1u] = (uint8_t)((word >> 8) & 0xFFu);
            setup[at * 4u + 2u] = (uint8_t)((word >> 16) & 0xFFu);
            setup[at * 4u + 3u] = (uint8_t)((word >> 24) & 0xFFu);
        }
        AK_USB_TRACE_NOTE(setups);
        setup_pending = 1;
        return;
    }

    if (kind != OTG_GRXSTS_DATA) {
        return; /* the end of a transfer: nothing to read, nothing to do */
    }

    /* Two places a data packet can belong, and it is read out whichever it is:
     * this is the only place the FIFO gets emptied, and an entry that is not
     * read is an entry that stays in the queue. */
    unsigned to_line_coding = endpoint == 0u ? control_out_pending : 0u;

    for (unsigned at = 0; at < words; at++) {
        uint32_t word = otg_fifo_read();
        for (unsigned byte = 0; byte < 4u; byte++) {
            unsigned index = at * 4u + byte;

            if (index >= count) {
                break;
            }
            /* Endpoint 2 is the console's: what a person typed. Endpoint 0,
             * with a request's data stage outstanding, is the host setting the
             * line coding. Anything else reaching this device is nobody's. */
            if (endpoint == 2u && usb_configured) {
                rx_push((char)((word >> (8u * byte)) & 0xFFu));
            } else if (index < to_line_coding && index < sizeof line_coding) {
                line_coding[index] = (uint8_t)((word >> (8u * byte)) & 0xFFu);
            }
        }
    }

    if (to_line_coding > 0u) {
        /* The data has landed, so the transfer ends the way USB says it does:
         * the device answers the status stage with a zero-length packet. */
        control_out_pending = 0u;
        control_send(0, 0u);
        ep0_out_arm();
    }
}

/* A request, answered from the tables above. The three every host makes before
 * it will talk about anything else are GET_DESCRIPTOR, SET_ADDRESS and
 * SET_CONFIGURATION; the serial driver adds two of the CDC class ones when it
 * opens the port; everything else is stalled, which is how a device says "not
 * me" without hanging the bus. */
static void do_request(void)
{
    /*
     * The setup packet is USB 2.0 section 9.3's eight bytes in the order the
     * host puts them on the wire, and the FIFO hands them back in that order:
     * byte 0 is bmRequestType and byte 1 is bRequest. This read the request as
     * the second byte of a little-endian sixteen-bit pair, which is a device
     * that answers only packets no host sends: a real GET_DESCRIPTOR decodes as
     * 0x0680, matches nothing below, and gets stalled. The bench said exactly
     * that on 2026-09-17 - `device descriptor read/64, error -32`, then `Device
     * not responding to setup address` - and the host test agreed with the bug
     * because it fed the driver the same reversed packet. Artery's own library
     * for the same class of part parses the buffer this way round (INAV 9.1.0,
     * lib/main/AT32F43x/Middlewares/AT/AT32_USB_Device_Library/Core/Src/
     * usbd_sdr.c): bmRequestType is byte 0 and bRequest is byte 1.
     */
    unsigned request = setup[1];
    uint16_t value = (uint16_t)((unsigned)setup[3] * 256u + setup[2]);
    unsigned length = (unsigned)setup[7] * 256u + setup[6];
    unsigned type = (value >> 8) & 0xFFu;
    unsigned index = value & 0xFFu;

    /*
     * Class requests, which is what the serial driver sends once the device is
     * configured: bmRequestType 0x21 is host-to-device from an interface and
     * 0xA1 the other way, and what makes them class requests is bits 5 and 6
     * of that byte. The direction bit is redundant given the three request
     * numbers here, so the type field is what is checked. Linux survives a
     * stall on any of them (it ignores the answer to the first and only logs
     * the second), but a device that answers is a device that cannot be
     * blamed for it.
     */
    unsigned request_type = setup[0] & 0x60u;

    if (request_type == 0x20u && request == 0x0020u) { /* SET_LINE_CODING */
        control_out_pending = sizeof line_coding;
        return; /* the data stage follows; the status stage follows that */
    }
    if (request_type == 0x20u && request == 0x0022u) { /* SET_CONTROL_LINE_STATE */
        control_send(0, 0u);
        return;
    }
    if (request_type == 0x20u && request == 0x0021u) { /* GET_LINE_CODING */
        control_send(line_coding, sizeof line_coding);
        return;
    }

    if (request == 0x0005u) { /* SET_ADDRESS */
        OTG_DCFG = (OTG_DCFG & ~OTG_DCFG_DAD(0x7Fu)) | OTG_DCFG_DAD(index);
        control_send(0, 0u);
        return;
    }
    if (request == 0x0009u) { /* SET_CONFIGURATION */
        usb_configured = index != 0u;
        if (usb_configured) {
            /* Arm the bulk IN endpoint, the notification endpoint the class
             * requires, and the bulk OUT endpoint the console's input arrives
             * on. */
            OTG_DIEPCTL(1u) = OTG_DEPCTL_USBAEP | OTG_DEPCTL_EPTYP_BULK |
                              OTG_DEPCTL_TXFNUM(1u) |
                              OTG_DEPCTL_MPS(AK_USB_MPS_BULK);
            OTG_DIEPCTL(2u) = OTG_DEPCTL_USBAEP | OTG_DEPCTL_TXFNUM(2u) |
                              OTG_DEPCTL_MPS(AK_USB_MPS_NOTIFY) |
                              (0x3u << 18); /* interrupt */
            OTG_DOEPCTL(2u) = OTG_DEPCTL_USBAEP | OTG_DEPCTL_EPTYP_BULK |
                              OTG_DEPCTL_MPS(AK_USB_MPS_BULK);
            ep2_arm();
            OTG_DAINTMSK = (1u << 0) | (1u << 16) | (1u << 1) | (1u << 17);
            usb_ready = 1;
            /* Whatever was written while the host was still enumerating -
             * the banner, above all - has been waiting in the ring for
             * exactly this moment. */
            ep1_push();
        }
        control_send(0, 0u);
        return;
    }
    if (request == 0x0006u) { /* GET_DESCRIPTOR */
        const uint8_t *data = 0;
        unsigned size = 0u;

        if (type == 0x01u) {
            data = (const uint8_t *)&device_descriptor;
            size = sizeof device_descriptor;
        } else if (type == 0x02u) {
            data = configuration_descriptor;
            size = sizeof configuration_descriptor;
        } else if (type == 0x03u) {
            if (index == 0u) {
                data = string_language;
                size = sizeof string_language;
            } else if (index == 1u) {
                data = string_manufacturer;
                size = sizeof string_manufacturer;
            } else if (index == 2u) {
                data = string_product;
                size = sizeof string_product;
            } else if (index == 3u) {
                data = string_serial;
                size = sizeof string_serial;
            }
        }
        if (data == 0) {
            control_stall();
            return;
        }
        if (size > length) {
            size = length;
        }
        control_send(data, size);
        return;
    }
    control_stall();
}

/* One request, and then the OUT side is armed again - always, including when
 * what was asked for has a data stage.
 *
 * The core NAKs endpoint 0 OUT once a setup packet lands, so a data stage
 * (SET_LINE_CODING's seven bytes) is only received after the endpoint is
 * enabled and its NAK cleared - which is what ST's driver does in
 * USBD_CtlPrepareRx. Leaving it unarmed until the data had "landed" meant the
 * data could never land: Linux times the request out after five seconds and
 * opens the port anyway, but Windows usbser waits on it, so opening the port
 * hung and the request stayed stuck until the board was unplugged. */
static void do_setup(void)
{
    control_out_pending = 0u;
    do_request();
    ep0_out_arm();
}

/* --- the poll and the init ------------------------------------------------ */

/*
 * Put the device on the bus. Called by the first poll and by nothing else: see
 * the note at the end of `ak_usb_init()`. Doing it here rather than in the
 * board means every caller of the poll inherits the guarantee, including the
 * diagnostic boot pump, which is itself polling.
 */
static void usb_attach(void)
{
    OTG_DCTL &= ~OTG_DCTL_SDIS;
}

void ak_usb_poll(void)
{
    /* How often this runs is a fact about the whole firmware rather than about
     * USB, and it is the one thing a trace of the bus cannot tell you: a poll
     * too slow to catch an event that has already happened looks exactly like
     * an event that never happened. */
    AK_USB_TRACE_NOTE(passes);
    AK_USB_TRACE_OR(gintsts_or, OTG_GINTSTS);

    /* The first poll is the first moment this stack can answer the bus, so it
     * is the first moment the bus is allowed to see us. Then carry on: there is
     * nothing latched from a bus we were not on, and anything that is latched
     * is this pass's to handle anyway. */
    if (usb_detached) {
        usb_detached = 0;
        usb_attach();
    }

    if ((OTG_GINTSTS & OTG_GINT_USBRST) != 0u) {
        /* The host reset the bus: address zero, endpoints off, and endpoint 0
         * armed again. Everything else about a running configuration is the
         * host's to redo, and it will. */
        AK_USB_TRACE_NOTE(resets);
        OTG_GINTSTS = OTG_GINT_USBRST;
        OTG_DCFG = (OTG_DCFG & ~OTG_DCFG_DAD(0x7Fu)) | OTG_DCFG_DSPD_FS;
        OTG_DAINTMSK = 0u;
        OTG_DIEPMSK = OTG_DEPINT_XFRC;
        OTG_DOEPMSK = OTG_DEPINT_XFRC | OTG_DEPINT_STUP;
        OTG_DAINTMSK = (1u << 0) | (1u << 16);
        OTG_DIEPCTL0 = OTG_DEPCTL_USBAEP | OTG_DEPCTL_EPTYP_CTRL |
                       OTG_DEPCTL_MPS(AK_USB_MPS_EP0);
        OTG_DOEPCTL0 = OTG_DEPCTL_USBAEP | OTG_DEPCTL_EPTYP_CTRL |
                       OTG_DEPCTL_MPS(AK_USB_MPS_EP0);
        ep0_out_arm();
        usb_configured = 0;
        usb_ready = 0;
        control_in_length = 0u;
        control_out_pending = 0u;
        setup_pending = 0;
        /* Half a line from before the reset is not a command, and the host is
         * about to describe itself again anyway. */
        rx_head = 0u;
        rx_tail = 0u;
        return;
    }

    if ((OTG_GINTSTS & OTG_GINT_ENUMDNE) != 0u) {
        /*
         * Speed is known - full speed expected - but that is not all this
         * interrupt is for.
         *
         * The reset that preceded it leaves the core's global IN NAK set, and
         * that NAK covers every non-periodic IN endpoint. Endpoint 0 is one of
         * them, so until this bit is written the device descriptor the host
         * asks for is answered NAK, for as long as the host cares to keep
         * asking. It is not a stall - the host sees a device that is present
         * and unwilling, which is what a NAK means - so enumeration simply
         * never completes.
         *
         * The vendor's own stack treats this interrupt as the place for it:
         * `DCD_HandleEnumDone_ISR` (`usb_dcd_int.c`) calls `USB_OTG_EP0Activate`
         * as its first statement, and `USB_OTG_EP0Activate` (`usb_core.c`) sets
         * EP0's packet size from `DSTS.enumspd` and then writes
         * `DCTL.cgnpinnak = 1`. This event and no other.
         */
        OTG_DCTL |= OTG_DCTL_CGNPINNAK;
        OTG_GINTSTS = OTG_GINT_ENUMDNE;
    }

    /* One entry from the receive status queue a pass. The queue can hold more
     * than one - a descriptor request is a setup packet and then the status of
     * the transfer that follows - and the pass rate is thousands a second
     * against a bus that delivers hundreds, so there is nothing to gain by
     * spinning here until the flag clears. */
    if ((OTG_GINTSTS & OTG_GINT_RXFLVL) != 0u) {
        receive_packet();
    }

    /* Endpoint 0's IN side finished sending: a longer descriptor continues,
     * otherwise the status stage is the host's to finish. */
    if ((OTG_DIEPINT0 & OTG_DEPINT_XFRC) != 0u) {
        OTG_DIEPINT0 = OTG_DEPINT_XFRC;
        if (control_in_length > AK_USB_MPS_EP0) {
            unsigned sent = AK_USB_MPS_EP0;

            control_in_length -= sent;
            control_send(control_in_source + sent, control_in_length);
        }
    }

    /* Endpoint 0's OUT side finished a transfer - a data stage, or the status
     * stage of something that was sent - so it is armed for the next request. */
    if ((OTG_DOEPINT0 & OTG_DEPINT_XFRC) != 0u) {
        OTG_DOEPINT0 = OTG_DEPINT_XFRC;
        ep0_out_arm();
    }

    /* The bulk endpoint finished a packet: if there is more, send it. */
    if ((OTG_DIEPINT(1u) & OTG_DEPINT_XFRC) != 0u) {
        OTG_DIEPINT(1u) = OTG_DEPINT_XFRC;
        ep1_push();
    }

    /* The bulk OUT endpoint took a packet. Its bytes are already in the ring -
     * the receive queue was drained above - so it is armed for the next one. */
    if ((OTG_DOEPINT(2u) & OTG_DEPINT_XFRC) != 0u) {
        OTG_DOEPINT(2u) = OTG_DEPINT_XFRC;
        if (usb_configured) {
            ep2_arm();
        }
    }

    /*
     * The setup stage is over, and only now is the request answered.
     *
     * A setup packet arrives as two receive-queue entries: its eight bytes
     * (PKTSTS 6), then "setup stage done" (PKTSTS 4), and only when the second
     * has been popped does the core raise STUP. This driver used to answer from
     * the first - loading endpoint 0's IN side while the core still held the
     * setup stage open - which is a race with the core rather than a sequence,
     * and its outcome moved with the poll's own timing: the trace build (a few
     * counters more per pass) took nineteen setups, the flight build none,
     * with the same USB code and the same clock. ST's driver
     * (`PCD_EP_OUT_IRQHandler`, the STUP branch) and TinyUSB both decode the
     * request here and nowhere earlier.
     */
    if ((OTG_DOEPINT0 & OTG_DEPINT_STUP) != 0u) {
        OTG_DOEPINT0 = OTG_DEPINT_STUP;
        if (setup_pending) {
            setup_pending = 0;
            do_setup();
        }
    }
}

void ak_usb_init(ak_pin_t dm, ak_pin_t dp)
{
    /* The controller is on AHB2, and its pins are PA11/PA12 on alternate
     * function 10. Both are on the board's USB connector already. */
    RCC_AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    (void)RCC_AHB2ENR;
    ak_pin_af(dm, 10u, GPIO_PUPD_NONE);
    ak_pin_af(dp, 10u, GPIO_PUPD_NONE);

    /* Wait for the core to be idle, then reset it. GRSTCTL.AHBIDLE is what
     * says it is safe: a reset while the core is mid-transfer is the one way
     * to leave it in a state nothing recovers from. */
    uint32_t guard = 100000u;
    while ((OTG_GRSTCTL & OTG_GRSTCTL_AHBIDLE) == 0u && guard-- > 0u) {
    }
    OTG_GRSTCTL |= OTG_GRSTCTL_CSRST;
    guard = 100000u;
    while ((OTG_GRSTCTL & OTG_GRSTCTL_CSRST) != 0u && guard-- > 0u) {
    }

    /*
     * The phy select, device mode, and the turn-around time.
     *
     * The third one is a register an interrupt writes in the reference and
     * nothing wrote here. ST programs TRDTIM[13:10] in `DCD_HandleEnumDone_ISR`
     * (`upstream/betaflight-2026.6.1/lib/main/STM32_USB_OTG_Driver/src/
     * usb_dcd_int.c`), from a table over HCLK that gives 5 for everything from
     * 34.3 MHz to 168 MHz - so the value is 5 for any clock this part runs at.
     * This stack is polled: `OTG_GINTMSK` is set below but nothing is enabled in
     * the NVIC, so that handler never runs and the field stayed at its reset
     * value of zero.
     *
     * That reasoning produced a fix that was flashed on 2026-09-20 at 03:25.
     * **It did not change the symptom.** The host said
     * `device descriptor read/64, error -71` at 03:25:24 - the same words it
     * used before the change, and the same words it used on four other boots
     * that day. Whatever is wrong here is not this field.
     *
     * The register is still right to set; leaving it at zero is a defect on its
     * own. But it is not the fault, and an earlier version of this comment said
     * it was, in the past tense, on the strength of a flash that had not
     * happened yet. Two lessons, both already paid for: a cause announced before
     * its check is a cause that will be retracted, and "the host said exactly
     * this" is only evidence if the host said it *after* the change.
     *
     * The AT32 port writes the same field (src/arch/at32f435/usb.c). An earlier
     * version of this comment also claimed that board enumerates, and offered
     * that as the reason to believe the fix. That file's own header says
     * "Nothing here has run on the board yet", and neither port has ever
     * enumerated. There was no such evidence, and a comment that invents it is
     * worse than no comment: it is the reason this one was believed.
     */
    OTG_GUSBCFG = OTG_GUSBCFG_PHYSEL | OTG_GUSBCFG_FDMOD |
                  OTG_GUSBCFG_TRDTIM(OTG_GUSBCFG_TRDTIM_48MHZ);

    /*
     * And the two bits that decide whether a host sees a device at all.
     *
     * This core comes out of reset **powered down** and **sensing VBUS**, and
     * it puts nothing on the bus while either is true. ST's own driver writes
     * both, in this order, right after the core reset:
     *
     *   gccfg.b.pwdn = 1;                    "Deactivate the power down"
     *   gccfg.b.disablevbussensing = 1;      its own mod, for boards that do
     *                                       not wire VBUS to the sense pin
     *
     * (`upstream/betaflight-2026.6.1/lib/main/STM32_USB_OTG_Driver/src/
     * usb_core.c`, `USB_OTG_CoreInit`, the embedded-PHY branch; the same two
     * bits are in ST's HAL and in Artery's library for its own copy of this
     * core.)
     *
     * **This was missing until the firmware ran on the board.** The host tests
     * check what this driver writes against a mapped page, and the driver wrote
     * nothing here - so they were all green while the real device never
     * enumerated: the DFU bootloader handed over, the device disappeared from
     * the host, and no CDC port ever appeared. `docs/05-bringup.md` 3a has the
     * symptom. VBUS sensing is off because this board's PA9 is the receiver's
     * TX pin, not a VBUS sense line.
     *
     * **And the reference's 20 ms wait is deliberately not here.** This runs
     * from `ak_board_init()`, which `main()` calls *before* `ak_time_init()`:
     * the clock has to exist before a tick can be built on it. A tick-based
     * `ak_delay_ms()` in this function would wait for a SysTick that is not
     * running yet, which is a board that lights its LED and then says nothing
     * for ever. The connect is what the host reacts to, and it now happens at
     * the first poll rather than at the end of this function - so the phy has
     * a whole boot to settle rather than the hundred milliseconds a host would
     * have taken to enumerate a device it had just seen.
     */
    /*
     * The phy clock gate, opened before the power switch above.
     *
     * ST's driver opens it as the *first* statement of `USB_OTG_CoreInitDev`
     * - "Restart the Phy Clock", an unconditional `PCGCCTL = 0` - and this
     * tree's own AT32 port clears the same bit for the same reason
     * (`src/arch/at32f435/usb.c`: `OTG_FS_PCGCCTL &= ~OTG_PCGCCTL_STOPPCLK`).
     * Until now this port did neither, which left it the only one of the three
     * that does not touch this register at all.
     *
     * **This is written for parity. It is not known to fix anything, and the
     * board's own behaviour argues that it cannot be the fault.** RM0090 gives
     * PCGCCTL's reset value as 0x00000000, so the clear may be a no-op here;
     * and a stopped phy clock stops the D+ pull-up and stops transmission
     * outright. The host sees this device *attach*, and the app's four
     * enumerations fail at the link layer with EPROTO - which means this core
     * transmitted something. Both of those require the phy clock to be
     * running. So STOPPCLK is not set on this board: if it were, there would
     * be no attach and no -71 to explain. What this line buys is that the
     * divergence is gone, so the next reader does not have to re-run that
     * argument to establish that this is not where the fault lives.
     */
    OTG_FS_PCGCCTL &= ~OTG_PCGCCTL_STOPPCLK;

    OTG_GCCFG = OTG_GCCFG_PWRDWN | OTG_GCCFG_NOVBUSSENS;

    /* The FIFO sizes, in words, and where each one starts: the receive FIFO
     * first, then the transmit ones stacked above it. */
    OTG_GRXFSIZ = AK_USB_RX_WORDS;
    OTG_DIEPTXF(0u) = (AK_USB_EP0_WORDS << 16) | AK_USB_RX_WORDS;
    OTG_DIEPTXF(1u) = (AK_USB_EP1_WORDS << 16) | (AK_USB_RX_WORDS + AK_USB_EP0_WORDS);
    OTG_DIEPTXF(2u) = (AK_USB_EP2_WORDS << 16) |
                      (AK_USB_RX_WORDS + AK_USB_EP0_WORDS + AK_USB_EP1_WORDS);

    OTG_GAHBCFG = OTG_GAHBCFG_TXFELVL | OTG_GAHBCFG_PTXFELVL;

    /* Device mode, full speed, address zero, and the interrupts that matter
     * left unmasked so the poll above can see them: this is polled, so nothing
     * is enabled in the NVIC. */
    OTG_GINTMSK = OTG_GINT_USBRST | OTG_GINT_ENUMDNE | OTG_GINT_RXFLVL |
                  OTG_GINT_IEPINT | OTG_GINT_OEPINT;
    OTG_DCFG = OTG_DCFG_DSPD_FS;
    OTG_DIEPMSK = OTG_DEPINT_XFRC;
    OTG_DOEPMSK = OTG_DEPINT_XFRC | OTG_DEPINT_STUP;

    /* Endpoint 0, both directions: 64-byte packets, and armed for the first
     * setup packet the host sends. */
    OTG_DIEPCTL0 = OTG_DEPCTL_USBAEP | OTG_DEPCTL_EPTYP_CTRL |
                   OTG_DEPCTL_MPS(AK_USB_MPS_EP0);
    OTG_DOEPCTL0 = OTG_DEPCTL_USBAEP | OTG_DEPCTL_EPTYP_CTRL |
                   OTG_DEPCTL_MPS(AK_USB_MPS_EP0);
    ep0_out_arm();

    tx_head = 0u;
    tx_tail = 0u;
    tx_dropped = 0u;
    rx_head = 0u;
    rx_tail = 0u;
    rx_dropped = 0u;
    usb_configured = 0;
    usb_ready = 0;
    control_in_length = 0u;
    control_out_pending = 0u;
    setup_pending = 0;

    /*
     * And stay off the bus. SDIS high is the soft disconnect - it holds D+
     * down, so a host sees no device at all - and `ak_usb_poll()` clears it on
     * its first call rather than this function clearing it here.
     *
     * That is deliberate, and it is the mechanism the bench readings point at.
     * Clearing SDIS is what puts a
     * device on the bus, and from that instant the host may reset the port and
     * ask for a descriptor, on its own timeout rather than ours. Nothing drives
     * USB between here and the main loop: this stack is polled with no
     * interrupt enabled, and the only caller of the poll is the console poll in
     * the main loop. So clearing SDIS here is a promise to answer that the boot
     * path cannot keep. `ak_board_init()` returns into `selftest()`, the
     * flash-log resume, and `AK_BOOT_BLINKS` rounds of a blocking
     * `ak_delay_ms()` - six hundred milliseconds of the host talking to a
     * device that is not listening, and some seconds of boot either side of it.
     *
     * On the bench that read as `device descriptor read/64, error -71`, and
     * then as `-110` on a later attempt from the very same image: a device that
     * answers undecodably, or not at all, depending on where the host's retry
     * happened to land in the boot. Attaching from the poll instead makes the
     * two the same event - we are on the bus exactly when we are being driven -
     * and it is why the ROM DFU, which is interrupt-driven, never showed any of
     * this.
     *
     * Nothing printed before then is lost: `ak_usb_write()` buffers into the
     * transmit ring whether or not a host is there, and `ep1_push()` drains it
     * once the host has configured the device. The ring keeps the earliest
     * bytes, and the earliest bytes are the banner.
     */
    usb_detached = 1;
    OTG_DCTL |= OTG_DCTL_SDIS;
}

#if AK_USB_TRACE
/*
 * The trace's one door, and the only place the clock field can be filled: the
 * core may not include an arch header, so `ak_clk_sysclk_hz()` is visible here
 * and nowhere the caller lives.
 *
 * It is the first number `ak_board_clock_summary()` prints as the banner's
 * `clocks:` line, in hertz rather than in prose - the same measurement, put
 * somewhere that does not need a working USB port or a UART adapter to read.
 */
void ak_usb_trace_get(ak_usb_trace_t *out)
{
    if (out == 0) {
        return;
    }

    /* The measured crystal, not the configured sysclk: the one clock number
     * that is a measurement (clk.c, measure_hse). */
    out->sysclk_hz = ak_clk_hse_hz();
    out->resets    = usb_trace.resets;
    out->setups    = usb_trace.setups;
    out->sends     = usb_trace.sends;

    out->passes       = usb_trace.passes;
    out->rx_entries   = usb_trace.rx_entries;
    out->pktsts_setup = usb_trace.pktsts_setup;
    out->pktsts_data  = usb_trace.pktsts_data;
    out->pktsts_other = usb_trace.pktsts_other;
    out->gintsts_or   = usb_trace.gintsts_or;

    /* Read, not remembered: these are the core's own registers as they stand
     * at the moment of the report, which is after the host has given up. */
    out->pllcfgr = RCC_PLLCFGR;
    out->dsts    = OTG_DSTS;
    out->dctl    = OTG_DCTL;
}
#endif
