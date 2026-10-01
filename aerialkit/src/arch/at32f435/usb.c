#include "arch.h"

/*
 * USB, as a virtual serial port - the console, on the part that is not an
 * STM32.
 *
 * **This file is the F405 port's driver, re-derived for this part**, and that
 * is worth saying before the code. The two parts carry the same Synopsys
 * full-speed OTG core at the same address with the same register layout, so the
 * *design* - polled, device-only, four endpoints, a control transfer state
 * machine that a host's enumeration drives - is not two designs that happen to
 * look alike. What is not shared is the part-specific half at the bottom: the
 * clock the core needs, the pins, and three registers that behave differently,
 * all of them in `ak_usb_init`.
 *
 * **So a change to this file belongs in the F405's as well, and the reverse.**
 * That is the real cost of not sharing one implementation, and it is written
 * down here rather than discovered later. If a third part with this core turns
 * up, the part-specific half becomes a hook and the rest moves into one file;
 * with two parts, the Makefile machinery to compile one source twice (two
 * objects, two sets of renamed entry points, one of them in a different
 * include world) is not obviously cheaper than saying "the same driver, twice".
 *
 * Why it exists at all: this board's console is USART1 on a pad, and the board
 * is already on a USB port - it is how the thing is flashed. With this, the
 * same cable gives `/dev/ttyACM0` and a banner instead of a UART adapter and a
 * guess about which pad is transmit.
 *
 * Polled, like the console's UART and for the same reason: one person types at
 * it, and an interrupt for that is an interrupt to get wrong. The cost is that
 * a host can outrun us between calls - the receive FIFO absorbs that - and the
 * benefit is that everything happens in a known place, in a known order.
 *
 * Endpoints in use: 0 control, 0x82 notification (which the CDC class requires
 * and nothing here sends on), 0x81 bulk IN for the console's output, 0x02 bulk
 * OUT for what a person types. Both bulk directions are live: the banner
 * leaves by 0x81, typing arrives on 0x02, and ak_usb_read() is where the
 * console picks it up.
 *
 * Nothing here has run on the board yet. It compiles, and tests/test_arch_at32.c
 * executes it against a mapped register block - enumeration, the descriptors,
 * both bulk directions and the requests a serial driver makes - so what it does
 * is pinned as behaviour rather than as constants. Whether the silicon agrees
 * is the next bench question, and the host's kernel log is the instrument that
 * answers it.
 */

/* FIFO budget, in 32-bit words. OTG_FS has 320 words of shared FIFO RAM:
 * 128 for everything the host sends plus each IN endpoint's transmit buffer. */
#define AK_USB_RX_WORDS   128u /* receive: shared by all OUT endpoints */

/*
 * The receive FIFO, behind the same seam the F405 port has.
 *
 * Reading it is what makes the core drop its entry, so the order of the reads
 * is part of the protocol - a setup packet is two words and a data packet as
 * many as its length needs - and a page of memory cannot model that. See the
 * F405 port for the whole story; the short version is that the host test used
 * to feed both ports the same word twice, which is a packet no host sends.
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
#define AK_USB_EP0_WORDS   16u /* endpoint 0's transmit buffer */
#define AK_USB_EP1_WORDS   64u /* the console's, so a line can go out whole */
#define AK_USB_EP2_WORDS   16u /* the notification endpoint CDC requires */

#define AK_USB_MPS_EP0 64u
#define AK_USB_MPS_BULK 64u
#define AK_USB_MPS_NOTIFY 8u

/* The USB ids. The F405 board answers as the ST virtual COM port (0483:5740);
 * this one answers as **Artery's own vendor id** with the same product id
 * (2E3C:5740, which is what that part's own CDC example uses), for two reasons:
 * claiming ST's id on a part ST did not make is a small lie in a place where the
 * truth is free, and with two boards on one bench "which one is on
 * /dev/ttyACM0" is a question `lsusb` should be able to answer.
 *
 * Nothing is lost by not copying the F405's id: Linux binds `cdc_acm` to the
 * interface *class*, which is what makes either board come up as a serial port
 * with no rule of its own. */
#define AK_USB_VID 0x2E3Cu
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

/* String descriptors, in the UTF-16LE the class asks for. Index 0 is the
 * language list; 1 to 3 name the product. */
static const uint8_t string_language[] = { 4u, 0x03u, 0x09u, 0x04u };
static const uint8_t string_manufacturer[] = {
    20u, 0x03u, 'A', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0, 'K', 0, 'i', 0, 't', 0,
};
static const uint8_t string_product[] = {
    30u, 0x03u, 'A', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0, 'K', 0, 'i', 0,
    't', 0, ' ', 0, 'F', 0, '4', 0, '0', 0, '5', 0,
};
static const uint8_t string_serial[] = {
    12u, 0x03u, 'A', 0, 'K', 0, '0', 0, '0', 0, '0', 0, '1', 0,
};

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
 * Where the rest of a control IN that did not fit one packet lives - see the
 * F405 port's file for the whole story. The packet buffer is one packet long,
 * so the second packet of a 67-byte configuration descriptor has to come from
 * the table the request was answered from. Both ports had the same bug, which
 * is what two files that are near-identical by design will do.
 */
static const uint8_t *control_in_source;
static uint8_t  setup[8];

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
    /* Byte 0 of the setup buffer is bmRequestType and byte 1 is bRequest - the
     * order USB 2.0 section 9.3 puts them on the wire, and the order Artery's
     * own library reads (`usbd_sdr.c`: bmRequestType from buf[0], bRequest from
     * buf[1]). This was byte-swapped, so every request a real host makes
     * decoded as a number that matched nothing and was stalled; the F405 port
     * had the same line and the same bench symptom
     * (`device descriptor read/64, error -32`). */
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

/* One request, and then the OUT side is armed again - unless what was asked
 * for is a data stage, in which case the data has to land first and
 * receive_packet() arms it once the status stage has been sent. */
static void do_setup(void)
{
    control_out_pending = 0u;
    do_request();
    if (control_out_pending == 0u) {
        ep0_out_arm();
    }
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
         * Speed is known - full speed expected - and the global IN NAK the
         * reset left behind is released here, because this is the interrupt that
         * says the reset is over. Written up at length in the F405 port; the
         * short of it is that endpoint 0 is under that NAK with every other
         * non-periodic IN endpoint, so the device NAKs the host's descriptor
         * read until this bit is written, and enumeration never completes.
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
    /*
     * The part-specific half, in the order it has to happen.
     *
     * The clock first, because nothing else works without it: this part's USB
     * is fed from **the PLL and not from the crystal** - `misc1.hick_to_usb`
     * clear says so - and the divider then has to give 48 MHz. The port runs the
     * PLL at 288 MHz, so that divider is 6, and the encoding is the reference's
     * own (CRM_USB_DIV_6 is 0x0B, a fractional divider written as a code rather
     * than as a number). A wrong one is a device that enumerates on nothing, or
     * not at all, and there is no symptom in between to notice.
     */
    CRM_MISC1 &= ~CRM_MISC1_HICK_TO_USB;
    CRM_MISC2 = (CRM_MISC2 & ~(CRM_MISC2_USBDIV_MASK << CRM_MISC2_USBDIV_SHIFT)) |
                ((uint32_t)CRM_USBDIV_6 << CRM_MISC2_USBDIV_SHIFT);

    /* The controller's own clock, on AHB2 (bit 7 of the second enable
     * register), and its pins: PA11 is D- and PA12 is D+, function 10 on this
     * part, push-pull and with no pull of their own - the transceiver drives
     * them. Both are on the board's USB connector already. */
    CRM_AHBEN2 |= 1u << 7;
    (void)CRM_AHBEN2;
    ak_pin_af(dm, 10u, AK_GPIO_PULL_NONE);
    ak_pin_af(dp, 10u, AK_GPIO_PULL_NONE);
    /* And the stronger driver on both, which this part's own USB configuration
     * asks for: `mux` mode does not set the drive field, and the reset value is
     * not the one a 12 Mb/s pair wants. */
    ak_pin_drive(dm, GPIO_SPEED_FAST);
    ak_pin_drive(dp, GPIO_SPEED_FAST);

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
     * And the three things this part needs that the F405's core does not, each
     * of which is a device that does nothing at all if it is left out:
     *
     *   - GCCFG.PWRDOWN: this core comes out of reset *in power-down*. Artery's
     *     own library sets the same bit in the same place, right after the
     *     reset above.
     *   - PCGCCTL.STOPPCLK cleared: the phy clock is gated until it is opened.
     *   - the turn-around time. GUSBCFG bit 6 is PHYSEL on the F405 and is
     *     inside `usbtrdtim` here, so the F405's line would write zero into this
     *     field. The value is the reference's: 5 for a 48 MHz phy clock, written
     *     by its own device library when enumeration completes - which is before
     *     any packet moves, so writing it here is the same thing earlier.
     */
    /* Out of power-down, and VBUS sensing off. The second bit came from the
     * other target: on 2026-09-17 the F405's driver put *nothing* on the USB
     * bus because it never wrote these two bits at all, and the host saw a
     * board that looked unplugged while the ROM's own bootloader had enumerated
     * on the same cable. ST's driver and Artery's set both; this core is the
     * same Synopsys device core with the same two bits in the same place, and
     * this firmware is a device with no host role - so there is no reason to
     * wait for a VBUS sense line the board may not wire. */
    OTG_GCCFG |= OTG_GCCFG_PWRDOWN | OTG_GCCFG_NOVBUSSENS;
    OTG_FS_PCGCCTL &= ~OTG_PCGCCTL_STOPPCLK;
    OTG_GUSBCFG = OTG_GUSBCFG_USBTRDTIM(OTG_GUSBCFG_TRDTIM_48MHZ) |
                  OTG_GUSBCFG_FDMOD;

    /* The FIFO sizes, in words, and where each one starts: the receive FIFO
     * first, then the transmit ones stacked above it. The budget is the same
     * 320 words the F405's core has, which is what makes this part of the file
     * identical rather than merely similar - the reference's own header says
     * OTG_FIFO_SIZE 320. */
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
     * Clearing SDIS is what puts a device on the bus, and from that instant the
     * host may reset the port and ask for a descriptor, on its own timeout
     * rather than ours. This stack is polled with no interrupt enabled, so
     * between here and the main loop nothing drives USB at all and the boot
     * path runs on with the host already talking to a device that is not
     * listening - which is a device descriptor read that fails at the link
     * layer, or times out, depending on where the host's retry landed.
     * Attaching from the poll makes the two the same event: we are on the bus
     * exactly when we are being driven.
     *
     * The F405 port carries the bench readings this was derived from; this one
     * has never been flashed, and is changed to keep the two in step.
     *
     * Nothing printed before then is lost: `ak_usb_write()` buffers into the
     * transmit ring whether or not a host is there, and the transmit path
     * drains it once the host has configured the device. The ring keeps the
     * earliest bytes, and the earliest bytes are the banner.
     */
    usb_detached = 1;
    OTG_DCTL |= OTG_DCTL_SDIS;
}
