#include "host_usb_model.h"

/*
 * A receive FIFO with an order, for the host build: see host_usb_model.h for
 * why a page of memory is not enough and what this cost when it was not.
 */

#define HOST_USB_FIFO_WORDS 512u

static uint32_t words[HOST_USB_FIFO_WORDS];
static unsigned head;   /* the next word to pop */
static unsigned tail;   /* where the next word goes */
static unsigned queued; /* how many are in between */

void host_usb_reset(void)
{
    head = 0u;
    tail = 0u;
    queued = 0u;
}

void host_usb_push(uint32_t word)
{
    if (queued >= HOST_USB_FIFO_WORDS) {
        return; /* a test that overruns this is a test to fix, not a FIFO */
    }
    words[tail] = word;
    tail = (tail + 1u) % HOST_USB_FIFO_WORDS;
    queued++;
}

uint32_t host_usb_fifo_read(void)
{
    uint32_t word;

    if (queued == 0u) {
        /* Nothing was queued for this read, which means the test described a
         * packet with fewer words than its own length needs. Zero rather than
         * a crash, so the checks that follow report it. */
        return 0u;
    }
    word = words[head];
    head = (head + 1u) % HOST_USB_FIFO_WORDS;
    queued--;
    return word;
}

unsigned host_usb_words_queued(void)
{
    return queued;
}
