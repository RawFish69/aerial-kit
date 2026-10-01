#ifndef AK_HOST_USB_MODEL_H
#define AK_HOST_USB_MODEL_H

/*
 * The USB receive FIFO, modelled, for the host build of tests/test_arch.c and
 * tests/test_arch_at32.c.
 *
 * The driver reads FIFO 0 one word at a time, and on the part *reading* is what
 * makes the core drop its queue entry: the order of the reads is part of the
 * protocol. A page of memory cannot say that - every read of it returns the
 * same word - and the first version of the USB tests did exactly that, feeding
 * a setup packet whose first two bytes were the *request* rather than the
 * bmRequestType a host puts there. The driver and the test agreed with each
 * other and neither agreed with USB: on the bench the device appeared on the
 * bus and stalled every request it was sent (`device descriptor read/64, error
 * -32`), which is what a request decoded out of the wrong byte looks like.
 *
 * So the model is a queue. The test pushes the words a packet's bytes make and
 * the driver pops them in order; a read of an empty queue is a bug in the test
 * rather than something the firmware did, and `host_usb_words_queued()` is how
 * a test can say so.
 */

#include <stdint.h>

void     host_usb_reset(void);
void     host_usb_push(uint32_t word);
uint32_t host_usb_fifo_read(void);
unsigned host_usb_words_queued(void);

#endif /* AK_HOST_USB_MODEL_H */
