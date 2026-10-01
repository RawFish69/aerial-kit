#include <stdint.h>

#include "arch.h"
#include "ak_boot.h"
#include "ak_fault.h"

/*
 * Fault capture for the Cortex-M4.
 *
 * The vector table routes the hard fault here; the other configurable faults
 * are disabled at reset on the M4, so they escalate to a hard fault and arrive
 * at this same path. The CFSR still says which one it was.
 */

__attribute__((section(".noinit"), used))
ak_fault_record_t ak_fault;

void ak_fault_capture(uint32_t *frame);
void ak_fault_record(uint32_t *frame);

int ak_fault_present(void)
{
    return ak_fault.magic == AK_FAULT_MAGIC;
}

void ak_fault_clear(void)
{
    ak_fault.magic = 0;
    ak_fault.count = 0;
}

/*
 * The record itself, and then the stop.
 *
 * These are two functions rather than one so that the *record* can be run
 * somewhere other than a fault: the frame layout below is the part that has to
 * be right - r0, r1, r2, r3, r12, lr, pc, xpsr, in the order the hardware
 * pushed them - and an off-by-one there reports the wrong instruction as the
 * one that faulted, which is worse than reporting nothing. `ak_fault_capture`
 * is what the vector table reaches through `startup.c`; `ak_fault_record` is
 * what `tests/test_fault.c` calls with a frame it made itself, because the
 * loop below it never returns and a test that cannot return is not a test.
 */
void ak_fault_record(uint32_t *frame)
{
    /* The stacked frame is r0, r1, r2, r3, r12, lr, pc, xpsr - the same eight
     * words whether or not the extended (floating point) frame follows. */
    ak_fault.count++;
    ak_fault.r0 = frame[0];
    ak_fault.r1 = frame[1];
    ak_fault.r2 = frame[2];
    ak_fault.r3 = frame[3];
    ak_fault.r12 = frame[4];
    ak_fault.lr = frame[5];
    ak_fault.pc = frame[6];
    ak_fault.psr = frame[7];

    ak_fault.cfsr = AK_REG32(0xE000ED28u);
    ak_fault.hfsr = AK_REG32(0xE000ED2Cu);
    ak_fault.mmfar = AK_REG32(0xE000ED34u);
    ak_fault.bfar = AK_REG32(0xE000ED38u);

    /* And where the frame was, which is the only field here that lets somebody
     * walk the stack afterwards: the stacked words are at this address. */
    ak_fault.frame = (uint32_t)(uintptr_t)frame;

    ak_fault.magic = AK_FAULT_MAGIC; /* last: a partial record is not trusted */
}

void ak_fault_capture(uint32_t *frame)
{
    ak_fault_record(frame);
#if AK_BOOT_STAGE
    /*
     * The tell-tale build says "a fault, not a hang": ten quick blinks, a
     * pause, and then the stage the boot had reached, which is the number a
     * person reads off the LED. The board's own code writes the pin, because
     * which pin and which way round it is is the board's business and this file
     * is shared by both ARM parts - and the pins are written directly there, so
     * this does not depend on anything the fault may have broken.
     */
    ak_boot_mark(AK_BOOT_FAULT);
#endif
#if AK_FAULT_REBOOT
    /*
     * Take the comment below literally, in the one build that can afford to.
     *
     * The record is in `.noinit` and main() prints it at the banner, so the
     * only thing missing on a bench is something to perform the reset. The
     * watchdog would, in the air; here nothing does, and the CPU stops
     * answering the console's polled door along with everything else - so the
     * record that is sitting there correct is also unreadable. See the flag's
     * note in ak_fault.h.
     *
     * AIRCR is written directly rather than through CMSIS, for the same reason
     * the status registers above are: this file is the port's, and it does not
     * want a vendor header to compile. VECTKEY 0x05FA is what makes the write
     * take; without it the register ignores the store.
     */
    AK_REG32(0xE000ED0Cu) = 0x05FA0004u; /* AIRCR = VECTKEY | SYSRESETREQ */
#endif
    for (;;) {
        /* Stop here. The fault survives the reset; the next boot prints it. */
    }
}
