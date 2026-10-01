#ifndef AK_CORE_AK_FAULT_H
#define AK_CORE_AK_FAULT_H

#include <stdint.h>

/*
 * What a crash left behind.
 *
 * A board that resets into a quiet loop is a board that tells you nothing. The
 * record below lives in RAM that startup does not clear, so the fault survives
 * the reset the watchdog (or the debugger) causes, and the next boot prints it.
 * That is the difference between "it stops sometimes" and "it stops at
 * 0x08001B42 with an imprecise bus fault".
 *
 * The fault handler itself prints nothing: by then the stack may be the thing
 * that is broken, and a console write from a fault context is how a bad fault
 * becomes an invisible one. It records, and it stops.
 */

typedef struct {
    uint32_t magic;    /* set last, so a partial record is not trusted */
    uint32_t count;    /* faults recorded since the last power-on */
    uint32_t cfsr;     /* configurable fault status */
    uint32_t hfsr;     /* hard fault status */
    uint32_t mmfar;    /* memory management fault address */
    uint32_t bfar;     /* bus fault address */
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;       /* where it was called from */
    uint32_t pc;       /* where it was */
    uint32_t psr;
    /* Where the hardware pushed that frame. It is the one field a person
     * cannot reconstruct from the rest: the stacked words are *at* this
     * address, so a debugger (or `addr2line` and a memory dump) can walk
     * backwards from here and see what the code that faulted was doing. The
     * stack pointer at the fault is 32 bytes above it for a basic frame, and
     * 104 for one with the floating-point state in it. */
    uint32_t frame;
} ak_fault_record_t;

#define AK_FAULT_MAGIC 0x414B4654u /* "AKFT" */

/*
 * Who performs the reset the record above survives.
 *
 * In a flight image: not this file. The handler records and stops, and the
 * part that knows the machine has just proved it cannot be trusted - the
 * watchdog - is what puts it back. That is the right division in the air.
 *
 * On a bench there is no watchdog, and the stop has a consequence the
 * paragraph above does not mention. `for (;;)` is not a quiet failure on a
 * board whose console is polled: the poll lives in the main loop, so a CPU
 * stopped in the handler stops answering USB as well, and the host reports a
 * device that pulled up D+ and then went silent. The record in RAM is correct
 * and complete the whole time, and nothing can read it - not the console,
 * which cannot transmit, and not a debugger, which is not attached to a board
 * somebody is holding.
 *
 * `-DAK_FAULT_REBOOT=1` has the handler request the reset itself, which turns
 * that dead end into a beacon: the board restarts, the next boot prints the
 * record at the banner and clears it, faults again, and repeats. A board left
 * plugged in says the same sentence over and over until somebody reads it.
 *
 * This is a bring-up instrument, not a flight behaviour: in the air a reset is
 * a restart nobody asked for, and the watchdog is the thing allowed to decide
 * that. Off by default, and the default compiles to nothing.
 */
#ifndef AK_FAULT_REBOOT
#define AK_FAULT_REBOOT 0
#endif

/* Provided by the port, in RAM startup does not clear. */
extern ak_fault_record_t ak_fault;

int  ak_fault_present(void);
void ak_fault_clear(void);

#endif /* AK_CORE_AK_FAULT_H */
