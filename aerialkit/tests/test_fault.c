/*
 * The fault record, which is the file that explains a board that stops saying
 * anything.
 *
 * `src/arch/arm/cortex-m4/fault.c` had never been executed anywhere: it is
 * reached from the vector table on a real fault, and a fault is not something
 * a test can arrange on a target. What a host *can* do is the half that has to
 * be right - the record itself - because the frame layout is fixed by the
 * hardware: r0, r1, r2, r3, r12, lr, pc, xpsr, in that order, whether or not
 * the extended (floating-point) frame follows.
 *
 * An off-by-one in that layout does not fail loudly. It reports the *wrong
 * instruction* as the one that faulted, and the person reading
 * `arm-none-eabi-addr2line` afterwards is sent to a line that was fine - which
 * is why this file exists, and why the record half is a function of its own
 * (`ak_fault_record()`) with the stop after it in `ak_fault_capture()`.
 *
 * The register block is mapped by tests/test_arch.c; this asks whether that has
 * happened and maps the one page it needs if it has not.
 */

#include <stdio.h>
#include <string.h>

#include "ak_fault.h"
#include "tests.h"

void ak_fault_record(uint32_t *frame);

#define CFSR   0xE000ED28UL
#define HFSR   0xE000ED2CUL
#define MMFAR  0xE000ED34UL
#define BFAR   0xE000ED38UL

static volatile uint32_t *reg(uint32_t address)
{
    return (volatile uint32_t *)(uintptr_t)address;
}

void test_fault(void)
{
    /* The four status registers below are in the system control block, so that
     * is the one page this needs, and it maps it here rather than asking
     * whether the port's own test has been through - the answer to that is a
     * fact about the order the tests run in. */
    expect("the system control block is mapped for this check",
           ak_test_map_system_control());
    if (!ak_test_system_control_mapped()) {
        return; /* the writes below would land on an unmapped page */
    }

    /* A fault, as the hardware would have left it: the frame on the stack and
     * the status registers set. The values are distinctive on purpose - a
     * record that reads the frame one word out is a record that sends somebody
     * to the wrong line, and only values that differ can show it. */
    uint32_t frame[8];
    frame[0] = 0x11111111u; /* r0 */
    frame[1] = 0x22222222u; /* r1 */
    frame[2] = 0x33333333u; /* r2 */
    frame[3] = 0x44444444u; /* r3 */
    frame[4] = 0x55555555u; /* r12 */
    frame[5] = 0x66666666u; /* lr */
    frame[6] = 0x08001B42u; /* pc: an address in flash, which is the point */
    frame[7] = 0x61000000u; /* xpsr */

    ak_fault_clear();
    expect("a cleared record is not a fault", !ak_fault_present());

    *reg(CFSR) = 0x00000082u;
    *reg(HFSR) = 0x40000000u;
    *reg(MMFAR) = 0x00000000u;
    *reg(BFAR) = 0x00000000u;

    /* And an unfinished record is not one either: everything but the magic is
     * written, which is what a record interrupted by the reset it caused looks
     * like. */
    ak_fault.cfsr = 0xDEADBEEFu;
    expect("a record with no magic is not trusted",
           !ak_fault_present() && ak_fault.cfsr == 0xDEADBEEFu);

    ak_fault_record(frame);

    expect("a fault is recorded", ak_fault_present());
    expect("with the count of faults since power-on",
           ak_fault.count == 1u);
    expect("and the instruction that faulted, which is the frame's pc",
           ak_fault.pc == frame[6]);
    expect("not the address of the handler that recorded it",
           ak_fault.pc != (uint32_t)(uintptr_t)&frame[0]);
    expect("with the registers the faulting code was using",
           ak_fault.r0 == frame[0] && ak_fault.r1 == frame[1] &&
               ak_fault.r2 == frame[2] && ak_fault.r3 == frame[3] &&
               ak_fault.r12 == frame[4]);
    expect("and the link register and status it left behind",
           ak_fault.lr == frame[5] && ak_fault.psr == frame[7]);
    expect("the configurable fault status is the part's own register",
           ak_fault.cfsr == 0x00000082u);
    expect("so are the hard fault status and the two addresses",
           ak_fault.hfsr == 0x40000000u && ak_fault.mmfar == 0u &&
               ak_fault.bfar == 0u);
    /* And where the frame was. It is the one field a reader cannot rebuild
     * from the others, and it is what makes the stack walkable afterwards: the
     * words in the record are copies, and the originals are at this address. */
    expect("and it says where the stacked registers still are",
           ak_fault.frame == (uint32_t)(uintptr_t)frame);

    /* A second fault, without a reset in between: the count is what tells a
     * person whether the board is in a loop of resets or stopped once. */
    frame[6] = 0x0800CAFEu;
    ak_fault_record(frame);
    expect("a second fault has its own pc and a count of two",
           ak_fault.pc == 0x0800CAFEu && ak_fault.count == 2u);

    /* And clearing leaves the fields alone: it is the magic that says whether
     * a record is worth reading, which is what makes a partially written one
     * safe to find on the next boot. */
    ak_fault_clear();
    expect("clearing forgets the record without pretending to erase it",
           !ak_fault_present() && ak_fault.pc == 0x0800CAFEu &&
               ak_fault.count == 0u);
}
