#include "arch.h"

/*
 * The host build of this file is a pair of empty functions: there is no AIRCR
 * to write and nothing at 0x1FFF0000 to jump to in a process on a Pi. They are
 * here because a *board* file linked into the test binary calls them - the
 * AT32 board is one of the two the tests link now - and the honest host
 * behaviour is to do nothing rather than to pretend. See the guard below.
 *
 * **The ROM entry cannot be host-tested and is not.** `ak_arch_early` is a
 * vector-table jump into the part's ROM taken from the top of Reset_Handler;
 * there is nothing at that address in a process, and the `.noinit` word it
 * reads is the part's RAM. What the host does check is the address (test 2 of
 * `tests/test_arch_at32.c`) and that the board offers the hand-over at all
 * (`tests/test_board_ghf435.c`). Everything below the jump is the bench's.
 */
#ifdef AK_HOST_SYSTEM

void ak_arch_reset(void)
{
}

void ak_arch_bootloader(void)
{
}

#else

/*
 * A reset, on the part whose reset is the core's.
 *
 * The two ports' system.c files are near-identical on purpose: AIRCR is a
 * Cortex-M4 register, not a vendor one, so the reset a board asks for is the
 * same write on both parts. What is worth saying out loud is what it is *not*:
 * this does not go through the ROM bootloader, does not re-enter a DFU mode,
 * and does not preserve anything in ordinary RAM - it is the reset pin, by
 * software, with the same consequences. (`.noinit` survives it, which is what
 * the boot word below and the retained log both rely on.)
 */

void ak_arch_reset(void)
{
    __asm volatile("dsb");
    SCB_AIRCR = SCB_AIRCR_SYSRESETREQ;
    __asm volatile("dsb");
    for (;;) {
        /* Wait for the reset to arrive. */
    }
}

/*
 * The boot word, and where it lives.
 *
 * The STM32F405 port keeps this in the backup domain, in a register that is
 * powered independently of the core. This part's equivalent - the ERTC's
 * retained registers - is behind a clock and a write-protection sequence of its
 * own, and it is not needed: a *system* reset does not clear SRAM, `.noinit` is
 * the section the linker already keeps out of startup's clear loop (it is where
 * the fault record and the retained log live), and nothing here has to survive
 * a power cycle - a boot word left behind by a power loss would be a fault, not
 * a feature.
 *
 * The two words are the F405's mechanism in this part's storage: one says "the
 * next boot is the ROM's", the other says "this boot came through a real
 * reset". Neither is ever both.
 */
#define AK_BOOT_WORD_ROM   0xA732DF00u /* next boot: hand over to the ROM */
#define AK_BOOT_WORD_CLEAN 0xA732C1E0u /* this boot came through a real reset */

__attribute__((section(".noinit"), used))
static uint32_t boot_word;

/*
 * Jump into the ROM at 0x1FFF0000.
 *
 * This is *not* how the part reaches its bootloader, and the F405 port paid for
 * that measurement: an image that branched here from a running firmware wrote
 * its trace record, took the branch, and the host watched for the ROM's USB
 * device for 170 s and saw the application's own device attach again 145 s
 * later. A branch is not a reset, and this part's ROM reads its boot strap at
 * reset exactly as the STM32's does. What works is to make address 0 say boot
 * memory and then reset, which is what ak_arch_bootloader() and the two arms of
 * ak_arch_early() do.
 *
 * So this is only reached *after* a real reset, from the top of Reset_Handler,
 * where the part is as reset left it - no PLL, no peripherals, nothing pending
 * - and the ROM's own first two words are a stack pointer and a reset handler.
 * There is deliberately no `cpsid`: the ROM's DFU is interrupt-driven, and
 * disabling interrupts on the way in is how the F405's first attempt at this
 * produced a silent hand-over.
 */
static void rom_jump(void)
{
    __asm volatile(
        "mov r3, %0       \n\t"
        "ldr r0, [r3]     \n\t" /* the ROM's stack pointer */
        "msr msp, r0      \n\t"
        "ldr r0, [r3, #4] \n\t" /* the ROM's reset handler */
        "bx r0            \n\t"
        :
        : "r"(AK_BOOTLOADER_BASE)
        : "r0", "r3", "memory");
}

/* `SCFG_CFG1.mem_map_sel` is what address 0x00000000 is, and SCFG's clock has
 * to be on before the register can be written - it is on APB2, bit 14 of
 * CRM_APB2EN. Both halves are in regs.h with the citation. */
static void map_boot_memory(int boot)
{
    CRM_APB2EN |= CRM_APB2EN_SCFG;
    (void)CRM_APB2EN;

    SCFG_CFG1 = (SCFG_CFG1 & ~(uint32_t)SCFG_CFG1_MEM_MAP_MASK) |
                (boot ? SCFG_MEM_MAP_BOOT : SCFG_MEM_MAP_MAIN);
}

/*
 * The console's `dfu`, and the whole reason this file has a second entry point:
 * leave the word, reset, and let ak_arch_early() below take the part to the ROM
 * from a state the ROM can actually start in. On this board that is the
 * difference between reflashing from a terminal and opening the case - the
 * wing's flight controller enters DFU with a button *and* a solder joint, and
 * the firmware this replaced, INAV, could reach the ROM by command.
 *
 * Not run on a board, on either part: what was measured on 2026-09-27 was the
 * branch this replaced, not this.
 */
void ak_arch_bootloader(void)
{
    boot_word = AK_BOOT_WORD_ROM;
    ak_arch_reset();
}

/*
 * Called first thing in Reset_Handler, before .data and .bss exist - so no
 * globals with initialisers, nothing but registers and `.noinit`, which startup
 * does not clear.
 *
 * Two jobs, both of them the F405 port's, and the second one is the one that
 * was learned rather than designed:
 *
 *   - a boot the previous one asked to be the ROM's becomes the ROM: the map is
 *     pointed at boot memory and the jump is taken. The word is cleared first,
 *     so a ROM that hands back to flash does not come round again.
 *   - every other boot is made to have come through a real reset. dfu-util's
 *     `:leave` starts this image by a jump from the ROM, with the ROM's clocks
 *     and its USB core still configured; on 2026-09-28 an F405 image started
 *     that way behaved differently from the same image after the reset pin - its
 *     trace save never landed. One extra reset per boot costs microseconds and
 *     makes the two starts the same start, and this port has the same ROM and
 *     the same `:leave`.
 *
 * That reset also puts the memory map back, because SCFG_CFG1 is not documented
 * as surviving one and a firmware that came in through the ROM should not be
 * running with the map still pointing at it. The map is set to main memory
 * explicitly rather than assumed, which is one register write and no clock
 * spent.
 */
void ak_arch_early(void)
{
    uint32_t word = boot_word;

    if (word == AK_BOOT_WORD_ROM) {
        boot_word = 0u;
        map_boot_memory(1);
        rom_jump();
    }
    if (word != AK_BOOT_WORD_CLEAN) {
        boot_word = AK_BOOT_WORD_CLEAN;
        ak_arch_reset();
    }
    boot_word = 0u;
    map_boot_memory(0);
}

#endif /* AK_HOST_SYSTEM */
