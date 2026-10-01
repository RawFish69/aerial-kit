#include "arch.h"

/*
 * Reset and the ROM bootloader: the two ways this firmware stops being the
 * thing that is running. Both are a couple of instructions and both are the
 * part's rather than the core's, which is why they are here and not in the
 * shared reset path.
 *
 * The host build has no AIRCR to write and nothing at 0x1FFF0000 to jump to,
 * and it compiles this file anyway because a *board* file linked into the test
 * binary calls these. On the host they do nothing, which is the honest answer:
 * see the guard at the bottom.
 */
#ifdef AK_HOST_SYSTEM

void ak_arch_reset(void)
{
}

void ak_arch_bootloader(void)
{
}

#else

void ak_arch_reset(void)
{
    /* A system reset through AIRCR: the same thing the reset pin does, without
     * needing a hand on the board. The key in the top half is required or the
     * write is ignored. */
    __asm volatile("dsb");
    SCB_AIRCR = SCB_AIRCR_SYSRESETREQ;
    __asm volatile("dsb");
    for (;;) {
        /* Wait for the reset to arrive. */
    }
}

/*
 * The other way out of the firmware: hand the part to its ROM bootloader, which
 * is where DFU lives.
 *
 * Not a jump from the running firmware. That was the first shape of this and
 * 2026-09-27 measured it not to work (below); what it does now is leave a word
 * in the backup domain and reset, and ak_arch_early() finishes the hand-over at
 * the top of the next boot, from a part that is as reset left it.
 *
 * **The jump-from-firmware version has been run on a board once, on 2026-09-27,
 * and it did not get to DFU.** The image carrying it wrote its trace record,
 * called this, and the host watched `lsusb -d 0483:df11` for 170 s: the ROM's
 * device never appeared, and 145 s later the *application's* device attached
 * again. So a bare branch to 0x1FFF0000 is not a way in - the ROM reads BOOT0
 * at reset, and a branch is not a reset. It found the pin low and booted flash,
 * which is what that second attach is. The reset-and-remap path below supplies
 * the two things that branch lacked: a real reset before the jump, and SYSCFG's
 * remap of system memory to address 0.
 *
 * That path reached the ROM's DFU on the bench board on 2026-09-30: the
 * console's `dfu` command was run twice and `0483:df11` appeared both times.
 * What the measurement does not separate is the two ways that could have
 * happened - this code, or BOOT0 having been left high, which resets into the
 * ROM whatever the firmware does. Naming the experiment is the honest state:
 * a plain `reboot` at the console, which does not set the backup word, then a
 * look at whether the application or the ROM came back. See docs/05-bringup.md
 * 6c.
 */
#define AK_BKP_ROM   0xB007DF00u /* next boot: go straight to the ROM */
#define AK_BKP_CLEAN 0xC1EA4B00u /* this boot came through a real reset */

static void backup_open(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_PWREN;
    (void)RCC_APB1ENR;
    PWR_CR |= PWR_CR_DBP;
}

static void rom_jump(void)
{
    __asm volatile(
        /* no cpsid: this runs straight after a reset, and the ROM DFU is interrupt-driven */
        "mov r3, %0       \n\t"
        "ldr r0, [r3]     \n\t" /* the ROM's stack pointer */
        "msr msp, r0      \n\t"
        "ldr r0, [r3, #4] \n\t" /* the ROM's reset handler */
        "bx r0            \n\t"
        :
        : "r"(AK_BOOTLOADER_BASE)
        : "r0", "r3", "memory");
}

/*
 * The ROM, reached the way Betaflight and INAV reach it: leave a word in the
 * backup domain, reset, and jump from the top of the reset handler - where the
 * part is as reset left it, with no PLL, no peripherals and nothing pending.
 * The jump above, taken from a running firmware, is what 2026-09-27 measured
 * not to work; SYSCFG's remap to system memory is the other half it lacked.
 */
void ak_arch_bootloader(void)
{
    backup_open();
    RTC_BKP0R = AK_BKP_ROM;
    ak_arch_reset();
}

/*
 * Called first thing in Reset_Handler, before .data and .bss exist - so no
 * globals, and nothing but registers.
 *
 * Two jobs. A boot the previous one asked to be the ROM becomes the ROM. And
 * every other boot is made to have come through a real reset: dfu-util's
 * `:leave` starts this image by a jump from the ROM, with the ROM's clocks and
 * its USB core still configured, and on 2026-09-28 an image started that way
 * behaved differently from the same image after the reset pin (its trace save
 * never landed). One extra reset per boot costs microseconds and makes the two
 * starts the same start.
 */
void ak_arch_early(void)
{
    uint32_t word;

    backup_open();
    word = RTC_BKP0R;
    if (word == AK_BKP_ROM) {
        RTC_BKP0R = 0u;
        RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;
        (void)RCC_APB2ENR;
        SYSCFG_MEMRMP = 1u; /* system memory at 0 */
        rom_jump();
    }
    if (word != AK_BKP_CLEAN) {
        RTC_BKP0R = AK_BKP_CLEAN;
        ak_arch_reset();
    }
    RTC_BKP0R = 0u;
}

#endif /* AK_HOST_SYSTEM */
