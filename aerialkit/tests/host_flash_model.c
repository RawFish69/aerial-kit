/*
 * The modelled flash controller behind the seam in src/arch/stm32f405/
 * flash.c. See host_flash_model.h for what it is and what it is not.
 *
 * Two things are worth saying about the shape of it. The register offsets and
 * bit positions come from the port's own regs.h rather than being copied,
 * because they are the hardware contract and tests/test_regs.c is what pins
 * them; what this file owns is the *behaviour* - what the part does with a
 * key, a start bit and a word - and that is the part that had never run.
 *
 * And everything a real controller does asynchronously, this does at once:
 * there is no busy period to wait out, because the wait is the driver's and
 * what the driver does with it is not what is under test here.
 */

#include <string.h>

#include "../src/arch/stm32f405/regs.h"
#include "host_flash_model.h"

#define MODEL_FLASH_BASE AK_FLASH_BASE
#define MODEL_FLASH_SIZE 0x00100000u

static uint32_t cr;     /* the control register, the lock bit included */
static uint32_t sr;     /* the status register */
static uint32_t keys;   /* the last key written to the key register */
static uint32_t erases; /* sector erases performed since the reset */
static int      key_stage;      /* 0 nothing, 1 the first key, 2 unlocked */
static int      unlock_refused; /* the test asks for a controller that will
                                   not take the keys */
static int      stuck_busy;     /* the test asks for one whose operation never
                                   finishes */
static int      program_refused; /* the test asks for one that erases and will
                                    not program */

/*
 * Where a sector starts and how big it is. A 1 MB STM32F405 is four sectors of
 * 16 KB, one of 64 KB and seven of 128 KB (RM0090 table 3) - not eight or
 * sixteen of one size, which is what a model that is wrong about this looks
 * like from the driver's side: nothing, because the driver never sees an
 * address, only a sector number.
 */
static void sector_range(unsigned sector, uintptr_t *base, uint32_t *size)
{
    if (sector < 4u) {
        *base = MODEL_FLASH_BASE + (uintptr_t)sector * 0x4000u;
        *size = 0x4000u;
    } else if (sector == 4u) {
        *base = MODEL_FLASH_BASE + 0x10000u;
        *size = 0x10000u;
    } else {
        *base = MODEL_FLASH_BASE + 0x20000u + (uintptr_t)(sector - 5u) * 0x20000u;
        *size = 0x20000u;
    }
}

void host_flash_model_reset(void)
{
    cr = FLASH_CR_LOCK;
    sr = 0u;
    keys = 0u;
    erases = 0u;
    key_stage = 0;
    unlock_refused = 0;
    stuck_busy = 0;
    program_refused = 0;
    memset((void *)(uintptr_t)MODEL_FLASH_BASE, 0xFF, MODEL_FLASH_SIZE);
}

void host_flash_model_set_locked(int locked)
{
    if (locked) {
        cr |= FLASH_CR_LOCK;
        key_stage = 0;
    } else {
        /* Somebody else holds it open. No real part reaches this state, and
         * the driver is supposed to refuse to touch it. */
        cr &= ~FLASH_CR_LOCK;
        key_stage = 2;
    }
}

void host_flash_model_set_unlock_refused(int refused)
{
    unlock_refused = refused != 0;
}

void host_flash_model_set_stuck_busy(int stuck)
{
    stuck_busy = stuck != 0;
}

void host_flash_model_set_program_refused(int refused)
{
    program_refused = refused != 0;
}

uint32_t host_flash_model_keys(void)
{
    return keys;
}

uint32_t host_flash_model_erases(void)
{
    return erases;
}

uint32_t host_flash_reg_read(uint32_t offset)
{
    if (offset == FLASH_OFF_SR) {
        /* A busy bit that never clears is what the driver's guard is for: the
         * part took the operation and has not finished it, and a driver that
         * waited on it for ever is a board that never says anything again. */
        return stuck_busy ? (sr | FLASH_SR_BSY) : sr;
    }
    if (offset == FLASH_OFF_CR) {
        return cr;
    }
    return 0u;
}

/* Writing one to a clearable status bit clears it, which is how the driver
 * clears end-of-program and the error flags between operations. */
static void write_status(uint32_t value)
{
    sr &= ~(value & (FLASH_SR_EOP | FLASH_SR_ERRORS));
}

/* The key sequence: the first key, then the second, and the lock bit goes.
 * Anything else puts the sequence back to the start, as the part does. */
static void write_key(uint32_t value)
{
    keys = value;

    if (unlock_refused) {
        return;
    }
    if (value == FLASH_KEY1 && key_stage == 0) {
        key_stage = 1;
        return;
    }
    if (value == FLASH_KEY2 && key_stage == 1) {
        key_stage = 2;
        cr &= ~FLASH_CR_LOCK;
        return;
    }
    key_stage = 0;
}

static void write_control(uint32_t value)
{
    /* The lock bit can be set by any write and cleared only by the keys; the
     * rest of the writable fields are whatever was written this time. */
    uint32_t locked = cr & FLASH_CR_LOCK;
    cr = locked | (value & ~(FLASH_CR_LOCK | FLASH_CR_STRT));
    if ((value & FLASH_CR_LOCK) != 0u) {
        cr |= FLASH_CR_LOCK;
        key_stage = 0;
    }

    if ((value & FLASH_CR_STRT) == 0u) {
        return;
    }

    /* Started, and never finished: the start bit clears as the part latches
     * it, and nothing is erased - which is the difference between "the sector
     * did not erase" and "the sector erased and then something else failed"
     * that the checks below depend on. */
    if (stuck_busy) {
        cr &= ~FLASH_CR_STRT;
        return;
    }

    /* The start bit: the part clears it and does what the other bits say. An
     * erase empties its sector; anything else in this driver's vocabulary has
     * nothing to start. */
    cr &= ~FLASH_CR_STRT;
    if ((cr & FLASH_CR_SER) != 0u) {
        uintptr_t base = 0u;
        uint32_t size = 0u;

        sector_range((cr & FLASH_CR_SNB(0xFu)) >> 3, &base, &size);
        memset((void *)base, 0xFF, size);
        erases++;
        sr |= FLASH_SR_EOP;
    }
}

void host_flash_reg_write(uint32_t offset, uint32_t value)
{
    if (offset == FLASH_OFF_SR) {
        write_status(value);
    } else if (offset == FLASH_OFF_CR) {
        write_control(value);
    } else if (offset == FLASH_OFF_KEYR) {
        write_key(value);
    }
}

void host_flash_word_write(uint32_t address, uint32_t value)
{
    if ((cr & FLASH_CR_PG) == 0u) {
        return; /* not in programming mode: the part ignores the write */
    }
    if (address < MODEL_FLASH_BASE ||
        address + 4u > MODEL_FLASH_BASE + MODEL_FLASH_SIZE) {
        return;
    }

    uint32_t *word = (uint32_t *)(uintptr_t)address;
    if (program_refused || *word != 0xFFFFFFFFu) {
        /* A word can only be programmed where the sector was erased first.
         * The part refuses and says so, and a model that wrote anyway would
         * turn a save without an erase into a save. */
        sr |= FLASH_SR_PGAERR;
        return;
    }
    *word = value;
    sr |= FLASH_SR_EOP;
}
