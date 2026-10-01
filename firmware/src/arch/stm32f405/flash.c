#include "arch.h"

/*
 * The internal flash controller (RM0090 section 3).
 *
 * Two operations, both of them things that can leave a board needing its
 * bootloader: erasing a sector and programming words. Neither can be tested
 * without hardware, so both are written to fail loudly rather than quietly:
 * every error flag in FLASH_SR is checked after every step, and the sector
 * number is passed in by the caller rather than assumed.
 *
 * The write granularity is set to 32 bits explicitly. It resets to byte
 * programming, and a word write while the controller expects a byte sets
 * PGAERR and does nothing - which looks exactly like a save that silently
 * never persisted. 32 bits is correct for the 3.3 V supply this board runs at.
 */

#define AK_FLASH_TIMEOUT 1000000u

/*
 * The three places every access goes through, so that a host build can
 * intercept them (tests/host_flash_model.c) and this file can be run against a
 * modelled controller rather than a real one: a register read, a register
 * write, and the write of a data word into the flash itself.
 *
 * The seam is here, and not around the whole register file, on purpose: it is
 * the smallest change that makes this driver's *success* path - erase a sector,
 * program words, check the flags after each - testable without silicon, and
 * that path is the half that had never run. The data word is part of it and
 * not an afterthought: a controller can only be modelled whole if it sees what
 * is written *into the flash*, which is also what lets the model refuse a word
 * programmed over a word that was never erased. Everything else in this port
 * still reaches its registers directly, and the flag positions and arithmetic
 * are still checked the way they always were.
 *
 * On the target these are three loads and stores, with no indirection the
 * compiler cannot see through.
 */
#ifdef AK_HOST_FLASH
#include "host_flash_model.h"
#define flash_reg_read   host_flash_reg_read
#define flash_reg_write  host_flash_reg_write
#define flash_word_write host_flash_word_write
#else
static uint32_t flash_reg_read(uint32_t offset)
{
    return AK_REG32(FLASH_BASE + offset);
}

static void flash_reg_write(uint32_t offset, uint32_t value)
{
    AK_REG32(FLASH_BASE + offset) = value;
}

/* The data word, and a macro rather than a function on purpose: on the target
 * this has to be the store that was here before the seam existed, byte for
 * byte, or a host-testing change would have quietly rewritten the one piece of
 * this file that only a board can check. */
#define flash_word_write(address, value) ((void)(AK_REG32(address) = (value)))
#endif

static int wait_not_busy(void)
{
    uint32_t guard = AK_FLASH_TIMEOUT;
    while ((flash_reg_read(FLASH_OFF_SR) & FLASH_SR_BSY) != 0 && guard-- > 0) {
    }
    return (flash_reg_read(FLASH_OFF_SR) & FLASH_SR_BSY) == 0 ? 0 : -1;
}

static void clear_flags(void)
{
    /* Writing 1 clears the clearable status bits. */
    flash_reg_write(FLASH_OFF_SR, FLASH_SR_EOP | FLASH_SR_ERRORS);
}

static int unlock(void)
{
    if ((flash_reg_read(FLASH_OFF_CR) & FLASH_CR_LOCK) == 0) {
        return -1; /* already unlocked: somebody else is mid-operation */
    }
    flash_reg_write(FLASH_OFF_KEYR, FLASH_KEY1);
    flash_reg_write(FLASH_OFF_KEYR, FLASH_KEY2);
    return (flash_reg_read(FLASH_OFF_CR) & FLASH_CR_LOCK) == 0 ? 0 : -1;
}

static void lock(void)
{
    flash_reg_write(FLASH_OFF_CR, flash_reg_read(FLASH_OFF_CR) | FLASH_CR_LOCK);
}

int ak_flash_erase_sector(uint8_t sector)
{
    if (sector > 11u) {
        return -1; /* a 1 MB part has sectors 0..11 */
    }
    if (unlock() != 0) {
        return -1;
    }
    if (wait_not_busy() != 0) {
        lock();
        return -1;
    }
    clear_flags();

    flash_reg_write(FLASH_OFF_CR,
                    FLASH_CR_SER | FLASH_CR_SNB(sector) | FLASH_CR_PSIZE_X32);
    flash_reg_write(FLASH_OFF_CR,
                    flash_reg_read(FLASH_OFF_CR) | FLASH_CR_STRT);

    int result = wait_not_busy();
    if (result == 0 && (flash_reg_read(FLASH_OFF_SR) & FLASH_SR_ERRORS) != 0) {
        result = -1;
    }
    clear_flags();
    flash_reg_write(FLASH_OFF_CR, flash_reg_read(FLASH_OFF_CR) &
                                      ~(FLASH_CR_SER | FLASH_CR_SNB(0xFu)));
    lock();
    return result;
}

int ak_flash_program(uint32_t address, const void *source, uint32_t length)
{
    if ((address & 0x3u) != 0 || (length & 0x3u) != 0) {
        return -1; /* word aligned, because the controller is in x32 mode */
    }
    if (unlock() != 0) {
        return -1;
    }
    if (wait_not_busy() != 0) {
        lock();
        return -1;
    }
    clear_flags();

    flash_reg_write(FLASH_OFF_CR, (flash_reg_read(FLASH_OFF_CR) & ~(0x3u << 8)) |
                                      FLASH_CR_PSIZE_X32 | FLASH_CR_PG);

    const uint32_t *words = source;
    uint32_t count = length / 4u;
    int result = 0;

    for (uint32_t i = 0; i < count; i++) {
        if (wait_not_busy() != 0) {
            result = -1;
            break;
        }
        flash_word_write(address + i * 4u, words[i]);
    }
    if (result == 0 && wait_not_busy() != 0) {
        result = -1;
    }
    if (result == 0 && (flash_reg_read(FLASH_OFF_SR) & FLASH_SR_ERRORS) != 0) {
        result = -1;
    }

    flash_reg_write(FLASH_OFF_CR, flash_reg_read(FLASH_OFF_CR) & ~FLASH_CR_PG);
    clear_flags();
    lock();
    return result;
}
