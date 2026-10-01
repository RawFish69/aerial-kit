#include "arch.h"

/*
 * The internal flash controller, which on this part is two controllers: a 1 MB
 * part is two banks of 512 KB, and each bank has its own unlock, status,
 * control and address registers. Which one a driver is talking to is decided by
 * the *address* it was given, so the bank is chosen here rather than by a caller
 * that could get it wrong - and getting it wrong is not a small mistake: it is
 * an erase aimed at the wrong controller.
 *
 * The operations are page erase and word program, and a page is **2 KB** where
 * the F405's sectors are 16 to 128 KB, so the arithmetic around the record and
 * any log is recomputed rather than rescaled.
 *
 * Neither operation can be tested without hardware, so both are written to fail
 * loudly rather than quietly: every error flag is checked after every step, the
 * busy wait is bounded, the address is checked to be page aligned (for an
 * erase) or word aligned (for a program) before anything is unlocked, and the
 * controller is locked again on every path out. The success path - a page that
 * erases, a word that lands where it was addressed - runs against a modelled
 * controller on a host, the same seam the F405 port has
 * (tests/host_flash_model_at32.c), because that is the half that had never run
 * anywhere when this project started.
 */

#define AK_FLASH_TIMEOUT 1000000u

/* The three places every access goes through, so a host build can intercept
 * them; on the target these are loads and stores with no indirection. */
#ifdef AK_HOST_FLASH_AT32
#include "host_flash_model_at32.h"
#define flash_reg_read   host_at32_flash_reg_read
#define flash_reg_write  host_at32_flash_reg_write
#define flash_word_write host_at32_flash_word_write
#else
static uint32_t flash_reg_read(uint32_t offset)
{
    return AK_REG32(FLASH_REG_BASE + offset);
}

static void flash_reg_write(uint32_t offset, uint32_t value)
{
    AK_REG32(FLASH_REG_BASE + offset) = value;
}

#define flash_word_write(address, value) ((void)(AK_REG32(address) = (value)))
#endif

/* The register offsets, spelled once and used with the helpers above - so the
 * host model sees offsets and the target sees addresses in the same file. */
#define FLASH_OFF_UNLOCK  0x04u
#define FLASH_OFF_STS     0x0Cu
#define FLASH_OFF_CTRL    0x10u
#define FLASH_OFF_ADDR    0x14u
#define FLASH2_OFF_UNLOCK 0x44u
#define FLASH2_OFF_STS    0x4Cu
#define FLASH2_OFF_CTRL   0x50u
#define FLASH2_OFF_ADDR   0x54u

typedef struct {
    uint32_t unlock;
    uint32_t sts;
    uint32_t ctrl;
    uint32_t addr;
} ak_flash_bank_t;

/* Which bank an address belongs to, and the registers that go with it. An
 * address outside both is refused rather than defaulted, because the only thing
 * a default could do here is erase something nobody asked for. */
static int bank_for(uint32_t address, ak_flash_bank_t *bank)
{
    if (address >= AK_FLASH_BANK1_START && address <= AK_FLASH_BANK1_END) {
        bank->unlock = FLASH_OFF_UNLOCK;
        bank->sts    = FLASH_OFF_STS;
        bank->ctrl   = FLASH_OFF_CTRL;
        bank->addr   = FLASH_OFF_ADDR;
        return 1;
    }
    if (address >= AK_FLASH_BANK2_START && address <= AK_FLASH_BANK2_END) {
        bank->unlock = FLASH2_OFF_UNLOCK;
        bank->sts    = FLASH2_OFF_STS;
        bank->ctrl   = FLASH2_OFF_CTRL;
        bank->addr   = FLASH2_OFF_ADDR;
        return 1;
    }
    return 0;
}

static void unlock(const ak_flash_bank_t *bank)
{
    flash_reg_write(bank->unlock, FLASH_UNLOCK_KEY1);
    flash_reg_write(bank->unlock, FLASH_UNLOCK_KEY2);
}

static void lock(const ak_flash_bank_t *bank)
{
    flash_reg_write(bank->ctrl,
                    flash_reg_read(bank->ctrl) | FLASH_CTRL_OPLK);
}

static void clear_flags(const ak_flash_bank_t *bank)
{
    /* The flags clear by writing a one back, which is also why they are read
     * before they are cleared everywhere in this file. */
    flash_reg_write(bank->sts,
                    flash_reg_read(bank->sts) & FLASH_STS_ERRORS);
    flash_reg_write(bank->sts, FLASH_STS_ODF);
}

static int wait_not_busy(const ak_flash_bank_t *bank)
{
    uint32_t guard = AK_FLASH_TIMEOUT;

    while ((flash_reg_read(bank->sts) & FLASH_STS_OBF) != 0u) {
        if (guard-- == 0u) {
            return -1;
        }
    }
    return 0;
}

/* The lock bit is the controller's own answer to "am I locked", so this is a
 * read rather than a guess - and it is how the driver notices a controller that
 * refused the key sequence, which is a board whose option bytes are not what
 * anybody thinks they are. */
static int is_locked(const ak_flash_bank_t *bank)
{
    return (flash_reg_read(bank->ctrl) & FLASH_CTRL_OPLK) != 0u;
}

int ak_flash_erase_page(uint32_t address)
{
    if ((address & (AK_FLASH_PAGE_BYTES - 1u)) != 0u) {
        return -1; /* a page erase erases whole pages, aligned to their start */
    }

    ak_flash_bank_t bank;

    if (!bank_for(address, &bank)) {
        return -1;
    }
    unlock(&bank);
    if (is_locked(&bank)) {
        return -1;
    }
    if (wait_not_busy(&bank) != 0) {
        lock(&bank);
        return -1;
    }
    clear_flags(&bank);

    flash_reg_write(bank.ctrl,
                    flash_reg_read(bank.ctrl) | FLASH_CTRL_SECERS);
    flash_reg_write(bank.addr, address);
    flash_reg_write(bank.ctrl,
                    flash_reg_read(bank.ctrl) | FLASH_CTRL_ERSTR);

    int result = wait_not_busy(&bank);
    if (result == 0 &&
        (flash_reg_read(bank.sts) & FLASH_STS_ERRORS) != 0) {
        result = -1;
    }
    clear_flags(&bank);
    flash_reg_write(bank.ctrl,
                    flash_reg_read(bank.ctrl) & ~FLASH_CTRL_SECERS);
    lock(&bank);
    return result;
}

int ak_flash_program(uint32_t address, const void *source, uint32_t length)
{
    if ((address & 0x3u) != 0u || (length & 0x3u) != 0u) {
        return -1; /* word aligned: the controller programs 32 bits at a time */
    }
    if (length == 0u) {
        return 0;
    }

    ak_flash_bank_t bank;
    ak_flash_bank_t last;

    /*
     * The whole write has to be in one bank, and that is a comparison of the
     * two banks rather than two calls that each have to succeed: both ends
     * being in *a* bank is not the same as both being in the *same* one, and a
     * write that straddles the boundary is a write that would have to unlock
     * and program two controllers to do what it was asked - so it is refused.
     */
    if (!bank_for(address, &bank) ||
        !bank_for(address + length - 1u, &last) ||
        bank.unlock != last.unlock) {
        return -1;
    }
    unlock(&bank);
    if (is_locked(&bank)) {
        return -1;
    }
    if (wait_not_busy(&bank) != 0) {
        lock(&bank);
        return -1;
    }
    clear_flags(&bank);

    /*
     * The program bit goes on once and the words follow it, which is how
     * Artery's own driver does it: the controller latches each word as it is
     * written and raises the done flag for each. The busy wait between words is
     * what keeps a slow controller from losing one.
     */
    flash_reg_write(bank.ctrl,
                    flash_reg_read(bank.ctrl) | FLASH_CTRL_FPRGM);

    const uint32_t *words = (const uint32_t *)source;
    uint32_t count = length / 4u;
    int result = 0;

    for (uint32_t i = 0; i < count; i++) {
        if (wait_not_busy(&bank) != 0) {
            result = -1;
            break;
        }
        /* No address register here: this part's controller takes the address
         * from the bus write itself, which is why Artery's own driver writes
         * `fprgm` and then the word. The address register belongs to the erase
         * path, and writing it here would be a second source of truth about
         * where the word is going. */
        flash_word_write(address + i * 4u, words[i]);
    }
    if (result == 0 && wait_not_busy(&bank) != 0) {
        result = -1;
    }
    if (result == 0 &&
        (flash_reg_read(bank.sts) & FLASH_STS_ERRORS) != 0) {
        result = -1;
    }

    flash_reg_write(bank.ctrl,
                    flash_reg_read(bank.ctrl) & ~FLASH_CTRL_FPRGM);
    clear_flags(&bank);
    lock(&bank);
    return result;
}
