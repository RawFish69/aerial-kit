/*
 * The AT32F435's flash controller, modelled. See host_flash_model_at32.h for
 * what it is for; this file is the behaviour.
 *
 * The shape is the F405 model's, and the part is not: two banks, each with its
 * own unlock, status, control and address registers; pages of 2 KB rather than
 * sectors of 16 to 128; and an address register that the erase path uses and
 * the program path does not. Everything a real controller does asynchronously,
 * this does at once - the wait is the driver's, and what the driver does with
 * it is not what is under test.
 */

#include <string.h>

#include "../src/arch/at32f435/regs.h"
#include "host_flash_model_at32.h"

#define MODEL_FLASH_BASE AK_FLASH_BANK1_START
#define MODEL_FLASH_SIZE 0x00100000u

#define BANK_COUNT 2u

static uint32_t unlock_reg[BANK_COUNT];
static uint32_t sts[BANK_COUNT];
static uint32_t ctrl[BANK_COUNT];
static uint32_t addr[BANK_COUNT];
static uint32_t key_stage[BANK_COUNT];
static uint32_t keys;            /* the last key written, either bank */
static int      unlock_refused;  /* a controller that will not take the keys */
static int      program_error;   /* a controller that refuses to program */
static int      stuck_busy;      /* a controller whose operation never ends */

/* Which bank a register offset belongs to, and which register it is. Only the
 * four the driver uses are modelled; anything else reads as zero and takes
 * writes nowhere, which is what a model should do about registers it does not
 * understand: nothing. */
static int bank_of_offset(uint32_t offset, unsigned *bank, uint32_t *which)
{
    switch (offset) {
    case 0x04u:
    case 0x44u:
        *bank = offset == 0x04u ? 0u : 1u;
        *which = 0u;
        return 1;
    case 0x0Cu:
    case 0x4Cu:
        *bank = offset == 0x0Cu ? 0u : 1u;
        *which = 1u;
        return 1;
    case 0x10u:
    case 0x50u:
        *bank = offset == 0x10u ? 0u : 1u;
        *which = 2u;
        return 1;
    case 0x14u:
    case 0x54u:
        *bank = offset == 0x14u ? 0u : 1u;
        *which = 3u;
        return 1;
    default:
        return 0;
    }
}

static unsigned bank_of_address(uint32_t address)
{
    uint32_t bank = address >= AK_FLASH_BANK2_START ? 1u : 0u;

    return bank;
}

void host_at32_flash_model_reset(void)
{
    for (unsigned b = 0; b < BANK_COUNT; b++) {
        unlock_reg[b] = 0u;
        sts[b] = 0u;
        ctrl[b] = FLASH_CTRL_OPLK; /* both banks start locked */
        addr[b] = 0u;
        key_stage[b] = 0u;
    }
    keys = 0u;
    unlock_refused = 0;
    program_error = 0;
    stuck_busy = 0;
    memset((void *)(uintptr_t)MODEL_FLASH_BASE, 0xFF, MODEL_FLASH_SIZE);
}

void host_at32_flash_model_set_unlock_refused(int refused)
{
    unlock_refused = refused != 0;
}

void host_at32_flash_model_set_program_error(int error)
{
    program_error = error != 0;
}

void host_at32_flash_model_set_stuck_busy(int stuck)
{
    stuck_busy = stuck != 0;
}

uint32_t host_at32_flash_model_keys(void)
{
    return keys;
}

uint32_t host_at32_flash_reg_read(uint32_t offset)
{
    unsigned bank = 0u;
    uint32_t which = 0u;

    if (!bank_of_offset(offset, &bank, &which)) {
        return 0u;
    }
    switch (which) {
    case 1u:
        /* The busy bit, which this part calls OBF, never clearing is the state
         * the driver's guard exists for. */
        return stuck_busy ? (sts[bank] | FLASH_STS_OBF) : sts[bank];
    case 2u:
        return ctrl[bank];
    case 3u:
        return addr[bank];
    default:
        return unlock_reg[bank];
    }
}

static void write_key(unsigned bank, uint32_t value)
{
    unlock_reg[bank] = value;
    keys = value;

    if (unlock_refused) {
        return;
    }
    if (value == FLASH_UNLOCK_KEY1 && key_stage[bank] == 0u) {
        key_stage[bank] = 1u;
        return;
    }
    if (value == FLASH_UNLOCK_KEY2 && key_stage[bank] == 1u) {
        key_stage[bank] = 2u;
        ctrl[bank] &= ~(uint32_t)FLASH_CTRL_OPLK;
        return;
    }
    key_stage[bank] = 0u;
}

/* Writing a one to a clearable flag clears it, which is how the driver clears
 * the done flag and the errors between operations. */
static void write_status(unsigned bank, uint32_t value)
{
    sts[bank] &= ~(value & (FLASH_STS_ODF | FLASH_STS_ERRORS));
}

static void write_control(unsigned bank, uint32_t value)
{
    uint32_t locked = ctrl[bank] & FLASH_CTRL_OPLK;

    ctrl[bank] = locked | (value & ~(FLASH_CTRL_OPLK | FLASH_CTRL_ERSTR));
    if ((value & FLASH_CTRL_OPLK) != 0u) {
        ctrl[bank] |= FLASH_CTRL_OPLK;
        key_stage[bank] = 0u;
    }

    if ((value & FLASH_CTRL_ERSTR) == 0u) {
        return;
    }

    /* The start bit: the part clears it and erases the page the address
     * register points at - a whole page, 2 KB of it, and nothing else. */
    ctrl[bank] &= ~(uint32_t)FLASH_CTRL_ERSTR;
    if (stuck_busy) {
        return; /* started, never finished, and nothing erased */
    }
    if ((ctrl[bank] & FLASH_CTRL_SECERS) != 0u) {
        uint32_t page = addr[bank] & ~(AK_FLASH_PAGE_BYTES - 1u);

        if (page >= MODEL_FLASH_BASE &&
            page + AK_FLASH_PAGE_BYTES <= MODEL_FLASH_BASE + MODEL_FLASH_SIZE) {
            memset((void *)(uintptr_t)page, 0xFF, AK_FLASH_PAGE_BYTES);
            sts[bank] |= FLASH_STS_ODF;
        } else {
            sts[bank] |= FLASH_STS_EPPERR;
        }
    }
}

void host_at32_flash_reg_write(uint32_t offset, uint32_t value)
{
    unsigned bank = 0u;
    uint32_t which = 0u;

    if (!bank_of_offset(offset, &bank, &which)) {
        return;
    }
    switch (which) {
    case 0u:
        write_key(bank, value);
        break;
    case 1u:
        write_status(bank, value);
        break;
    case 2u:
        write_control(bank, value);
        break;
    default:
        addr[bank] = value;
        break;
    }
}

void host_at32_flash_word_write(uint32_t address, uint32_t value)
{
    unsigned bank = bank_of_address(address);

    if ((ctrl[bank] & FLASH_CTRL_FPRGM) == 0u) {
        return; /* not in programming mode: the part ignores the write */
    }
    if (address < MODEL_FLASH_BASE ||
        address + 4u > MODEL_FLASH_BASE + MODEL_FLASH_SIZE) {
        return;
    }

    uint32_t *word = (uint32_t *)(uintptr_t)address;

    if (program_error || *word != 0xFFFFFFFFu) {
        /* A word can only be programmed where the page was erased first, and a
         * model that wrote anyway would turn a save without an erase into a
         * save. */
        sts[bank] |= FLASH_STS_PRGMERR;
        return;
    }
    *word = value;
    sts[bank] |= FLASH_STS_ODF;
}
