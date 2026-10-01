/*
 * The configuration ring, recycled: what a power cut in the erase costs.
 *
 * The board's own comment says the hazard out loud - "that erase ... is the one
 * window a power cut can still lose everything in. It comes once every
 * AK_CONFIG_SLOTS saves instead of on every one" - and then records that a
 * second sector would remove it and that the trade was not taken. This file is
 * that sentence turned into a number, because "once every thirty-two saves" is
 * a frequency and the thing that matters is the consequence.
 *
 * The hazard, in the F405's terms: the part erases 128 KB at once, never one
 * slot, so when the ring of thirty-two slots is full the whole sector goes
 * blank and the new record is programmed into slot zero. Power lost between
 * those two operations leaves a sector that reads as "nobody has ever saved
 * here" - which is also what a brand new board reads as, so nothing downstream
 * can tell the difference. The aircraft boots on defaults. `ak_board_config_read`
 * returns 0, `preflight` prints "none stored" and counts no problem, and the
 * tuning, the airframe and the mixer are gone with nothing but a console line
 * to say so.
 *
 * What this file measures, and what it asserts:
 *
 *   - the ring does not erase on every save (a control: the hazard is rare)
 *   - a recycle without a fault keeps the newest record (a control: the ring
 *     works, so a failure below is the fault and not a broken probe)
 *   - a refused program *outside* the recycle keeps the previous record (the
 *     property the ring already has, and the proof that the load path and the
 *     model's refusal both work)
 *   - **a refused program *inside* the recycle keeps the last good record** -
 *     the one this file exists for, and the one that fails today
 *
 * It is a baseline in the shape of tests/oracle/test_config_policy.c and
 * tests/oracle/test_estimator_oracle.c: it asserts the behaviour B3.3 is
 * supposed to produce, so it fails against the tree as it stands. It lives in
 * tests/oracle/ because `TEST_OBJS` globs every C file in the tests directory
 * into the main binary and a second main() would collide with it, and it is
 * deliberately not in ci.sh until B3.3's repair lands, so that adding it cannot
 * turn a documented defect into a red suite.
 *
 * What is real here and what is a stand-in: the board's own
 * `ak_board_config_write` and `ak_board_config_read`, the real
 * `src/arch/stm32f405/flash.c` driver, and the modelled controller behind it,
 * which erases and programs into the memory this file maps - so the bytes read
 * back are the bytes the controller would really have written. There is no
 * stand-in for the part's behaviour; there is no part. `program_refused` is the
 * one thing here no silicon does on demand, and what it models is real: the
 * erase completed and the program did not, which is a supply that sagged, a
 * worn cell, or somebody pulling the cable.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <sys/mman.h>

#include "ak_board.h"
#include "host_flash_model.h"

/* Where the board puts the saved configuration: the last 128 KB sector of a
 * 1 MB part. Private to the board file, so it is written here the way
 * tests/test_arch.c writes it - a board that moved its record has to change
 * this line too, loudly, rather than silently reading a sector nobody writes. */
#define CONFIG_BASE 0x080E0000u
#define FLASH_BASE  0x08000000u
#define FLASH_SIZE  0x00100000u

/* The board's AK_CONFIG_SLOTS: one 4 KB slot per record, thirty-two of them in
 * the 128 KB sector. Written out rather than derived because it is the number
 * the hazard's *frequency* is stated in, and a probe that computed it from a
 * constant the board also computes would still pass if both changed. */
#define RING_SLOTS 32u

/* --- the measurements ---------------------------------------------------- */

static int failures;

static void expect(const char *name, int passed)
{
    printf("  %-6s %s\n", passed ? "ok" : "FAILED", name);
    if (!passed) {
        failures++;
    }
}

/* --- the part ------------------------------------------------------------ */

static int map_flash(void)
{
    void *flash = mmap((void *)FLASH_BASE, FLASH_SIZE, PROT_READ | PROT_WRITE,
                       MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);
    return flash != MAP_FAILED;
}

/* A board nobody has saved to: the controller at power-on and every byte of the
 * flash 0xFF. There is no state one measurement can leave for the next. */
static void fresh_part(void)
{
    host_flash_model_reset();
}

static int save(const char *text)
{
    return ak_board_config_write(text, (uint32_t)strlen(text));
}

/* What the board would load, as the number of bytes, or a negative error. The
 * text is left in `out` for the cases that care which record it was. */
static int load(char *out, unsigned len)
{
    int got = ak_board_config_read(out, len);
    if (got >= 0) {
        out[got] = '\0';
    } else {
        out[0] = '\0';
    }
    return got;
}

/* Fill the ring without recycling: RING_SLOTS saves, slots zero to thirty-one.
 * Leaves the newest record reading "n=<RING_SLOTS-1>". */
static void fill_the_ring(char *report, unsigned len)
{
    for (unsigned i = 0; i < RING_SLOTS; i++) {
        (void)snprintf(report, len, "n=%u\n", i);
        if (save(report) != 0) {
            printf("      (save %u refused)\n", i);
        }
    }
}

/* --- the probe ----------------------------------------------------------- */

int main(void)
{
    char text[64];
    char loaded[128];

    printf("B3.3 baseline: the configuration ring, recycled\n");

    if (!map_flash()) {
        printf("  FAILED could not map the flash region - nothing measured\n");
        return 1;
    }

    printf("\nthe ring      %u slots of 4 KB in one 128 KB sector\n", RING_SLOTS);
    printf("the part      128 KB is the erase unit: one slot cannot be erased "
           "alone\n");

    /*
     * 1. The frequency, and the control under everything else.
     *
     *    Thirty-two saves fill the ring and erase nothing; the thirty-third
     *    recycles. If the ring erased on every save this hazard would be
     *    constant rather than rare, and the count is how that is known rather
     *    than believed.
     */
    fresh_part();
    fill_the_ring(text, sizeof text);
    uint32_t erases_at_full = host_flash_model_erases();
    printf("\n%u saves fill the ring\n", RING_SLOTS);
    printf("  sector erases    %u\n", erases_at_full);
    expect("thirty-two saves erase nothing", erases_at_full == 0u);

    int got = save("n=32\n");
    uint32_t erases_at_recycle = host_flash_model_erases();
    printf("\nthe thirty-third save recycles\n");
    printf("  save returned    %d\n", got);
    printf("  sector erases    %u, was %u\n", erases_at_recycle,
           erases_at_full);
    (void)load(loaded, sizeof loaded);
    printf("  loads            \"%s\"\n", loaded);
    expect("a recycle without a fault keeps the newest record",
           got == 0 && strcmp(loaded, "n=32\n") == 0);
    expect("and it costs exactly one erase", erases_at_recycle == 1u);

    /*
     * 2. The control the measurement below is worthless without.
     *
     *    A refused program *outside* the recycle: one record in the ring, the
     *    next slot blank, nothing to erase. The save fails and the record it
     *    was replacing still loads. That is the property the ring was built
     *    for and it already holds - which is what makes the next measurement a
     *    statement about the recycle rather than about refused programs.
     */
    fresh_part();
    expect("the first save writes", save("first\n") == 0);
    host_flash_model_set_program_refused(1);
    int refused = save("second\n");
    host_flash_model_set_program_refused(0);
    (void)load(loaded, sizeof loaded);
    printf("\ncontrol: a refused program outside the recycle\n");
    printf("  save returned    %d\n", refused);
    printf("  loads            \"%s\"\n", loaded);
    expect("a refused program is reported rather than assumed",
           refused < 0);
    expect("and outside the recycle the record it was replacing survives",
           strcmp(loaded, "first\n") == 0);

    /*
     * 3. The measurement. The same refusal, one save later in the ring's life:
     *    the save that recycles.
     *
     *    The erase happens, the program does not, and the sector is left as
     *    blank as the day the board was made. Every record the ring holds was
     *    inside the erase - that is what "the F405 erases 128 KB at once"
     *    means - so there is nothing to fall back to and nothing left to say
     *    that anything was ever here.
     */
    fresh_part();
    fill_the_ring(text, sizeof text);
    host_flash_model_set_program_refused(1);
    int lost = save("n=32\n");
    host_flash_model_set_program_refused(0);
    int after = load(loaded, sizeof loaded);
    printf("\nthe recycle, with the power cut in it\n");
    printf("  save returned    %d\n", lost);
    printf("  loads            %d byte(s), \"%s\"\n", after, loaded);
    printf("  sector word 0    0x%08X%s\n",
           *(const volatile uint32_t *)(uintptr_t)CONFIG_BASE,
           *(const volatile uint32_t *)(uintptr_t)CONFIG_BASE == 0xFFFFFFFFu
               ? " (erased)" : "");
    expect("a power cut in the recycle window keeps the last good record",
           after > 0 && strcmp(loaded, "n=31\n") == 0);
    expect("and the save that lost it still reported the failure",
           lost < 0);

    /*
     * 4. Why zero was the whole of the hazard, as a number rather than an
     *    adjective.
     *
     *    Zero is not an error. It is the answer a brand new part gives, and
     *    `preflight` prints it as "none stored" and counts no problem - so a
     *    board that lost its configuration and a board that never had one are
     *    indistinguishable from the inside. That is what made the window worth
     *    a second sector: not that a save could fail, but that failing this way
     *    left nothing behind to notice.
     */
    fresh_part();
    int never_saved = load(loaded, sizeof loaded);
    printf("\nfor comparison, a part nobody has saved to\n");
    printf("  loads            %d byte(s)\n", never_saved);
    printf("  and the recycle  %d byte(s)\n", after);
    expect("a new part reads as nothing stored, which is not an error",
           never_saved == 0);
    expect("and the recycle no longer reads the same way",
           after != never_saved && after > 0);

    printf("\n%d measurement(s) failed\n", failures);
    printf("%s\n", failures == 0
           ? "the ring survives a power cut in its recycle"
           : "this is the baseline, not a regression: B3.3 is not written yet");
    return failures == 0 ? 0 : 1;
}
