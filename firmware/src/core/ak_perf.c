#include "ak_perf.h"
#include "ak_time.h"
#include "ak_flight.h" /* AK_FLIGHT_LOOP_MS: the period the loop is trying to hold */

/*
 * The profiling window's state. All of it is written from the fast path, so
 * every field is a plain integer and nothing here allocates, locks or divides.
 *
 * The only division is in ak_perf_snapshot(), which the console and the
 * protocol call - not the loop.
 *
 * Every field is written by the loop and read by whatever asked for a report,
 * with nothing to serialise them. That is deliberate and it is why the report
 * is allowed to be a snapshot rather than a transaction: a torn read costs a
 * person one odd-looking line in a diagnostic they are about to take again,
 * and a lock here would cost the loop time on every iteration.
 */

/*
 * Longer than a tenth of the nominal period past it counts as late, in
 * microseconds: tight enough to see a link costing the loop something, loose
 * enough not to count the tick's own quantisation.
 *
 * It was a flat hundred microseconds, which is a tenth of a *one-kilohertz*
 * period and was written when that was the only period there was. Phase 1.4
 * made the period a number the aircraft states, and a flat hundred is two
 * fifths of a four-kilohertz period - a threshold that means something
 * different at every rate, and in the direction that under-reports. Same rule
 * and same reasoning as `ak_sched_slack_us`, deliberately not shared with it:
 * the two are different clocks measuring different things, and one of them
 * changing should not silently move the other.
 *
 * The floor is for the division rather than for the loop: a nominal period
 * below fifty microseconds would otherwise be given a slack smaller than the
 * granularity of the comparison.
 */
#define AK_PERF_LATE_SLACK_MIN_US 5u

static struct {
    int      started;
    uint32_t nominal_us;

    /* The open period, and the section open inside it. */
    uint32_t loop_start;
    int      loop_open;

    /* Which section is open, and when it began. AK_PERF_NONE between sections,
     * which is a state with time in it and no owner. */
    ak_perf_section_t open_section;
    uint32_t section_start;

    /* Closed periods, and how many of those the port could actually measure.
     *
     * Two counts rather than one, and the difference is not academic. A port
     * whose clock says zero can still say a loop closed - that needs no clock -
     * but it cannot say how long it took, and *every* figure derived from time
     * has to be divided by the periods that have one. Dividing a histogram that
     * was never fed by a loop count that was is how a profiler that cannot
     * measure reports a p99 of 255 microseconds: the percentile walk runs off
     * the end of an empty histogram and returns the last bin. That is a
     * fabricated number, which is the one thing this file is written not to
     * produce. */
    uint32_t loops;
    uint32_t samples;
    uint32_t late;
    uint32_t period_last_us;
    uint32_t period_min_us;
    uint32_t period_max_us;

    /* |period - nominal| in microseconds, one bin each, saturating nowhere:
     * a 32-bit bin does not fill in any session this firmware will fly. */
    uint32_t jitter[AK_PERF_JITTER_BINS];
    uint32_t jitter_over;
    uint16_t jitter_max_us;

    /* Sections: cycles accumulated, and the longest single run seen. */
    uint64_t section_cycles[AK_PERF_SECTIONS];
    uint32_t section_max_cycles[AK_PERF_SECTIONS];
} P;

/* One vocabulary, and these are its words.
 *
 * The names are printed by the console and are also the ones a client labels the
 * wire's five section arrays with (`PERF_SECTIONS` in tools/akproto.py, and the
 * list in docs/16-protocol.md). It said "imu read" here for a while, which is a
 * better sentence and was a second name for one section: the console said one
 * thing, every client said another, and nothing compared them. `make proto-test`
 * reads the console's report and compares these to the wire's list now, so a
 * rename on either side is a failing check rather than two names quietly
 * drifting apart - which is why the prose that used to live here is in
 * docs/29-timing.md's table instead, where it belongs and where it does not
 * have to be a name. */
const char *ak_perf_section_name(ak_perf_section_t section)
{
    switch (section) {
    case AK_PERF_NONE:      return "other";
    case AK_PERF_IMU:       return "imu";
    case AK_PERF_ESTIMATOR: return "estimator";
    case AK_PERF_PID:       return "pid";
    case AK_PERF_MIXER:     return "mixer";
    case AK_PERF_OUTPUT:    return "output";
    default:                return "?";
    }
}

static void clear_window(void)
{
    uint32_t i;

    P.loops = 0u;
    P.samples = 0u;
    P.late = 0u;
    P.period_last_us = 0u;
    P.period_min_us = 0u;
    P.period_max_us = 0u;
    P.jitter_over = 0u;
    P.jitter_max_us = 0u;
    for (i = 0u; i < AK_PERF_JITTER_BINS; i++) {
        P.jitter[i] = 0u;
    }
    for (i = 0u; i < (uint32_t)AK_PERF_SECTIONS; i++) {
        P.section_cycles[i] = 0ull;
        P.section_max_cycles[i] = 0u;
    }
}

/* Charge the open section, if there is one, with `now - section_start`. Called
 * from every boundary, which is what makes a section impossible to leave open. */
static void close_section(uint32_t now)
{
    uint32_t cycles;

    if (P.open_section == AK_PERF_NONE) {
        return;
    }
    /* Unsigned subtraction: correct across the counter's 25.6 s wrap at
     * 168 MHz, and the only form that is. */
    cycles = (uint32_t)(now - P.section_start);
    P.section_cycles[P.open_section] += (uint64_t)cycles;
    if (cycles > P.section_max_cycles[P.open_section]) {
        P.section_max_cycles[P.open_section] = cycles;
    }
    P.open_section = AK_PERF_NONE;
}

void ak_perf_init(void)
{
    P.started = 1;
    P.loop_open = 0;
    P.loop_start = 0u;
    P.open_section = AK_PERF_NONE;
    P.section_start = 0u;
    /* One millisecond, from the core's own constant rather than a copy of it:
     * the number the jitter is measured against is the number the loop is
     * actually told to run at. */
    P.nominal_us = (uint32_t)AK_FLIGHT_LOOP_MS * 1000u;
    clear_window();
    ak_cycles_init();
}

void ak_perf_set_nominal_us(uint32_t nominal_us)
{
    if (nominal_us == 0u) {
        return;
    }
    P.nominal_us = nominal_us;
}

void ak_perf_loop_begin(uint32_t start_cycles)
{
    if (!P.started) {
        return;
    }
    /* A period cannot begin inside a section: whatever was open belonged to the
     * loop that just ended, and ak_perf_loop_end() would have closed it - this
     * is the belt to that pair of braces, because a section left open across a
     * period boundary would be charged to a loop it did not run in. */
    P.open_section = AK_PERF_NONE;
    P.loop_start = start_cycles;
    P.loop_open = 1;
}

void ak_perf_phase(ak_perf_section_t phase)
{
    uint32_t now;

    /* Outside a period there is nothing to attribute the time to. A phase mark
     * here is not an error - a caller may be driving the flight step without
     * the loop's bracket around it, which is what most host tests do - so it is
     * dropped rather than recorded against a loop that does not exist. */
    if (!P.started || !P.loop_open) {
        return;
    }
    now = ak_cycles();
    close_section(now);
    P.open_section = (phase < AK_PERF_SECTIONS) ? phase : AK_PERF_NONE;
    P.section_start = now;
}

void ak_perf_loop_end(void)
{
    uint32_t per_us;
    uint32_t now;
    uint32_t period_us;
    uint32_t jitter;

    if (!P.started || !P.loop_open) {
        return;
    }
    P.loop_open = 0;

    now = ak_cycles();
    close_section(now);

    per_us = ak_cycles_per_us();
    if (per_us == 0u) {
        /* A port that cannot say how fast it is running cannot be turned into
         * microseconds, and a guess here would be a fabricated number. Count
         * the loop and leave the durations unmeasured. */
        P.loops++;
        return;
    }

    /* Unsigned subtraction, for the same reason close_section uses it. */
    period_us = (uint32_t)(now - P.loop_start) / per_us;

    P.loops++;
    P.samples++;
    P.period_last_us = period_us;
    if (P.samples == 1u || period_us < P.period_min_us) {
        P.period_min_us = period_us;
    }
    if (period_us > P.period_max_us) {
        P.period_max_us = period_us;
    }

    /* A tenth of the period this loop is trying to hold - see the constant's
     * comment for why it is not flat. Read here rather than cached at init
     * because ak_perf_set_nominal_us() can change the period under a running
     * profiler. */
    if (period_us > P.nominal_us +
                        (P.nominal_us / 10u > AK_PERF_LATE_SLACK_MIN_US
                             ? P.nominal_us / 10u
                             : AK_PERF_LATE_SLACK_MIN_US)) {
        P.late++;
    }

    /* Jitter is the distance from the nominal period, either side of it. A
     * loop that came back early is as interesting as one that came back late,
     * and reporting only the late half would hide a clock running fast. */
    jitter = (period_us > P.nominal_us) ? (period_us - P.nominal_us)
                                        : (P.nominal_us - period_us);
    if (jitter >= AK_PERF_JITTER_BINS) {
        P.jitter_over++;
        jitter = AK_PERF_JITTER_BINS - 1u;
    }
    P.jitter[jitter]++;
    if (jitter > P.jitter_max_us) {
        P.jitter_max_us = (uint16_t)jitter;
    }
}

int ak_perf_reset(void)
{
    if (!P.started || P.loops == 0u) {
        return 0;
    }
    clear_window();
    return 1;
}

/* The bin a percentile lands in, from the cumulative histogram. Returns the bin
 * index, so the answer is exact to the microsecond the bins are wide.
 *
 * `total` is the number of *samples in the histogram*, never the number of
 * loops that closed. With no samples it returns 0 rather than the last bin: an
 * empty histogram walked to its end is how a port that cannot measure comes to
 * claim a p99 of 255 microseconds. See the note on P.samples. */
static uint16_t percentile(const uint32_t *bins, uint32_t total, uint32_t permille)
{
    uint32_t want;
    uint32_t seen = 0u;
    uint32_t i;

    if (total == 0u) {
        return 0u;
    }
    want = (uint32_t)(((uint64_t)total * permille) / 1000u);
    if (want == 0u) {
        want = 1u; /* p50 of one sample is that sample, not bin 0 */
    }
    for (i = 0u; i < AK_PERF_JITTER_BINS; i++) {
        seen += bins[i];
        if (seen >= want) {
            return (uint16_t)i;
        }
    }
    return (uint16_t)(AK_PERF_JITTER_BINS - 1u);
}

void ak_perf_snapshot(ak_perf_snapshot_t *out)
{
    uint32_t per_us;
    uint32_t i;
    uint64_t busy_cycles = 0ull;

    if (out == 0) {
        return;
    }

    out->started = P.started;
    out->loops = P.loops;
    out->samples = P.samples;
    out->nominal_us = P.nominal_us;
    out->period_last_us = P.period_last_us;
    out->period_min_us = P.period_min_us;
    out->period_max_us = P.period_max_us;
    out->late = P.late;
    out->jitter_over = P.jitter_over;
    out->jitter_max_us = P.jitter_max_us;
    out->jitter_p50_us = percentile(P.jitter, P.samples, 500u);
    out->jitter_p99_us = percentile(P.jitter, P.samples, 990u);

    per_us = ak_cycles_per_us();

    for (i = 0u; i < (uint32_t)AK_PERF_SECTIONS; i++) {
        uint64_t avg_cycles;
        uint32_t ns;

        out->section_max_us[i] = (per_us == 0u)
            ? 0u
            : (P.section_max_cycles[i] / per_us);
        busy_cycles += P.section_cycles[i];

        if (per_us == 0u || P.samples == 0u) {
            out->section_avg_ns[i] = 0u;
            continue;
        }
        /* Per *loop*, not per time the section ran: a section that runs on one
         * loop in ten costs a tenth of this per loop, which is what a budget
         * is spent in. */
        avg_cycles = P.section_cycles[i] / P.samples;
        ns = (uint32_t)((avg_cycles * 1000ull) / per_us);
        out->section_avg_ns[i] = ns;
    }

    /* The instrumented sections against the nominal slot, in per-mille. A
     * lower bound: the console, the links and the navigation are not stamped.
     * AK_PERF_NONE accumulates nothing, so the un-owned time is excluded by
     * construction rather than by a caller remembering to exclude it. */
    if (per_us == 0u || P.samples == 0u || P.nominal_us == 0u) {
        out->load_permille = 0u;
        return;
    }
    {
        uint64_t busy_per_loop = busy_cycles / P.samples;
        uint64_t slot_cycles = (uint64_t)per_us * P.nominal_us;
        out->load_permille = (uint32_t)((busy_per_loop * 1000ull) / slot_cycles);
    }
}
