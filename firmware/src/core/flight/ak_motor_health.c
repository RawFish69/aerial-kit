#include "ak_motor_health.h"

#include <string.h>

/*
 * The twelve decisions are in the header. This file is their arithmetic, and
 * the only things here that are not a direct transcription of one of them are
 * marked.
 */

/* --- the window -------------------------------------------------------- */

static void window_clear(ak_motor_health_window_t *w)
{
    memset(w, 0, sizeof(*w));
}

/* Move the window up to the bucket `now_ms` falls in, clearing every bucket
 * between (decision 6).
 *
 * The subtraction is deliberately an unsigned one. A clock that wrapped, or a
 * gap longer than the window, gives a difference at or above the bucket count
 * and the whole window goes - which is the right answer for both, because in
 * neither case is what is in the buckets the present. */
static void window_advance(ak_motor_health_window_t *w, uint32_t now_ms)
{
    uint32_t bucket = now_ms / AK_MOTOR_HEALTH_BUCKET_MS;

    if (w->bucket == bucket) {
        return;
    }

    if (bucket - w->bucket >= AK_MOTOR_HEALTH_BUCKETS) {
        window_clear(w);
        w->bucket = bucket;
        return;
    }

    while (w->bucket != bucket) {
        unsigned i;

        w->bucket++;
        i = (unsigned)(w->bucket % AK_MOTOR_HEALTH_BUCKETS);

        w->answer_sum -= w->answers[i];
        w->stopped_sum -= w->stopped[i];
        w->off_model_sum -= w->off_model[i];
        w->answers[i] = 0u;
        w->stopped[i] = 0u;
        w->off_model[i] = 0u;
    }
}

/* One reply that carried a speed. `stopped` and `off_model` are subsets of
 * `answers`, which is why the three are counted separately rather than as one
 * total with a bad count beside it: decision 9 has to know which kind of bad. */
static void window_add(ak_motor_health_window_t *w, int stopped, int off_model)
{
    unsigned i = (unsigned)(w->bucket % AK_MOTOR_HEALTH_BUCKETS);

    if (w->answers[i] < 0xFFFFu) {
        w->answers[i]++;
        w->answer_sum++;
    }
    if (stopped && w->stopped[i] < 0xFFFFu) {
        w->stopped[i]++;
        w->stopped_sum++;
    }
    if (off_model && w->off_model[i] < 0xFFFFu) {
        w->off_model[i]++;
        w->off_model_sum++;
    }
}

static uint32_t per_10k(uint32_t part, uint32_t whole)
{
    return whole == 0u ? 0u : (uint32_t)(((uint64_t)part * 10000u) / whole);
}

/* The window's verdict, given the flag the motor already carried (decision 8).
 *
 * The hysteresis is one comparison: the threshold is the setting one, or the
 * clearing one when the motor is already flagged. Nothing is latched, so a
 * window that genuinely empties of bad samples loses the flag by itself, and a
 * motor that is right again is reported right without anyone clearing it. */
static ak_motor_health_flag_t window_verdict(const ak_motor_health_window_t *w,
                                             ak_motor_health_flag_t prev)
{
    uint32_t answers = w->answer_sum;
    uint32_t threshold;

    /* Decision 5: an absence, not a threshold. A window in which not one reply
     * carried a speed is the one answer this module gives about the wire rather
     * than about the motor, and it is not a fraction of anything. */
    if (answers == 0u) {
        return AK_MOTOR_HEALTH_NO_TELEMETRY;
    }

    /* Too thin to be a verdict (decision 6). Reported as OK rather than as
     * NO_TELEMETRY, because the two are different: nothing answered is a fact
     * about the wire, and "eight replies have arrived in the last two
     * milliseconds" is a fact about how recently the motor was asked. */
    if (answers < AK_MOTOR_HEALTH_MIN_SAMPLES) {
        return AK_MOTOR_HEALTH_OK;
    }

    threshold = (prev == AK_MOTOR_HEALTH_STALLED || prev == AK_MOTOR_HEALTH_OFF_MODEL)
                    ? AK_MOTOR_HEALTH_CLEAR_PER_10K
                    : AK_MOTOR_HEALTH_BAD_PER_10K;

    /* Decision 9: the worst kind over the threshold, so that a window holding
     * both answers the same whichever sample landed last. */
    if (per_10k(w->stopped_sum, answers) >= threshold) {
        return AK_MOTOR_HEALTH_STALLED;
    }
    if (per_10k(w->off_model_sum, answers) >= threshold) {
        return AK_MOTOR_HEALTH_OFF_MODEL;
    }
    return AK_MOTOR_HEALTH_OK;
}

/* --- the model --------------------------------------------------------- */

/* Advance one motor's learned gain with one sample (decision 2).
 *
 * The first sample seeds the filter instead of being low-passed into a zero:
 * a PT1 primed at zero takes its whole time constant to climb to the value it
 * is being fed, and for the first second and a half of a flight that would make
 * the model systematically low - which the two-sided test of decision 3 would
 * read as every motor racing. */
static void gain_feed(ak_motor_health_motor_t *m, float ratio, uint32_t dt_ms)
{
    float dt = (float)dt_ms / 1000.0f;

    if (dt <= 0.0f) {
        return;
    }

    if (!m->gain_fed) {
        ak_filter_pt1_init(&m->gain_filter, AK_MOTOR_HEALTH_GAIN_CUTOFF_HZ, dt);
        ak_filter_pt1_prime(&m->gain_filter, ratio);
        m->gain = ratio;
        m->gain_dt = dt;
        m->gain_fed = 1u;
    } else {
        /* The coefficients are a function of the interval, and on a loop that
         * is not jittering the interval does not change - so this is one
         * comparison in the common case and a cosine only when the loop's rate
         * really has moved. */
        if (dt != m->gain_dt) {
            ak_filter_pt1_set(&m->gain_filter, AK_MOTOR_HEALTH_GAIN_CUTOFF_HZ, dt);
            m->gain_dt = dt;
        }
        m->gain = ak_filter_pt1_apply(&m->gain_filter, ratio);
    }

    if (m->gain_ms < AK_MOTOR_HEALTH_GAIN_MS) {
        m->gain_ms += dt_ms;
        if (m->gain_ms >= AK_MOTOR_HEALTH_GAIN_MS) {
            m->gain_valid = 1u;
        }
    }
}

/* --- the module -------------------------------------------------------- */

void ak_motor_health_init(ak_motor_health_t *health, unsigned motors)
{
    unsigned i;

    if (!health) {
        return;
    }

    memset(health, 0, sizeof(*health));
    health->motors = motors < AK_MAX_MOTORS ? motors : AK_MAX_MOTORS;

    /* Zeroed state carries flag 0, which is OK - and OK is a claim this module
     * has not earned for a motor nobody has asked about yet. */
    for (i = 0; i < AK_MAX_MOTORS; i++) {
        health->motor[i].flag = AK_MOTOR_HEALTH_IDLE;
    }
}

ak_motor_health_flag_t ak_motor_health_update(ak_motor_health_t *health,
                                              unsigned motor, float command,
                                              const ak_motor_health_reply_t *reply,
                                              uint32_t now_ms)
{
    ak_motor_health_motor_t *m;
    int in_band;
    int have_speed = 0;
    int stopped = 0;
    int off_model = 0;
    float erpm = 0.0f;
    uint32_t dt_ms;
    uint32_t bucket;

    if (!health || motor >= health->motors) {
        return AK_MOTOR_HEALTH_IDLE;
    }

    m = &health->motor[motor];

    /* `command >= FLOOR` is written this way round on purpose: a NaN command
     * compares false and lands on IDLE, which is the safe reading of a number
     * that is not a number. */
    in_band = command >= AK_MOTOR_HEALTH_COMMAND_FLOOR ? 1 : 0;

    /* Decision 7: being asked after being idle clears the window, so the motor
     * is not judged on the idleness that preceded the command. */
    if (in_band && !m->in_band) {
        window_clear(&m->window);
        bucket = now_ms / AK_MOTOR_HEALTH_BUCKET_MS;
        m->window.bucket = bucket;
    }
    m->in_band = (uint8_t)in_band;

    if (!in_band) {
        m->flag = AK_MOTOR_HEALTH_IDLE;
        m->last_ms = now_ms;
        return m->flag;
    }

    /* The window moves whatever the reply says, so a motor that has stopped
     * answering empties its own window rather than freezing a good one. */
    window_advance(&m->window, now_ms);

    /* The interval from the timestamps, not from an assumption about the loop
     * (decision 2). A repeat inside the same millisecond is the honest zero a
     * millisecond clock can give, and gain_feed declines it.
     *
     * The first call for a motor has no previous timestamp to subtract, and the
     * interval it would produce is not an interval at all - it is however long
     * the aircraft has been powered. That number as a filter interval is a
     * filter that does not filter, so the first call establishes the clock and
     * advances nothing. */
    dt_ms = m->started ? now_ms - m->last_ms : 0u;
    m->started = 1u;
    m->last_ms = now_ms;

    /* What the reply was, in its own terms (decision 5). Three cases, and the
     * third is not an error: an extended reply carrying a temperature is a
     * normal thing to receive and it is simply not a speed. */
    if (reply && reply->replied) {
        if (reply->status == AK_DSHOT_EDT_STOPPED) {
            have_speed = 1;
            stopped = 1;
        } else if (reply->status == AK_DSHOT_EDT_OK &&
                   reply->type == AK_DSHOT_EDT_ERPM) {
            have_speed = 1;
            erpm = (float)reply->erpm;
        }
    }

    if (have_speed) {
        if (!stopped) {
            /* What this motor is doing *now*, in the same units as the model:
             * eRPM per unit of command. Comparing it with the gain means the
             * comparison is between two of the same thing, and that is the
             * whole point of learning the model in those units rather than in
             * eRPM - a motor asked for a different throttle is not a departure.
             * Dividing the eRPM by the gain instead would produce the command
             * back, which is dimensionless, always inside [0,1], and would call
             * every motor off model at low throttle. */
            float instant = erpm / command;
            int on_model = 1;

            /* Decision 3, and only when there is a model to be far from
             * (decision 2). A gain that is zero is a gain that has only ever
             * been fed zeroes, and dividing by it is not a comparison. */
            if (m->gain_valid && m->gain > 0.0f) {
                float ratio = instant / m->gain;

                if (ratio > AK_MOTOR_HEALTH_RATIO ||
                    ratio < 1.0f / AK_MOTOR_HEALTH_RATIO) {
                    off_model = 1;
                    on_model = 0;
                }
            }

            /* Decision 2's second half: only a sample that agrees with the
             * model feeds it (see the header - the weaker rule, freezing the
             * model once the flag is up, is not enough and the tests say so). */
            if (on_model) {
                gain_feed(m, instant, dt_ms);
            }
        }

        window_add(&m->window, stopped, off_model);
    }

    m->flag = window_verdict(&m->window, m->flag);
    return m->flag;
}

ak_motor_health_flag_t ak_motor_health_of(const ak_motor_health_t *health,
                                          unsigned motor)
{
    if (!health || motor >= health->motors) {
        return AK_MOTOR_HEALTH_IDLE;
    }
    return health->motor[motor].flag;
}

float ak_motor_health_gain(const ak_motor_health_t *health, unsigned motor)
{
    if (!health || motor >= health->motors) {
        return 0.0f;
    }
    return health->motor[motor].gain_valid ? health->motor[motor].gain : 0.0f;
}

void ak_motor_health_counts(const ak_motor_health_t *health, unsigned motor,
                            uint32_t *answers, uint32_t *stopped,
                            uint32_t *off_model)
{
    const ak_motor_health_window_t *w;

    if (!health || motor >= health->motors) {
        if (answers) {
            *answers = 0u;
        }
        if (stopped) {
            *stopped = 0u;
        }
        if (off_model) {
            *off_model = 0u;
        }
        return;
    }

    w = &health->motor[motor].window;
    if (answers) {
        *answers = w->answer_sum;
    }
    if (stopped) {
        *stopped = w->stopped_sum;
    }
    if (off_model) {
        *off_model = w->off_model_sum;
    }
}

void ak_motor_health_forget(ak_motor_health_t *health, unsigned motor)
{
    ak_motor_health_motor_t *m;
    uint8_t in_band;

    if (!health || motor >= health->motors) {
        return;
    }

    m = &health->motor[motor];

    /* The window clears and the learned gain goes, but where the motor *is* -
     * in the band or below it - is a fact about the command, not about the
     * motor, and forgetting it would make the next in-band call look like the
     * decision 7 transition and clear a second time. */
    in_band = m->in_band;

    memset(m, 0, sizeof(*m));
    m->in_band = in_band;
    m->flag = AK_MOTOR_HEALTH_IDLE;
}

const char *ak_motor_health_flag_name(ak_motor_health_flag_t flag)
{
    switch (flag) {
    case AK_MOTOR_HEALTH_OK:
        return "ok";
    case AK_MOTOR_HEALTH_IDLE:
        return "idle";
    case AK_MOTOR_HEALTH_NO_TELEMETRY:
        return "no telemetry";
    case AK_MOTOR_HEALTH_OFF_MODEL:
        return "off model";
    case AK_MOTOR_HEALTH_STALLED:
        return "stalled";
    }
    return "unknown";
}
