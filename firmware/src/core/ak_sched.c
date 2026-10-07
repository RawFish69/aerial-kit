#include "ak_sched.h"

#include "ak_time.h"

/*
 * The task table, and the three decisions in it that are not obvious.
 *
 * 1. **`due_us` is advanced arithmetically, not by a loop.** A task that has
 *    been away for an hour is not caught up one period at a time: the number of
 *    periods it never got is a division, and the next deadline is that division
 *    plus one. Looping would be correct and would also spend a millisecond of
 *    the *next* period counting the last one - which is the failure it is
 *    supposed to be reporting.
 *
 * 2. **A run is measured with the clock, not with the caller's `now`.** `now`
 *    is when the task *began*; nothing in it says how long the task took, and
 *    the host tests want to be able to drive both. So the duration comes from
 *    ak_time_us(), and a test that wants to exercise `overrun` moves the clock
 *    from inside the task - which is what an overrunning task does.
 *
 * 3. **The priority is a tie-break, not a preemption.** Every due task runs
 *    before `run` returns; the order decides which of them is *first*, and
 *    therefore which one pays for the clock read at the top of the period. A
 *    cooperative scheduler cannot promise more than that, and claiming more
 *    would be claiming preemption this does not have.
 */

typedef struct {
    const char      *name;
    void           (*fn)(void *);
    void            *ctx;
    ak_sched_prio_t  prio;
    uint32_t         period_us;
    uint32_t         due_us;
    ak_sched_stats_t stats;
    uint8_t          used;
    uint8_t          need_arm;
} ak_task_t;

static ak_task_t tasks[AK_SCHED_MAX_TASKS];
static int       used_count;

static void zero_stats(ak_sched_stats_t *s)
{
    s->runs        = 0u;
    s->late        = 0u;
    s->late_max_us = 0u;
    s->missed      = 0u;
    s->overrun     = 0u;
    s->last_us     = 0u;
    s->max_us      = 0u;
    s->period_us   = 0u;
}

void ak_sched_init(void)
{
    for (int i = 0; i < AK_SCHED_MAX_TASKS; i++) {
        tasks[i].name      = 0;
        tasks[i].fn        = 0;
        tasks[i].ctx       = 0;
        tasks[i].prio      = AK_SCHED_IDLE;
        tasks[i].period_us = 0u;
        tasks[i].due_us    = 0u;
        tasks[i].used      = 0u;
        tasks[i].need_arm  = 0u;
        zero_stats(&tasks[i].stats);
    }
    used_count = 0;
}

int ak_sched_add(const char *name, void (*fn)(void *), void *ctx,
                 uint32_t period_us, ak_sched_prio_t prio)
{
    if (fn == 0 || period_us == 0u) {
        return -1;
    }
    for (int i = 0; i < AK_SCHED_MAX_TASKS; i++) {
        if (tasks[i].used) {
            continue;
        }
        tasks[i].name      = name;
        tasks[i].fn        = fn;
        tasks[i].ctx       = ctx;
        tasks[i].prio      = prio;
        tasks[i].period_us = period_us;
        tasks[i].due_us    = 0u;
        tasks[i].used      = 1u;
        /* Its first deadline is set by the next run, from that run's `now`.
         * Arming it here would need a `now` this call does not have, and
         * guessing one would put a deadline on a clock nobody read. */
        tasks[i].need_arm  = 1u;
        zero_stats(&tasks[i].stats);
        tasks[i].stats.period_us = period_us;
        used_count++;
        return i;
    }
    return -1;
}

int ak_sched_set_period(int task, uint32_t period_us, uint32_t now_us)
{
    if (task < 0 || task >= AK_SCHED_MAX_TASKS) {
        return -1;
    }
    if (!tasks[task].used || period_us == 0u) {
        return -1;
    }
    tasks[task].period_us            = period_us;
    tasks[task].stats.period_us      = period_us;
    /* From now, not from the old deadline: shortening a period must not leave
     * a deadline in the past that fires on the next pass and is counted as a
     * miss the caller caused. */
    tasks[task].due_us               = now_us + period_us;
    tasks[task].need_arm             = 0u;
    return 0;
}

void ak_sched_arm(uint32_t now_us)
{
    for (int i = 0; i < AK_SCHED_MAX_TASKS; i++) {
        if (!tasks[i].used) {
            continue;
        }
        tasks[i].due_us   = now_us + tasks[i].period_us;
        tasks[i].need_arm = 0u;
    }
}

/* The task to run now: the highest priority among those whose deadline has
 * passed, and the lowest id among equals - so the order two tasks of the same
 * priority run in is the order they were added, which is a thing a caller can
 * see rather than a thing that depends on the table's internal layout. */
static int pick_due(uint32_t now_us)
{
    int best = -1;

    for (int i = 0; i < AK_SCHED_MAX_TASKS; i++) {
        if (!tasks[i].used || tasks[i].need_arm) {
            continue;
        }
        /* The wrap-safe comparison: `due` is in the past when the signed
         * difference is negative. Comparing the two with `<` would be wrong
         * across the 71-minute wrap, which is the rule ak_time.h states. */
        if ((int32_t)(now_us - tasks[i].due_us) < 0) {
            continue;
        }
        if (best < 0 || tasks[i].prio > tasks[best].prio) {
            best = i;
        }
    }
    return best;
}

static void run_task(int id, uint32_t now_us)
{
    ak_task_t *t = &tasks[id];
    uint32_t   lateness = now_us - t->due_us;   /* >= 0: pick_due said so */
    uint32_t   start    = ak_time_us();
    uint32_t   took;

    t->stats.runs++;
    /* The slack this task's own period earns it - see the constant's comment
     * for why it is not flat. Read here rather than at registration because
     * ak_sched_set_period() can change the period under a running table. */
    if (lateness > ak_sched_slack_us(t->period_us)) {
        t->stats.late++;
        if (lateness > t->stats.late_max_us) {
            t->stats.late_max_us = lateness;
        }
    }

    t->fn(t->ctx);

    took = ak_time_us() - start;
    t->stats.last_us = took;
    if (took > t->stats.max_us) {
        t->stats.max_us = took;
    }
    if (took > t->period_us) {
        t->stats.overrun++;
    }

    /* Past every period that has already elapsed, counting the ones it never
     * got. See the note at the top of this file for why this is a division
     * rather than a loop. */
    {
        uint32_t behind = (uint32_t)(now_us - t->due_us);
        uint32_t skip   = behind / t->period_us;

        t->due_us += (skip + 1u) * t->period_us;
        t->stats.missed += skip;
    }
}

int ak_sched_run(uint32_t now_us)
{
    int ran = 0;

    /* A task added since the last arm gets its first deadline from this call's
     * `now`, and does not run on this pass. A task that has just been added has
     * not been waiting for anything. */
    for (int i = 0; i < AK_SCHED_MAX_TASKS; i++) {
        if (tasks[i].used && tasks[i].need_arm) {
            tasks[i].due_us   = now_us + tasks[i].period_us;
            tasks[i].need_arm = 0u;
        }
    }

    for (;;) {
        int id = pick_due(now_us);
        if (id < 0) {
            break;
        }
        run_task(id, now_us);
        ran++;
    }
    return ran;
}

uint32_t ak_sched_next_due_us(uint32_t now_us)
{
    uint32_t earliest = 0u;
    int      have     = 0;

    for (int i = 0; i < AK_SCHED_MAX_TASKS; i++) {
        if (!tasks[i].used || tasks[i].need_arm) {
            continue;
        }
        /* Deadlines already gone by are not something to wait for: the next
         * `ak_sched_run` runs them. Only the future is interesting here, which
         * is also why zero means "nothing to wait for" rather than a time. */
        if ((int32_t)(tasks[i].due_us - now_us) <= 0) {
            continue;
        }
        if (!have || (int32_t)(tasks[i].due_us - earliest) < 0) {
            earliest = tasks[i].due_us;
            have     = 1;
        }
    }
    return have ? earliest : 0u;
}

int ak_sched_stats(int task, ak_sched_stats_t *out)
{
    if (task < 0 || task >= AK_SCHED_MAX_TASKS || !tasks[task].used) {
        return -1;
    }
    if (out == 0) {
        return -1;
    }
    *out = tasks[task].stats;
    return 0;
}

const char *ak_sched_name(int task)
{
    if (task < 0 || task >= AK_SCHED_MAX_TASKS || !tasks[task].used) {
        return 0;
    }
    return tasks[task].name;
}

int ak_sched_count(void)
{
    return used_count;
}
