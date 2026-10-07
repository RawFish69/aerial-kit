#ifndef AK_CORE_AK_SCHED_H
#define AK_CORE_AK_SCHED_H

#include <stdint.h>

/*
 * The task table.
 *
 * The firmware's main loop has been a superloop since before there was anything
 * to schedule: a `for (;;)` with a handful of `next_something_ms` deadlines
 * tested at the top and a great deal of work that runs every pass. That is not
 * wrong, and it is why the thing flew before this existed. What it cannot do is
 * *say* anything: doc 29 could report that the loop's period was one
 * millisecond and nothing about whether any particular piece of work met its
 * own deadline, because no piece of work had one - the gates were local
 * variables in one function and their lateness was never counted.
 *
 * This is a table of periods, priorities and statistics, and it is deliberately
 * not more than that:
 *
 * 1. **Cooperative, with no preemption.** A task runs to completion. There is
 *    no stack per task, no context switch, and no way for a task to be
 *    interrupted by another - which is what makes it safe to put this in a
 *    flight controller. The cost is that a task which overruns delays every
 *    task below it, and that is exactly the thing the statistics below exist to
 *    show rather than hide.
 *
 * 2. **`now` is handed in, never read.** `ak_sched_run(now_us)` takes the time
 *    from its caller, so the host tests and `tools/ak_control.c` drive the same
 *    code on a virtual clock with no board and no SysTick - and so that nothing
 *    in here has to know which port it is on. Ground rule 7: `src/core/` stays
 *    MCU-free.
 *
 * 3. **A missed period is skipped and counted, not caught up.** A task that was
 *    due three times while the loop was busy runs *once*, and the two periods it
 *    never got are counted in `missed`. Running it three times back to back
 *    would be the other choice and it is the wrong one here: the work is a
 *    control law reading a sensor, and running it three times on one sample
 *    computes the same answer three times and then falls further behind. Count
 *    it and let a person see it.
 *
 * The statistics are the point of the file. `ak_perf` measures one loop - its
 * sections, its jitter, its duration - and stays what it is; a table of tasks
 * each with a period needs its own accounting, and this is it.
 */

/* Twelve is a fixed ceiling rather than a malloc because there is no allocator
 * in the firmware and a table that can fail to allocate at boot is a table that
 * can silently lose a task. A board that needs a thirteenth gets the number
 * raised here, deliberately, in a diff that shows what the RAM costs. */
#define AK_SCHED_MAX_TASKS 12

/* A task that begins within its slack of its deadline is on time.
 *
 * It is not zero, and that is a decision: the deadline is a comparison of two
 * clock readings taken at different moments, and a task that begins 200 ns
 * after its period elapsed has not missed anything a flight controller cares
 * about. A twentieth of the task's own period, which is fifty microseconds at
 * the one-kilohertz loop this firmware ran at before phase 1.4 and is the same
 * fifty microseconds there today.
 *
 * It was a flat fifty microseconds, and phase 1.4 is what made that wrong: the
 * loop's rate is a parameter now, and at four kilohertz a flat fifty is a fifth
 * of a period. A threshold that is a different fraction of the thing it is
 * measuring at every rate is a threshold whose number does not mean the same
 * thing twice - and the direction of the error is the bad one, because a
 * too-large slack *under*-reports lateness, and "zero late runs" is one of the
 * numbers this firmware is judged on. A tenth of a period would have changed
 * what the one-kilohertz loop reports, so a twentieth it is.
 *
 * The floor is for the clock rather than for the period: reading a microsecond
 * counter is exact, but the deadline and the reading are taken at different
 * moments, so a task whose period is shorter than a hundred microseconds would
 * otherwise be given a slack smaller than the granularity of the comparison
 * itself. Lateness beyond the slack is counted, and `late_max_us` says how far.
 */
#define AK_SCHED_SLACK_DIVISOR 20u
#define AK_SCHED_SLACK_MIN_US  5u

/* The slack for one task's period, in microseconds. */
static inline uint32_t ak_sched_slack_us(uint32_t period_us)
{
    const uint32_t slack = period_us / AK_SCHED_SLACK_DIVISOR;
    return slack > AK_SCHED_SLACK_MIN_US ? slack : AK_SCHED_SLACK_MIN_US;
}

typedef enum {
    AK_SCHED_IDLE = 0,   /* runs when nothing else is due */
    AK_SCHED_LOW,
    AK_SCHED_NORMAL,
    AK_SCHED_HIGH,       /* the fast loop: the one that must not be delayed */
} ak_sched_prio_t;

/* What a task has done, and what it cost. Every field is a count or a duration
 * the scheduler measured itself - none is inferred from a period. */
typedef struct {
    uint32_t runs;          /* times the task has been entered */
    uint32_t late;          /* runs that began later than deadline + slack */
    uint32_t late_max_us;   /* the worst of those, in microseconds */
    uint32_t missed;        /* periods skipped because the task could not keep up */
    uint32_t overrun;       /* runs whose own duration exceeded its period */
    uint32_t last_us;       /* how long the most recent run took */
    uint32_t max_us;        /* the longest run since reset */
    uint32_t period_us;     /* the period in force now, which may have been re-armed */
} ak_sched_stats_t;

/* Empty the table and every counter. Safe to call again; a task added before a
 * reset is dropped, which is what makes this usable from a test and from a
 * console command that wants to rebuild the table. */
void ak_sched_init(void);

/*
 * Add a task. Returns its id (0..AK_SCHED_MAX_TASKS-1) or -1 if the table is
 * full, if `fn` is null, or if `period_us` is zero.
 *
 * The name is kept by pointer and must outlive the task - every caller in this
 * tree passes a string literal. It exists for the report and for the console,
 * because a table of periods nobody can read is a table nobody will check.
 *
 * A task added while the scheduler is running is *not* armed until
 * `ak_sched_arm` is called again; a task added before the first `run` gets its
 * first deadline from the arm that `main` performs once at boot.
 */
int ak_sched_add(const char *name, void (*fn)(void *), void *ctx,
                 uint32_t period_us, ak_sched_prio_t prio);

/* Change a task's period, and re-arm its deadline from `now_us` rather than
 * leaving it where the old period put it. This is for the rates a client asks
 * for - the telemetry stream and the log stream are both set from the wire -
 * and it is why the deadline is not simply `previous + new_period`: shortening
 * a period must not produce a deadline in the past that fires immediately and
 * counts as a miss. Returns 0 on success, -1 for an unknown task or a zero
 * period. */
int ak_sched_set_period(int task, uint32_t period_us, uint32_t now_us);

/* Set every deadline to now_us + that task's period. Called once at boot, and
 * again after a task's period changes. */
void ak_sched_arm(uint32_t now_us);

/* Run every task whose deadline has passed, highest priority first, and return
 * how many ran. A task's deadline is advanced to the next period that has not
 * yet passed, counting the ones that were skipped.
 *
 * This is the whole scheduler. It does not sleep, block or yield: a caller that
 * has nothing to run calls it again later with a later `now`. */
int ak_sched_run(uint32_t now_us);

/* The soonest deadline still *ahead of* `now_us`, as an absolute microsecond
 * time. Zero means there is nothing to wait for - either the table is empty or
 * work is already due - and those are deliberately the same answer, because the
 * caller's next move is the same either way: call `ak_sched_run`. A caller that
 * wants to idle uses this to know how long it has; nothing in here sleeps on
 * its own, and nothing in here reads the clock. */
uint32_t ak_sched_next_due_us(uint32_t now_us);

/* One task's statistics. Returns 0 and fills `out`, or -1 if there is no such
 * task - which is a different answer from a task that has never run, and the
 * two are kept apart on purpose: a zeroed struct means "this has not happened
 * yet", -1 means "there is no such thing". */
int ak_sched_stats(int task, ak_sched_stats_t *out);

/* The task's name, or null. */
const char *ak_sched_name(int task);

/* How many tasks are in the table, which is what a report iterates. */
int ak_sched_count(void);

#endif /* AK_CORE_AK_SCHED_H */
