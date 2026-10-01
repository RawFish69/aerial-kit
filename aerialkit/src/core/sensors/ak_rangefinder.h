#ifndef AK_SENSORS_AK_RANGEFINDER_H
#define AK_SENSORS_AK_RANGEFINDER_H

#include <stdint.h>

#include "ak_bus.h"
#include "ak_console.h"

/*
 * A rangefinder: how far the ground is, when it is close enough to say.
 *
 * Every other sensor on this aircraft measures it in *one* frame of reference
 * that drifts. The barometer measures pressure, which is weather plus height,
 * and the aircraft anchors it once on the ground - so the height the navigator
 * flies is a change since take-off, good to tens of centimetres per flight but
 * with a reference that leaks, and the last metre of a descent is exactly
 * where a leak of a few metres costs something. A GPS altitude is absolute and
 * wanders by a few metres while it is being used.
 *
 * A rangefinder pointed down measures the one thing a landing needs and
 * nothing else does: the distance to the *ground below*. It is short-ranged,
 * it is fooled by what the ground is made of, and it says nothing at all above
 * its own reach - which is why nothing here replaces the barometer. It is a
 * measurement of the last few metres, and the two places that matter are the
 * two this firmware has been unable to answer: *is the aircraft on the
 * ground* (so stopping the motors is a measurement rather than an inference
 * from a height that has stopped falling), and *how far above the ground is
 * it* while the navigator is bringing it down onto a spot with no position
 * fix to aim at.
 *
 * The interface follows the barometer's: a driver table probed on the bus,
 * one reading at a time, and the arithmetic and the filtering in the core
 * where a host test can hold it against a modelled part.
 */

/* The part answered and there is nothing within its reach. A distance is a
 * number of millimetres; this is the absence of one. */
#define AK_RANGE_NONE (-1)

typedef struct {
    const char *name;
    uint8_t     address;   /* I2C address, which is a strap on the part */
    uint32_t    period_ms; /* how often it may be sampled */
    int32_t     max_mm;    /* beyond this it reports nothing in range */
    int32_t     min_mm;    /* below this the reading is the part's own noise */
    /* 0 when the part answers on this bus, -1 when it does not. */
    int (*probe)(const ak_bus_t *bus);
    /* 1 and a distance in millimetres, 0 when the part answered "nothing in
     * range", -1 when the read failed. */
    int (*read)(const ak_bus_t *bus, int32_t *distance_mm);
} ak_rangefinder_driver_t;

extern const ak_rangefinder_driver_t ak_range_tof10120;

/* Every driver this build knows about, ending with a null entry, so a test can
 * say which one it expected rather than only that something answered. */
extern const ak_rangefinder_driver_t *const ak_range_drivers[];

typedef struct {
    const ak_bus_t                *bus;
    const ak_rangefinder_driver_t *driver;
    int      present;
    /* The last reading the filter kept, in millimetres, or AK_RANGE_NONE for
     * "no ground within reach" or "nothing read yet". */
    int32_t  distance_mm;
    uint32_t last_ms;      /* when that reading arrived */
    uint32_t poll_ms;      /* when the part was last asked */
    int      polled;       /* ... and whether it ever has been */
    uint32_t samples;      /* readings kept */
    uint32_t out_of_range; /* times the part said nothing was in range */
    uint32_t faults;       /* reads that failed */
    uint32_t rejected;     /* readings the filter threw away */
    int      reject_streak;
} ak_rangefinder_t;

/*
 * A reading that says the ground moved faster than the aircraft can move is
 * not the aircraft.
 *
 * The quantity is a *rate*, not a step, and that is the correction this filter
 * needed: the first version compared a change against a fixed number of
 * millimetres whatever the time since the last reading, so the same physical
 * motion was impossible after a short gap and fine after a long one, and a
 * slow descent that happened to be sampled late looked like a spike while a
 * real glitch during a burst of samples did not. Eight metres a second is what
 * "the aircraft cannot have done that" means here - the navigator's own
 * descent is capped at three, so this is more than twice the fastest descent
 * the firmware will ever command, and a part is only disbelieved when nothing
 * with props on it could have moved that far.
 *
 * A reading that cannot be the aircraft is *rejected* rather than clamped, so
 * what the two callers get is either a measurement or nothing - never a number
 * somebody has quietly edited. The baseline is the last reading that was
 * believed, so one that has been refused is not refused forever: the aircraft
 * has that much longer to have moved, and after a long enough gap anything
 * within the part's range is physically possible and is believed again.
 *
 * What this cannot see is a part that is *steadily* wrong. A lidar looking
 * down at a hedge from twenty metres reports a metre and a half for as long as
 * the aircraft is over the hedge, and no comparison with the previous reading
 * can tell that from a landing. That is the job of the rule that uses the
 * number, and it is done there: a landing is refused unless the rangefinder
 * and the height estimate agree about where the aircraft is. The two guards
 * are deliberately different - one is about time, the other about agreement -
 * because the failures they catch are.
 */
#define AK_RANGE_SPIKE_MM_S    8000
/* A reading older than this is not a measurement of anything: the part is
 * polled at 20 Hz and this is ten samples. */
#define AK_RANGE_STALE_MS      500u

/* The first driver whose probe answers, or null. */
const ak_rangefinder_driver_t *ak_rangefinder_detect(const ak_bus_t *bus);

int ak_rangefinder_open(ak_rangefinder_t *rgf, const ak_bus_t *bus,
                        ak_printf_fn out);

/*
 * What one call to `ak_rangefinder_read` did.
 *
 * The three answers are three different facts, and conflating any two of them
 * is how a caller gets its bookkeeping wrong: a part that is not asked this
 * pass is not a part that failed, and a part that answered "nothing in range"
 * is not a part that failed either. The first version of this returned 0 for
 * both of those and -1 for a fault, and the console's "the rangefinder has
 * gone quiet / it is answering again" pair then flipped once per poll for as
 * long as a part was dead, with the count of failed reads never rising above
 * one. The simulation found it; see tests/test_rangefinder.c.
 */
typedef enum {
    AK_RANGE_FAULT = -1,     /* the part did not answer */
    AK_RANGE_NOTHING = 0,    /* it answered, and the reading was not usable */
    AK_RANGE_READING = 1,    /* a new reading, kept in `distance_mm` */
    AK_RANGE_IDLE = 2,       /* it was not this part's turn this pass */
} ak_range_result_t;

/* One measurement, when the part is due one. The caller passes the clock so
 * that the period and the staleness are measured on the same time base as
 * everything else. */
int ak_rangefinder_read(ak_rangefinder_t *rgf, uint32_t now_ms,
                        int32_t *distance_mm);

/* Is there a distance, and is it younger than `hold_ms`? */
int ak_rangefinder_valid(const ak_rangefinder_t *rgf, uint32_t now_ms,
                         uint32_t hold_ms);

#endif /* AK_SENSORS_AK_RANGEFINDER_H */
