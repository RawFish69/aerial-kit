#ifndef AK_FLIGHT_OUTPUT_H
#define AK_FLIGHT_OUTPUT_H

#include "ak_types.h"

/*
 * Flight-core commands to something the hardware can emit.
 *
 * This file is only the arithmetic: a motor value to a DShot frame, a servo
 * value to a pulse width. Turning those numbers into edges on a pin is the
 * board's job (a timer and DMA, in the milestone that has hardware to test
 * against), and keeping the two apart is what lets every number in here be
 * checked on the host.
 */

/* DShot: 16 bits on the wire, sent most significant bit first.
 *
 *   15..5   throttle, 11 bits
 *    4      telemetry request
 *    3..0   crc, the 12-bit value folded by nibbles
 *
 * Throttle 0 is the disarmed command, 1..47 are reserved, 48 is zero throttle
 * and 2047 is full. A motor value of exactly zero therefore means "off", not
 * "the smallest possible thrust", which is the distinction that stops an
 * armed-but-idle aircraft from spinning its motors. */
#define AK_DSHOT_MIN_THROTTLE 48u
#define AK_DSHOT_MAX_THROTTLE 2047u

uint16_t ak_dshot_crc(uint16_t value12);
uint16_t ak_dshot_pack(uint16_t throttle, int telemetry);

/* Motor command 0..1 from the mixer, to a DShot throttle field. 0 maps to the
 * disarmed command; anything else maps into 48..2047. */
uint16_t ak_dshot_from_motor(float motor);

/* Servo pulse width in microseconds. Defaults are the common 1000..2000 with
 * 1500 at centre, which is what an analog or digital hobby servo expects. */
#define AK_SERVO_MIN_US    1000
#define AK_SERVO_CENTER_US 1500
#define AK_SERVO_MAX_US    2000

uint16_t ak_servo_pulse_us(float servo, uint16_t min_us, uint16_t center_us,
                           uint16_t max_us);

/*
 * And the plumbing between that number and a surface, which is the pilot's
 * rather than the airframe's.
 *
 * The mixer says how far an elevon should move and which way; whether the servo
 * *does* is a question of which side of the aircraft its arm is on and how long
 * the pushrod is. Both are things a person fixes with a servo arm - and when the
 * arm cannot be moved, with these three numbers:
 *
 *   - `reversed`, for a linkage that moves the surface the other way from the
 *     one the airframe's table assumes. This is the one that stops a wing
 *     flying: with one elevon's linkage mirrored, a roll command moves both
 *     surfaces the same way and the aircraft does not turn at all;
 *   - `trim_us`, added to the centre, for a surface whose neutral is not
 *     mechanically at 1500;
 *   - `travel_us`, how far the linkage actually moves at full stick, which is
 *     how two elevons are made to deflect by the same amount when their
 *     geometry differs.
 *
 * Neutral is the default - not reversed, no trim, 500 us each way - which is
 * what a linkage built to the drawing gives, and it is the only setting a
 * quadrotor ever uses because a quadrotor has no servos to trim.
 */
#define AK_SERVO_DEFAULT_TRAVEL_US 500u
#define AK_SERVO_MIN_TRAVEL_US     100u
#define AK_SERVO_MAX_TRAVEL_US     900u
#define AK_SERVO_MAX_TRIM_US       200
/* What a servo can survive, rather than what the linkage wants: a driving
 * command outside this is a parameter or an arithmetic mistake, and a servo
 * held against its stop is a servo that burns. */
#define AK_SERVO_HARD_MIN_US 750u
#define AK_SERVO_HARD_MAX_US 2250u

typedef struct {
    /* uint32_t and int32_t rather than the smallest types that fit, because
     * these three are parameters and the parameter table's numbers are u32 and
     * float: a struct the table can point straight at is a struct with no
     * second copy to keep in step. */
    uint32_t reversed;
    float    trim_us;   /* signed: the table's numbers are u32 and float, and a
                         * trim is the one that has to point either way */
    uint32_t travel_us;
} ak_servo_trim_t;

void ak_servo_trim_defaults(ak_servo_trim_t *trims, unsigned count);

/* One servo's pulse width with its own plumbing applied: `servo` is -1..1 from
 * the mixer, and the result is clamped to what the servo survives. */
uint16_t ak_servo_pulse_trimmed(float servo, const ak_servo_trim_t *trim);

/* The whole output frame for one loop iteration, in the form a board would
 * hand to its timers. */
typedef struct {
    uint16_t dshot[AK_MAX_MOTORS];
    uint16_t servo_us[AK_MAX_SERVOS];
} ak_output_frame_t;

/* The whole frame, with a trim per servo. A null `trims` is the neutral
 * mapping, which is what a caller that has no pilot's numbers yet wants. */
void ak_output_encode(const ak_outputs_t *out, const ak_servo_trim_t *trims,
                      int telemetry, ak_output_frame_t *frame);

#endif /* AK_FLIGHT_OUTPUT_H */
