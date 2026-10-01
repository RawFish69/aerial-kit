#include "ak_output.h"

#include "ak_math.h"

uint16_t ak_dshot_crc(uint16_t value12)
{
    return (uint16_t)((value12 ^ (value12 >> 4) ^ (value12 >> 8)) & 0x0Fu);
}

uint16_t ak_dshot_pack(uint16_t throttle, int telemetry)
{
    /* The crc covers the top twelve bits of the frame - the eleven throttle
     * bits and the telemetry bit - so it is taken over (throttle << 1) |
     * telemetry, not over the shifted frame. Passing the shifted frame here
     * instead drops the (value >> 8) term and every crc comes out wrong, which
     * an ESC sees as a corrupt frame and ignores. The hand-derived vectors in
     * the test are what caught it: 0xFFFF for full throttle with telemetry,
     * 0x7D0A for throttle 1000 without. */
    uint16_t value12 =
        (uint16_t)(((throttle & 0x7FFu) << 1) | (telemetry ? 1u : 0u));
    return (uint16_t)((uint16_t)(value12 << 4) | ak_dshot_crc(value12));
}

uint16_t ak_dshot_from_motor(float motor)
{
    if (motor <= 0.0f) {
        return 0; /* disarmed: the only value a stopped motor gets */
    }
    float span = (float)(AK_DSHOT_MAX_THROTTLE - AK_DSHOT_MIN_THROTTLE);
    float throttle = (float)AK_DSHOT_MIN_THROTTLE +
                     ak_clampf(motor, 0.0f, 1.0f) * span;
    return (uint16_t)throttle;
}

uint16_t ak_servo_pulse_us(float servo, uint16_t min_us, uint16_t center_us,
                           uint16_t max_us)
{
    float value = ak_clampf(servo, -1.0f, 1.0f);
    if (value >= 0.0f) {
        float span = (float)(max_us - center_us);
        return (uint16_t)((float)center_us + value * span);
    }
    float span = (float)(center_us - min_us);
    return (uint16_t)((float)center_us + value * span);
}

void ak_servo_trim_defaults(ak_servo_trim_t *trims, unsigned count)
{
    for (unsigned i = 0; i < count; i++) {
        trims[i].reversed = 0u;
        trims[i].trim_us = 0;
        trims[i].travel_us = AK_SERVO_DEFAULT_TRAVEL_US;
    }
}

uint16_t ak_servo_pulse_trimmed(float servo, const ak_servo_trim_t *trim)
{
    /* Reversal first, because it is about the linkage rather than the servo:
     * everything below is arithmetic on the deflection the airframe asked for,
     * with one sign for a linkage that goes the other way. */
    float value = trim->reversed != 0u ? -servo : servo;
    int travel = (int)trim->travel_us;

    if (travel < (int)AK_SERVO_MIN_TRAVEL_US) {
        travel = (int)AK_SERVO_MIN_TRAVEL_US;
    }
    if (travel > (int)AK_SERVO_MAX_TRAVEL_US) {
        travel = (int)AK_SERVO_MAX_TRAVEL_US;
    }

    /* One float sum and one cast at the end, which is the shape
     * ak_servo_pulse_us() has: truncating the product first would bias every
     * pulse by up to a microsecond the other way, and a neutral trim has to
     * produce *exactly* the pulse it produced before trims existed. */
    float us = (float)AK_SERVO_CENTER_US + trim->trim_us +
               ak_clampf(value, -1.0f, 1.0f) * (float)travel;
    int pulse = (int)us;

    if (pulse < (int)AK_SERVO_HARD_MIN_US) {
        pulse = (int)AK_SERVO_HARD_MIN_US;
    }
    if (pulse > (int)AK_SERVO_HARD_MAX_US) {
        pulse = (int)AK_SERVO_HARD_MAX_US;
    }
    return (uint16_t)pulse;
}

void ak_output_encode(const ak_outputs_t *out, const ak_servo_trim_t *trims,
                      int telemetry, ak_output_frame_t *frame)
{
    for (int i = 0; i < AK_MAX_MOTORS; i++) {
        frame->dshot[i] = ak_dshot_pack(ak_dshot_from_motor(out->motor[i]),
                                        telemetry);
    }
    for (int i = 0; i < AK_MAX_SERVOS; i++) {
        if (trims != 0) {
            frame->servo_us[i] = ak_servo_pulse_trimmed(out->servo[i],
                                                        &trims[i]);
        } else {
            frame->servo_us[i] = ak_servo_pulse_us(out->servo[i],
                                                   AK_SERVO_MIN_US,
                                                   AK_SERVO_CENTER_US,
                                                   AK_SERVO_MAX_US);
        }
    }
}
