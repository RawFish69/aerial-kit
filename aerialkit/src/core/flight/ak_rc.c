#include "ak_rc.h"

#include "ak_math.h"

void ak_rc_default_config(ak_rc_config_t *cfg)
{
    cfg->min = 172;
    cfg->mid = 992;
    cfg->max = 1811;
    cfg->deadband = 0.02f;
    cfg->mode_threshold = 1300;
    cfg->arm_threshold = 1300;
}

void ak_rc_cal_init(ak_rc_cal_t *cal, uint32_t wanted)
{
    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        cal->sum[i] = 0;
        cal->first[i] = 0;
    }
    cal->samples = 0;
    cal->wanted = wanted;
    cal->running = 0;
    cal->done = 0;
}

void ak_rc_cal_start(ak_rc_cal_t *cal)
{
    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        cal->sum[i] = 0;
        cal->first[i] = 0;
    }
    cal->samples = 0;
    cal->running = 1;
    cal->done = 0;
}

void ak_rc_cal_feed(ak_rc_cal_t *cal, const ak_rc_input_t *input)
{
    if (!cal->running || !input->valid) {
        return;
    }
    if (cal->samples == 0) {
        for (int i = 0; i < AK_RC_CHANNELS; i++) {
            cal->first[i] = input->channel[i];
        }
    }
    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        cal->sum[i] += input->channel[i];
    }
    cal->samples++;
    if (cal->samples >= cal->wanted) {
        cal->running = 0;
        cal->done = 1;
    }
}

int ak_rc_cal_apply(ak_rc_cal_t *cal, ak_rc_config_t *cfg,
                    int32_t offset[AK_RC_CHANNELS])
{
    if (!cal->done || cal->samples == 0) {
        return 0;
    }

    /* The four stick channels are the ones a person can hold still: roll,
     * pitch, yaw and throttle. The switches are not centred by definition, so
     * they are reported rather than used. */
    uint32_t centre = 0;
    uint32_t counted = 0;
    const int sticks[4] = { AK_RC_ROLL, AK_RC_PITCH, AK_RC_YAW, AK_RC_THROTTLE };

    for (int i = 0; i < 4; i++) {
        uint32_t measured = cal->sum[sticks[i]] / cal->samples;
        offset[sticks[i]] = (int32_t)measured - (int32_t)cfg->mid;
        if (i < 3) {
            /* Throttle is at the bottom, not the middle, so it does not get a
             * vote on where the middle is. */
            centre += measured;
            counted++;
        }
    }
    for (int i = AK_RC_MODE; i < AK_RC_CHANNELS; i++) {
        offset[i] = (int32_t)(cal->sum[i] / cal->samples) - (int32_t)cfg->mid;
    }

    if (counted > 0) {
        cfg->mid = centre / counted;
    }
    return 1;
}

static float centred(uint16_t raw, const ak_rc_config_t *cfg, float deadband)
{
    float value;
    if (raw >= cfg->mid) {
        float span = (float)(cfg->max - cfg->mid);
        value = span > 0.0f ? (float)(raw - cfg->mid) / span : 0.0f;
    } else {
        float span = (float)(cfg->mid - cfg->min);
        value = span > 0.0f ? -((float)(cfg->mid - raw) / span) : 0.0f;
    }
    value = ak_clampf(value, -1.0f, 1.0f);
    if (ak_absf(value) < deadband) {
        return 0.0f;
    }
    return value;
}

int ak_rc_decode(const ak_rc_input_t *input, const ak_rc_config_t *cfg,
                 ak_rc_command_t *out)
{
    out->roll = 0.0f;
    out->pitch = 0.0f;
    out->yaw = 0.0f;
    out->throttle = 0.0f;
    out->angle_mode = 0;
    out->arm_request = 0;

    if (!input->valid) {
        return 0;
    }

    /* A frame of all-zero counts is a disconnected receiver, not a stick held
     * at the bottom: refuse it rather than reading it as full down-throttle. */
    uint32_t sum = 0;
    for (int i = 0; i < AK_RC_CHANNELS; i++) {
        sum += input->channel[i];
    }
    if (sum == 0) {
        return 0;
    }

    out->roll = centred(input->channel[AK_RC_ROLL], cfg, cfg->deadband);
    out->pitch = centred(input->channel[AK_RC_PITCH], cfg, cfg->deadband);
    out->yaw = centred(input->channel[AK_RC_YAW], cfg, cfg->deadband);

    uint16_t raw_throttle = input->channel[AK_RC_THROTTLE];
    float span = (float)(cfg->max - cfg->min);
    out->throttle = span > 0.0f
                        ? ak_clampf((float)(raw_throttle - cfg->min) / span, 0.0f, 1.0f)
                        : 0.0f;

    out->angle_mode = input->channel[AK_RC_MODE] > cfg->mode_threshold;
    out->arm_request = input->channel[AK_RC_ARM] > cfg->arm_threshold;
    return 1;
}
