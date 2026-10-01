#include "attitude_step.h"

#include "ak_math.h"

void ak_zyx_step(ak_euler_t *att, float p, float q, float r_dps, float dt)
{
    float roll = att->roll;
    float pitch = att->pitch;
    float sr = ak_sinf(roll), cr = ak_cosf(roll);
    float sp = ak_sinf(pitch), cp = ak_cosf(pitch);
    float r = ak_deg2rad(r_dps);
    float tan_pitch;

    if (cp > -0.05f && cp < 0.05f) {
        cp = cp < 0.0f ? -0.05f : 0.05f;
    }
    tan_pitch = sp / cp;

    att->roll = roll + (p + q * sr * tan_pitch + r * cr * tan_pitch) * dt;
    att->pitch = pitch + (q * cr - r * sr) * dt;
    att->heading_deg += ak_rad2deg((q * sr + r * cr) / cp) * dt;

    while (att->heading_deg >= 360.0f) {
        att->heading_deg -= 360.0f;
    }
    while (att->heading_deg < 0.0f) {
        att->heading_deg += 360.0f;
    }
}
