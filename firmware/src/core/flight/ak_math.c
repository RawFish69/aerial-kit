#include "ak_math.h"

#include <stdint.h>

float ak_sqrtf(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }

    /* Halve the exponent as a first guess - the classic bit trick - then let
     * Newton do the real work. Three steps from this seed leave a relative
     * error under 1e-6, finer than the sensor noise it will ever see. */
    union {
        float    f;
        uint32_t u;
    } v;
    v.f = x;
    v.u = (v.u >> 1) + 0x1fc00000u;

    float y = v.f;
    y = 0.5f * (y + x / y);
    y = 0.5f * (y + x / y);
    y = 0.5f * (y + x / y);
    return y;
}

float ak_atan2f(float y, float x)
{
    /* atan(2^-i) for i = 0..17 */
    static const float atan_table[18] = {
        0.785398163f, 0.463647609f, 0.244978663f, 0.124354995f,
        0.062418810f, 0.031239833f, 0.015623729f, 0.007812341f,
        0.003906230f, 0.001953123f, 0.000976562f, 0.000488281f,
        0.000244141f, 0.000122070f, 0.000061035f, 0.000030518f,
        0.000015259f, 0.000007629f,
    };

    if (x == 0.0f && y == 0.0f) {
        return 0.0f;
    }

    /* Fold the vector into the first quadrant and undo the fold afterwards.
     * The alternative - mirroring only x and adding pi - is wrong for a
     * negative y, and wrong by 2*phi rather than by 2*pi, which is why it is
     * worth doing this the dull way. Found by the host test that compares this
     * against libm over the whole circle. */
    int x_negative = x < 0.0f;
    int y_negative = y < 0.0f;
    float px = x_negative ? -x : x;
    float py = y_negative ? -y : y;

    /* Vectoring mode: rotate until py reaches zero, accumulating the angle.
     * Both inputs are non-negative, so the result is atan(py/px) in [0, pi/2].
     * The rotation gain does not matter here - only the angle does. */
    float angle = 0.0f;
    float step = 1.0f;

    for (int i = 0; i < 18; i++) {
        float nx;
        float ny;
        if (py > 0.0f) {
            nx = px + py * step;
            ny = py - px * step;
            angle += atan_table[i];
        } else {
            nx = px - py * step;
            ny = py + px * step;
            angle -= atan_table[i];
        }
        px = nx;
        py = ny;
        step *= 0.5f;
    }

    if (x_negative) {
        angle = AK_PI - angle;
    }
    return y_negative ? -angle : angle;
}

/* Shared range reduction: x = k * pi/2 + r, with |r| <= pi/4, so the
 * polynomials below only have to be good over a quarter of a period and the
 * quadrant decides the rest. */
static void reduce_quadrant(float x, int *quadrant, float *remainder)
{
    float scaled = x * 0.636619772f; /* 2/pi */
    int k = (int)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
    *quadrant = k & 3;
    *remainder = x - (float)k * 1.570796327f;
}

static void sin_cos_poly(float r, float *sin_r, float *cos_r)
{
    float r2 = r * r;

    *sin_r = r * (1.0f + r2 * (-1.0f / 6.0f +
                               r2 * (1.0f / 120.0f +
                                     r2 * (-1.0f / 5040.0f))));
    *cos_r = 1.0f + r2 * (-0.5f +
                          r2 * (1.0f / 24.0f +
                                r2 * (-1.0f / 720.0f +
                                      r2 * (1.0f / 40320.0f))));
}

float ak_sinf(float x)
{
    int quadrant;
    float r;
    float sin_r;
    float cos_r;

    reduce_quadrant(x, &quadrant, &r);
    sin_cos_poly(r, &sin_r, &cos_r);

    switch (quadrant) {
    case 1:
        return cos_r;
    case 2:
        return -sin_r;
    case 3:
        return -cos_r;
    default:
        return sin_r;
    }
}

float ak_cosf(float x)
{
    int quadrant;
    float r;
    float sin_r;
    float cos_r;

    reduce_quadrant(x, &quadrant, &r);
    sin_cos_poly(r, &sin_r, &cos_r);

    switch (quadrant) {
    case 1:
        return -sin_r;
    case 2:
        return -cos_r;
    case 3:
        return sin_r;
    default:
        return cos_r;
    }
}
