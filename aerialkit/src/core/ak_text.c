#include "ak_text.h"

unsigned ak_strlen(const char *text)
{
    unsigned n = 0;
    while (text[n] != '\0') {
        n++;
    }
    return n;
}

void ak_strlcpy(char *dst, const char *src, unsigned len)
{
    if (len == 0) {
        return;
    }
    unsigned i = 0;
    while (src[i] != '\0' && i + 1u < len) {
        dst[i] = src[i];
        i++;
    }
    dst[i] = '\0';
}

int ak_str_eq(const char *a, const char *b)
{
    while (*a != '\0' && *a == *b) {
        a++;
        b++;
    }
    return *a == *b;
}

int ak_str_starts_with(const char *text, const char *prefix)
{
    while (*prefix != '\0') {
        if (*text != *prefix) {
            return 0;
        }
        text++;
        prefix++;
    }
    return 1;
}

const char *ak_skip_spaces(const char *text)
{
    while (*text == ' ' || *text == '\t') {
        text++;
    }
    return text;
}

static int is_digit(char c)
{
    return c >= '0' && c <= '9';
}

static float scale_by_power_of_ten(float value, int exponent)
{
    if (exponent > 30) {
        exponent = 30;
    } else if (exponent < -30) {
        exponent = -30;
    }
    while (exponent > 0) {
        value *= 10.0f;
        exponent--;
    }
    while (exponent < 0) {
        value *= 0.1f;
        exponent++;
    }
    return value;
}

int ak_parse_float(const char *text, float *out)
{
    const char *p = ak_skip_spaces(text);
    int negative = 0;

    if (*p == '+' || *p == '-') {
        negative = *p == '-';
        p++;
    }

    float value = 0.0f;
    int digits = 0;
    while (is_digit(*p)) {
        value = value * 10.0f + (float)(*p - '0');
        p++;
        digits++;
    }
    if (*p == '.') {
        p++;
        /* The fraction is collected as an integer and divided once, rather
         * than added a digit at a time.
         *
         * `value += digit * place` with `place *= 0.1f` keeps a rounding error
         * in `place` that every later digit is multiplied by. It is small and
         * it is not harmless: "0.0500" came back two float steps *above* the
         * 0.05 the board had just printed as `rate_kd_yaw`'s own maximum, so
         * the parameter refused its own declared bound and the console showed
         * "out of range 0.0000..0.0500" for the number 0.0500. Three of the
         * flight table's sixty-six bounds did this.
         *
         * A single division is correctly rounded, so the value that comes back
         * is the nearest float to the decimal that was written - which is what
         * makes the board's own `format` and `parse` inverses of each other.
         *
         * The scale stops at 10^7 because that is the largest power of ten a
         * float holds exactly (10^8 needs 27 bits), and digits beyond it
         * cannot change a float anyway: they are past its precision. */
        long fraction = 0;
        long scale = 1;
        while (is_digit(*p) && scale <= 1000000L) {
            fraction = fraction * 10 + (*p - '0');
            scale *= 10;
            p++;
            digits++;
        }
        while (is_digit(*p)) {
            p++;
            digits++;
        }
        value += (float)fraction / (float)scale;
    }
    if (digits == 0) {
        return 0;
    }

    if (*p == 'e' || *p == 'E') {
        p++;
        int exponent_negative = 0;
        if (*p == '+' || *p == '-') {
            exponent_negative = *p == '-';
            p++;
        }
        if (!is_digit(*p)) {
            return 0;
        }
        int exponent = 0;
        while (is_digit(*p)) {
            exponent = exponent * 10 + (*p - '0');
            p++;
        }
        value = scale_by_power_of_ten(value, exponent_negative ? -exponent : exponent);
    }

    p = ak_skip_spaces(p);
    if (*p != '\0') {
        return 0;
    }

    *out = negative ? -value : value;
    return 1;
}

int ak_parse_int(const char *text, int32_t *out)
{
    /* Parsed as an integer, not through the float path: a timeout in
     * milliseconds should not lose its low bits on the way in. */
    const char *p = ak_skip_spaces(text);
    int negative = 0;

    if (*p == '+' || *p == '-') {
        negative = *p == '-';
        p++;
    }
    if (!is_digit(*p)) {
        return 0;
    }

    uint32_t value = 0;
    while (is_digit(*p)) {
        uint32_t digit = (uint32_t)(*p - '0');
        if (value > (429496729u - digit) / 10u) {
            return 0; /* would overflow 32 bits */
        }
        value = value * 10u + digit;
        p++;
    }

    p = ak_skip_spaces(p);
    if (*p != '\0') {
        return 0;
    }

    uint32_t limit = negative ? 2147483648u : 2147483647u;
    if (value > limit) {
        return 0;
    }
    *out = negative ? (int32_t)(0u - value) : (int32_t)value;
    return 1;
}

int ak_parse_uint(const char *text, uint32_t *out)
{
    const char *p = ak_skip_spaces(text);
    if (*p == '+') {
        p++;
    }
    if (!is_digit(*p)) {
        return 0;
    }
    uint32_t value = 0;
    while (is_digit(*p)) {
        uint32_t digit = (uint32_t)(*p - '0');
        if (value > (0xFFFFFFFFu - digit) / 10u) {
            return 0; /* overflow */
        }
        value = value * 10u + digit;
        p++;
    }
    p = ak_skip_spaces(p);
    if (*p != '\0') {
        return 0;
    }
    *out = value;
    return 1;
}

static unsigned emit(char *buf, unsigned len, unsigned pos, char c)
{
    if (pos + 1u < len) {
        buf[pos] = c;
    }
    return pos + 1u;
}

static unsigned emit_terminate(char *buf, unsigned len, unsigned pos)
{
    if (len == 0) {
        return 0;
    }
    buf[pos < len ? pos : len - 1u] = '\0';
    return pos < len ? pos : len - 1u;
}

unsigned ak_format_uint(uint32_t value, unsigned width, char *buf, unsigned len)
{
    char digits[10];
    unsigned n = 0;

    if (value == 0) {
        digits[n++] = '0';
    }
    while (value > 0) {
        digits[n++] = (char)('0' + (value % 10u));
        value /= 10u;
    }

    unsigned pos = 0;
    while (n < width) {
        pos = emit(buf, len, pos, '0');
        width--;
    }
    while (n > 0) {
        pos = emit(buf, len, pos, digits[--n]);
    }
    return emit_terminate(buf, len, pos);
}

unsigned ak_format_fixed(float value, unsigned decimals, char *buf, unsigned len)
{
    if (len == 0) {
        return 0;
    }
    /* Seven, because a latitude is written in seven decimal places and
     * printing one a decimal short is a position ten metres out. More than
     * seven would need more than 32 bits of fixed point for the numbers that
     * matter here. */
    if (decimals > 7) {
        decimals = 7;
    }

    uint32_t scale = 1;
    for (unsigned i = 0; i < decimals; i++) {
        scale *= 10u;
    }

    int negative = value < 0.0f;
    float magnitude = negative ? -value : value;
    /* Clamped to what still fits after scaling. Clamping the value instead
     * would let a large number with decimals wrap round and print as a small
     * one - a gain of a million read back as a gain of four. */
    float limit = 4294967040.0f / (float)scale;
    if (magnitude > limit) {
        magnitude = limit;
    }

    uint32_t fixed = (uint32_t)(magnitude * (float)scale + 0.5f);
    uint32_t whole = fixed / scale;
    uint32_t fraction = fixed % scale;

    unsigned pos = 0;
    if (negative) {
        pos = emit(buf, len, pos, '-');
    }

    char digits[10];
    unsigned n = 0;
    if (whole == 0) {
        digits[n++] = '0';
    }
    while (whole > 0) {
        digits[n++] = (char)('0' + (whole % 10u));
        whole /= 10u;
    }
    while (n > 0) {
        pos = emit(buf, len, pos, digits[--n]);
    }

    if (decimals > 0) {
        pos = emit(buf, len, pos, '.');
        for (unsigned i = decimals; i > 0; i--) {
            uint32_t divisor = 1;
            for (unsigned j = 1; j < i; j++) {
                divisor *= 10u;
            }
            pos = emit(buf, len, pos, (char)('0' + (fraction / divisor) % 10u));
        }
    }

    return emit_terminate(buf, len, pos);
}
