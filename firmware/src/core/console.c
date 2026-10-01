#include "ak_console.h"

#include <stddef.h>
#include <stdarg.h>
#include <stdint.h>

/* Small bounded formatter. Output is assembled in a fixed buffer and handed to
 * the board sink in one call, so an interrupt that also prints cannot
 * interleave inside a line. */
#define AK_CONSOLE_LINE 160

typedef struct {
    char   buf[AK_CONSOLE_LINE];
    size_t len;
} sink_t;

static void put(sink_t *out, char c)
{
    if (out->len + 1u < sizeof out->buf) {
        out->buf[out->len++] = c;
    }
}

static void pad_to(sink_t *out, size_t from, int width, char fill)
{
    int have = (int)(out->len - from);
    while (have < width) {
        put(out, fill);
        have++;
    }
}

static void put_padded(sink_t *out, const char *text, int width, int left)
{
    size_t start = out->len;
    size_t length = 0;

    while (text[length] != '\0') {
        length++;
    }

    /*
     * A field width pads on one side or the other, and this used to pad only
     * when the `-` flag asked for the right: `%10s` printed ten characters of
     * nothing and then the text. The parameter table is where that showed -
     * `out("  %-16s %10s  %s\n", name, value, help)` is meant to line the
     * values up in a column, and the column was ragged instead, longest value
     * to shortest. The banner's own text is written with `%-16s` and was
     * always right, which is why nobody had noticed the other half.
     */
    if (!left && (int)length < width) {
        for (int i = 0; i < width - (int)length; i++) {
            put(out, ' ');
        }
    }
    for (size_t i = 0; i < length; i++) {
        put(out, text[i]);
    }
    if (left) {
        pad_to(out, start, width, ' ');
    }
}

static void put_number(sink_t *out, uint32_t value, unsigned base, int upper,
                       int width, int left, int zero)
{
    char digits[32];
    unsigned n = 0;
    const char *alphabet = upper ? "0123456789ABCDEF" : "0123456789abcdef";

    if (value == 0) {
        digits[n++] = '0';
    }
    while (value != 0) {
        digits[n++] = alphabet[value % base];
        value /= base;
    }

    size_t start = out->len;
    if (!left && zero) {
        pad_to(out, start, width - (int)n, '0');
    }
    while (n > 0) {
        put(out, digits[--n]);
    }
    if (left) {
        pad_to(out, start, width, ' ');
    }
}

static int format(sink_t *out, const char *fmt, va_list ap)
{
    for (const char *p = fmt; *p != '\0'; p++) {
        if (*p != '%') {
            put(out, *p);
            continue;
        }
        p++;
        int left = 0, zero = 0;
        for (;; p++) {
            if (*p == '-') {
                left = 1;
            } else if (*p == '0') {
                zero = 1;
            } else {
                break;
            }
        }
        int width = 0;
        while (*p >= '0' && *p <= '9') {
            width = width * 10 + (*p - '0');
            p++;
        }
        switch (*p) {
        case 's': {
            const char *s = va_arg(ap, const char *);
            put_padded(out, s != NULL ? s : "(null)", width, left);
            break;
        }
        case 'c':
            put(out, (char)va_arg(ap, int));
            break;
        case 'd':
        case 'i': {
            int32_t v = va_arg(ap, int32_t);
            if (v < 0) {
                put(out, '-');
                if (width > 0) {
                    width--;
                }
                put_number(out, (uint32_t)(-(int64_t)v), 10, 0, width, left, zero);
            } else {
                put_number(out, (uint32_t)v, 10, 0, width, left, zero);
            }
            break;
        }
        case 'u':
            put_number(out, va_arg(ap, uint32_t), 10, 0, width, left, zero);
            break;
        case 'x':
            put_number(out, va_arg(ap, uint32_t), 16, 0, width, left, zero);
            break;
        case 'X':
            put_number(out, va_arg(ap, uint32_t), 16, 1, width, left, zero);
            break;
        case 'p':
            put_padded(out, "0x", 0, 0);
            put_number(out, (uint32_t)(uintptr_t)va_arg(ap, void *), 16, 0, 8, 0, 1);
            break;
        case '%':
            put(out, '%');
            break;
        case '\0':
            out->buf[out->len] = '\0';
            return (int)out->len;
        default:
            put(out, '%');
            put(out, *p);
            break;
        }
    }
    out->buf[out->len] = '\0';
    return (int)out->len;
}

void ak_console_write(const char *text)
{
    unsigned n = 0;
    while (text[n] != '\0') {
        n++;
    }
    if (n > 0) {
        ak_console_write_raw(text, n);
    }
}

int ak_console_printf(const char *fmt, ...)
{
    sink_t out;
    va_list ap;

    out.len = 0;
    va_start(ap, fmt);
    int n = format(&out, fmt, ap);
    va_end(ap);

    if (out.len > 0) {
        ak_console_write_raw(out.buf, (unsigned)out.len);
    }
    return n;
}
