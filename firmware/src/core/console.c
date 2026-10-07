#include "ak_console.h"

#include <stddef.h>
#include <stdarg.h>
#include <stdint.h>

/* Small bounded formatter. Output is assembled in a fixed buffer and handed to
 * the board sink in one call, so an interrupt that also prints cannot
 * interleave inside a line - as long as the line fits in the buffer, which is
 * the whole of what flush() below is about. */
#define AK_CONSOLE_LINE 160

typedef struct {
    char   buf[AK_CONSOLE_LINE];
    size_t len;     /* bytes in buf, not yet handed to the sink */
    size_t emitted; /* every byte this call has produced, flushed or not */
} sink_t;

/*
 * Hand the buffer to the board sink and start it again.
 *
 * This is what a line longer than the buffer needs, and it is here because the
 * sink used to drop what did not fit and `put()` did not say so. The blackbox's
 * column line was 317 characters over thirty-five columns (438 over forty-nine
 * since log version 4): a console reader got
 * its first 159, ending mid-name at `...torque_yaw,`, with the newline gone -
 * so the record that followed was glued onto the end of the header and the
 * sixteen remaining column names were named nowhere. Nothing else said they
 * were missing. Not the firmware, and not the suite: the log tests compare
 * `ak_log_write_header`'s lines against a capture taken from the same writer,
 * so they were comparing the truncation with itself.
 *
 * The price of the cure is that a line too long for the buffer reaches the sink
 * in several calls, and an interrupt that prints during one can now land inside
 * it. That is a real weakening of the guarantee above, and it is the right way
 * round: an interleaved line is still a line a person can read and a parser can
 * mostly recover, and a truncated one is a log whose rows do not match its
 * header. Lines that fit - which is every line the firmware prints but these
 * dumps - are still one call, so the guarantee holds where it was written for.
 */
static void flush(sink_t *out)
{
    if (out->len > 0u) {
        ak_console_write_raw(out->buf, (unsigned)out->len);
        out->len = 0u;
    }
}

static void put(sink_t *out, char c)
{
    /* One byte is kept back for the terminator `format()` writes, so the last
     * character of a field never lands out of bounds. */
    if (out->len + 1u >= sizeof out->buf) {
        flush(out);
    }
    out->buf[out->len++] = c;
    out->emitted++;
}

/*
 * Padding is measured from a byte count that outlives a flush, not from the
 * buffer's own length. A field whose padding is computed from `out->len` pads
 * to the wrong width the moment the buffer is emptied in the middle of it,
 * which is a second defect the first one was hiding rather than a thing to
 * discover later.
 */
static void pad_to(sink_t *out, size_t from, int width, char fill)
{
    int have = (int)(out->emitted - from);
    while (have < width) {
        put(out, fill);
        have++;
    }
}

static void put_padded(sink_t *out, const char *text, int width, int left)
{
    size_t start = out->emitted;
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

    size_t start = out->emitted;
    /* Right-justified is the default for a number with a width, with spaces
     * unless `0` asked for zeros: `%6d` used to ignore its width entirely. */
    if (!left) {
        pad_to(out, start, width - (int)n, zero ? '0' : ' ');
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
                uint32_t magnitude = (uint32_t)(-(int64_t)v);
                if (!left && !zero && width > 0) {
                    /* Spaces go before the sign, not between it and the
                     * digits: "    -7", not "-    7". */
                    int digits = 1;
                    for (uint32_t m = magnitude; m >= 10u; m /= 10u) {
                        digits++;
                    }
                    for (int pad = width - digits - 1; pad > 0; pad--) {
                        put(out, ' ');
                    }
                    width = 0;
                }
                put(out, '-');
                if (width > 0) {
                    width--;
                }
                put_number(out, magnitude, 10, 0, width, left, zero);
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
            return (int)out->emitted;
        default:
            put(out, '%');
            put(out, *p);
            break;
        }
    }
    out->buf[out->len] = '\0';
    return (int)out->emitted;
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
    out.emitted = 0;
    va_start(ap, fmt);
    (void)format(&out, fmt, ap);
    va_end(ap);

    /* Whatever is still in the buffer, and no more: everything past the first
     * AK_CONSOLE_LINE bytes has already gone to the sink a chunk at a time. */
    flush(&out);
    return (int)out.emitted;
}
