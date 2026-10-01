#ifndef AK_CORE_AK_TEXT_H
#define AK_CORE_AK_TEXT_H

#include <stdint.h>

/*
 * Text handling for a firmware that links no libc.
 *
 * The console formatter has no floating point on purpose, so values that
 * matter - gains, limits, angles - are parsed and printed here, as integers
 * and fixed point. Nothing in this file calls <string.h>: the ARM image would
 * link, since core/mem.c provides those functions, but the host tests would
 * then be testing libc's string functions instead of ours.
 */

unsigned ak_strlen(const char *text);
void     ak_strlcpy(char *dst, const char *src, unsigned len);
int      ak_str_eq(const char *a, const char *b);
int      ak_str_starts_with(const char *text, const char *prefix);
const char *ak_skip_spaces(const char *text);

/* Strict: the whole string has to parse, or it returns 0. A typo in a gain
 * must not be read as the number that happens to come first. */
int ak_parse_int(const char *text, int32_t *out);
int ak_parse_uint(const char *text, uint32_t *out);
int ak_parse_float(const char *text, float *out);

/* Fixed-point printing: ak_format_fixed(-1.5f, 3, buf, len) writes "-1.500".
 * Rounds, clamps to the buffer, and always terminates. Returns the length
 * written, or 0 if the buffer was too small for even a "-". */
unsigned ak_format_fixed(float value, unsigned decimals, char *buf, unsigned len);

/* Decimal, unsigned, with a caller-chosen minimum width (zero padded). */
unsigned ak_format_uint(uint32_t value, unsigned width, char *buf, unsigned len);

#endif /* AK_CORE_AK_TEXT_H */
