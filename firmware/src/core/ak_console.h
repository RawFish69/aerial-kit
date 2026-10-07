#ifndef AK_CORE_AK_CONSOLE_H
#define AK_CORE_AK_CONSOLE_H

/*
 * Console output. The formatter lives in core/console.c and is portable; the
 * byte sink is ak_console_write_raw(), which a board's UART provides.
 */

/* Anything that wants to print without owning a console takes this shape:
 * ak_console_printf on the board, a capturing shim in the host tests, and
 * nothing at all where there is no output to be had. */
typedef int (*ak_printf_fn)(const char *fmt, ...);

void ak_console_write_raw(const char *data, unsigned len);

void ak_console_write(const char *text);

/* Minimal printf: %s %c %d %i %u %x %X %p %%, with '-' and '0' flags and a
 * field width. No floats, no length modifiers - the point is to have no
 * dependency on libc at all, not to reimplement printf.
 *
 * Returns how many characters the call produced. A line longer than the
 * formatter's own buffer is handed to the sink in several pieces rather than
 * cut short; see console.c for why, and for what that costs. */
int ak_console_printf(const char *fmt, ...);

#endif /* AK_CORE_AK_CONSOLE_H */
