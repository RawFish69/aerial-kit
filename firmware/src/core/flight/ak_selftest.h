#ifndef AK_FLIGHT_SELFTEST_H
#define AK_FLIGHT_SELFTEST_H

/*
 * The checks that can run anywhere: on the board, on the bench, in CI.
 *
 * Same code in all three places, on purpose. A self-test that only exists in
 * the test build tests the test build. Reporting goes through a printf-shaped
 * function the caller supplies, so the core itself stays free of I/O.
 *
 * Values are reported as integers in milli-units (mrad, per-mille) because the
 * console formatter has no floating point on purpose: no soft-float printf in
 * a loop, and no libc.
 */

#include "ak_console.h" /* for ak_printf_fn */

/* Returns 0 when every check passed, 1 otherwise. Prints each one. */
int ak_flight_selftest(ak_printf_fn out);

#endif /* AK_FLIGHT_SELFTEST_H */
