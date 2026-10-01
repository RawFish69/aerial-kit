#ifndef AK_TESTS_H
#define AK_TESTS_H

/* Test groups that live in their own files, all linked into one binary with
 * tests/test_aerialkit.c's main(). */

void test_text(void);
void test_params_and_cli(void);
void test_dshot_timing(void);
void test_register_encodings(void);
void test_receive_path(void);
void test_crsf_telemetry(void);
void test_imu(void);
void test_align_and_calibration(void);
void test_mixer_authority(void);
void test_blackbox(void);
void test_gps(void);
void test_navigation(void);
void test_gps_config(void);
void test_protocol(void);
void test_rc_calibration(void);
void test_accel_calibration(void);
void test_mixer_parity(void);
void test_barometer(void);
void test_altitude(void);
void test_launch(void);
void test_rangefinder(void);
void test_gyro_arm_calibration(void);
void test_battery(void);
void test_sbus(void);
void test_i2c_timing(void);
void test_bmi270(void);
void test_lsm6dso(void);
void test_bmp280(void);
void test_bmp388(void);
void test_arch(void);
void test_arch_at32(void);
void test_arch_esp32_output(void);
void test_arch_esp32_bus(void);
void test_arch_esp32_uart(void);
void test_arch_esp32_adc(void);
void test_arch_esp32_net(void);
void test_fault(void);
void test_arch_esp32_fault(void);
void test_board_f405(void);
void test_board_ghf435(void);
/* The Feather is the second board file on the *same* part as the WeAct one, so
 * this is the first test in the tree that runs two boards on one arch layer: the
 * arch objects are shared and only the board file differs, which is what
 * `AK_HOST_FEATHER` in that board's header is for. See tests/test_board_feather.c
 * for the six facts that differ and the three that were wrong until it existed. */
void test_board_feather(void);
void test_boot(void);

/* The register pages, for the tests that run against them. tests/test_arch.c
 * maps the F405's peripheral, private, USB and flash pages, and the *three*
 * board tests run against the same four, so all four establish the mapping they
 * need here rather than depending on the order the tests happen to run in.
 * The fault record needs only the private page - its status registers live in
 * it - and says so, because a caller that had the whole model mapped would be
 * told yes by a check for one page of it. */
int ak_test_registers_mapped(void);
int ak_test_map_registers(void);
int ak_test_system_control_mapped(void);
int ak_test_map_system_control(void);

/*
 * A part a few hundred microseconds after reset, with its crystal up (or not):
 * the internal oscillator running, the PLL locked, the flash controller told
 * its divider, and the "which clock am I on" field already reporting the source
 * the clock code is about to ask for - that field is read-only on the part, so
 * a model can only answer it by saying the right thing in advance.
 *
 * tests/test_arch.c and tests/test_arch_at32.c each need this before they run
 * the port's clock code, and so do the two board tests now (the banner's clock
 * line is built from it). One definition each, so no two tests can disagree
 * about what a working part answers.
 */
void ak_test_f405_crystal(int up);
void ak_test_at32_clock_up(void);
void ak_test_at32_no_crystal(void);

/* Assertion helpers shared by the test files. */
void expect(const char *name, int passed);
int  expect_failures(void);

/*
 * A flight core with a board attached, for the tests that fly one.
 *
 * Arming compares what the selected airframe's mix needs with what the board
 * says it drives (`ak_flight_set_board_outputs`), and a core nobody has told
 * refuses to arm - which is the safe direction in the firmware and an
 * inconvenient one in a test that is not about outputs at all. Four motors and
 * two servos is the WeAct board's answer and enough for either airframe's mix.
 * It is *not* what every board here answers with and this comment used to say
 * it was: the Feather F405 drives two motors and two servos, so a test that
 * assumed four everywhere would be assuming the WeAct board's header. The
 * number below is this helper's own choice of a capable board, which is a
 * different claim and the honest one.
 *
 * A test that is *about* that gate calls `ak_flight_init` and the setter
 * directly, which is how the mismatch cases are written.
 */
#include "ak_flight.h"

static inline ak_flight_t *flight_with_outputs(ak_flight_t *flight,
                                               const ak_mixer_t *mixer)
{
    ak_flight_init(flight, mixer);
    ak_flight_set_board_outputs(flight, 4u, 2u);
    return flight;
}

#endif /* AK_TESTS_H */
