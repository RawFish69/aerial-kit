#ifndef AK_SENSORS_AK_IMU_BMI270_CONFIG_H
#define AK_SENSORS_AK_IMU_BMI270_CONFIG_H

#include <stdint.h>

/*
 * Bosch's configuration file for the BMI270.
 *
 * Declared here rather than in the driver so that the test can hold the bytes
 * the driver uploads against the bytes that were published - which is the only
 * way to check an eight-kilobyte constant that a copy could have truncated
 * without anything failing to compile.
 *
 * The file it lives in carries the BSD-3-Clause notice and where it came from;
 * it is the one thing in this repository that is copied rather than written,
 * and it says so in its first paragraph.
 */

#define AK_BMI270_CONFIG_SIZE 8192u

extern const uint8_t ak_bmi270_config_file[AK_BMI270_CONFIG_SIZE];

#endif /* AK_SENSORS_AK_IMU_BMI270_CONFIG_H */
