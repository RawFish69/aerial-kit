#ifndef AK_HOST_IDF_SDKCONFIG_H
#define AK_HOST_IDF_SDKCONFIG_H

/*
 * The one build-configuration value `src/arch/esp32/` reads: the CPU clock the
 * image was configured for. IDF generates this header from `sdkconfig` at
 * configure time; the host build has no sdkconfig, so it stands in with IDF's
 * own default for this chip, which is what the port's `sdkconfig.defaults`
 * leaves it at. If that file ever sets the frequency, this number has to move
 * with it - which is the sort of coupling `make -C ports/esp32` catches,
 * because the ESP32 build uses the generated header rather than this one.
 */

#define CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ 160

/* And which chip the host build is standing in for. The port has per-chip
 * branches - the ADC's calibration scheme is one, and the ESP32's is the
 * *line*-fitting one every other chip in the family lacks - so the host has to
 * say which of them it is compiling. It is the original ESP32 because that is
 * what this machine's QEMU can boot and what the idf-stub's driver models were
 * written for; the other chips' branches are compiled by their own builds
 * (`scripts/esp32-families.sh`). */
#define CONFIG_IDF_TARGET_ESP32 1

#endif /* AK_HOST_IDF_SDKCONFIG_H */
