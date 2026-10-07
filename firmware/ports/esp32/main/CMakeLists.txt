# The firmware, from the repository this project lives in.
#
# Listed rather than globbed. CMake evaluates a glob when it configures, so a
# new file in a globbed directory is not in the build until something forces a
# re-configure - which is exactly what happened the first time this port was
# built, with a fault handler that the linker could not find. The list is
# explicit for the same reason the ARM Makefile's wildcards are fine: make
# re-evaluates them every run, CMake does not.
#
# core/time.c is the one deliberate exclusion: it busy-waits, which starves
# FreeRTOS, so the ESP32 port supplies its own clock and delay instead.

set(AK_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../..")

# This public component has one ESP32 board profile.
if(DEFINED AK_BOARD AND NOT AK_BOARD STREQUAL "ESP32DEV")
    message(FATAL_ERROR "Only AK_BOARD=ESP32DEV is included in this repository")
endif()
set(AK_BOARD "ESP32DEV")

set(AK_SOURCES
    # Portable runtime; core/time.c is replaced by the FreeRTOS port.
    "${AK_ROOT}/src/core/ak_boot.c"
    "${AK_ROOT}/src/core/ak_cli.c"
    "${AK_ROOT}/src/core/ak_console_link.c"
    "${AK_ROOT}/src/core/ak_flashlog.c"
    "${AK_ROOT}/src/core/ak_params.c"
    "${AK_ROOT}/src/core/ak_perf.c"
    "${AK_ROOT}/src/core/ak_proto.c"
    "${AK_ROOT}/src/core/ak_ring.c"
    "${AK_ROOT}/src/core/ak_sched.c"
    "${AK_ROOT}/src/core/ak_text.c"
    "${AK_ROOT}/src/core/console.c"
    "${AK_ROOT}/src/core/filter/ak_dyn_notch.c"
    "${AK_ROOT}/src/core/filter/ak_fft.c"
    "${AK_ROOT}/src/core/filter/ak_filter.c"
    "${AK_ROOT}/src/core/filter/ak_rpm_filter.c"
    "${AK_ROOT}/src/core/flight/ak_altitude.c"
    "${AK_ROOT}/src/core/flight/ak_battery.c"
    "${AK_ROOT}/src/core/flight/ak_crsf.c"
    "${AK_ROOT}/src/core/flight/ak_crsf_telemetry.c"
    "${AK_ROOT}/src/core/flight/ak_dshot_capture.c"
    "${AK_ROOT}/src/core/flight/ak_dshot_edt.c"
    "${AK_ROOT}/src/core/flight/ak_dshot_gcr.c"
    "${AK_ROOT}/src/core/flight/ak_dshot_timing.c"
    "${AK_ROOT}/src/core/flight/ak_estimator.c"
    "${AK_ROOT}/src/core/flight/ak_flight.c"
    "${AK_ROOT}/src/core/flight/ak_launch.c"
    "${AK_ROOT}/src/core/flight/ak_log.c"
    "${AK_ROOT}/src/core/flight/ak_logenc.c"
    "${AK_ROOT}/src/core/flight/ak_math.c"
    "${AK_ROOT}/src/core/flight/ak_mixer.c"
    "${AK_ROOT}/src/core/flight/ak_motor_health.c"
    "${AK_ROOT}/src/core/flight/ak_nav.c"
    "${AK_ROOT}/src/core/flight/ak_output.c"
    "${AK_ROOT}/src/core/flight/ak_pid.c"
    "${AK_ROOT}/src/core/flight/ak_rc.c"
    "${AK_ROOT}/src/core/flight/ak_rc_receiver.c"
    "${AK_ROOT}/src/core/flight/ak_sbus.c"
    "${AK_ROOT}/src/core/flight/ak_selftest.c"
    "${AK_ROOT}/src/core/main.c"
    "${AK_ROOT}/src/core/mem.c"
    "${AK_ROOT}/src/core/sensors/ak_accel_cal.c"
    "${AK_ROOT}/src/core/sensors/ak_align.c"
    "${AK_ROOT}/src/core/sensors/ak_baro.c"
    "${AK_ROOT}/src/core/sensors/ak_baro_bmp280.c"
    "${AK_ROOT}/src/core/sensors/ak_baro_bmp388.c"
    "${AK_ROOT}/src/core/sensors/ak_baro_dps310.c"
    "${AK_ROOT}/src/core/sensors/ak_gps.c"
    "${AK_ROOT}/src/core/sensors/ak_gps_config.c"
    "${AK_ROOT}/src/core/sensors/ak_gyro_cal.c"
    "${AK_ROOT}/src/core/sensors/ak_imu.c"
    "${AK_ROOT}/src/core/sensors/ak_imu_bmi270.c"
    "${AK_ROOT}/src/core/sensors/ak_imu_bmi270_config.c"
    "${AK_ROOT}/src/core/sensors/ak_imu_bno055.c"
    "${AK_ROOT}/src/core/sensors/ak_imu_icm42688.c"
    "${AK_ROOT}/src/core/sensors/ak_imu_lsm6dso.c"
    "${AK_ROOT}/src/core/sensors/ak_imu_mpu6000.c"
    "${AK_ROOT}/src/core/sensors/ak_rangefinder.c"
    "${AK_ROOT}/src/core/sensors/ak_rangefinder_tof10120.c"
    # the ESP32 port
    "${AK_ROOT}/src/arch/esp32/console.c"
    "${AK_ROOT}/src/arch/esp32/adc.c"
    "${AK_ROOT}/src/arch/esp32/fault.c"
    "${AK_ROOT}/src/arch/esp32/flashlog.c"
    "${AK_ROOT}/src/arch/esp32/i2c.c"
    "${AK_ROOT}/src/arch/esp32/led.c"
    "${AK_ROOT}/src/arch/esp32/net.c"
    "${AK_ROOT}/src/arch/esp32/nvs.c"
    "${AK_ROOT}/src/arch/esp32/output.c"
    "${AK_ROOT}/src/arch/esp32/spi.c"
    "${AK_ROOT}/src/arch/esp32/system.c"
    "${AK_ROOT}/src/arch/esp32/time.c"
    "${AK_ROOT}/src/arch/esp32/uart.c"
    # the board
    "${AK_ROOT}/src/boards/${AK_BOARD}/board.c"
)

idf_component_register(
    SRCS ${AK_SOURCES} "app_main.c"
    INCLUDE_DIRS
        "${AK_ROOT}/src/core"
        "${AK_ROOT}/src/core/flight"
        "${AK_ROOT}/src/core/filter"
        "${AK_ROOT}/src/core/sensors"
        "${AK_ROOT}/src/arch/esp32"
        "${AK_ROOT}/src/boards/${AK_BOARD}"
)

execute_process(COMMAND git -C "${AK_ROOT}" describe --always --dirty
                OUTPUT_VARIABLE AK_REV OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET)
if(NOT AK_REV)
    set(AK_REV "unknown")
endif()
# No spaces, unlike the ARM build's stamp. A -D define cannot carry spaces
# without a layer of quoting that exists only to be got wrong, and an
# unambiguous timestamp is worth more than matching the other target's format.
string(TIMESTAMP AK_STAMP "%Y-%m-%dT%H:%M:%S")

target_compile_definitions(${COMPONENT_LIB} PRIVATE
    AK_BOARD=${AK_BOARD}
    AK_PRODUCT=aerialkit-esp32
    AK_REV=${AK_REV}
    AK_STAMP_VALUE=${AK_STAMP}
    # The long log lives in the ESP32's eight kilobytes of RTC memory rather
    # than in RAM, and at 40 bytes a record that is a shorter ring than the
    # F405's fifteen kilobytes allow. The log carries its capacity and refuses
    # to read a ring built for a different one, so the two targets cannot
    # misread each other's.
    AK_LOG_CAPACITY=64
)

# And whatever a *variant* of this board needs, which is how the half of the
# board file a bare devkit never compiles gets compiled at all:
#
#   idf.py -D AK_VARIANT_DEFINES="-DAK_BOARD_IMU_FITTED=1;-DAK_BOARD_VBAT_FITTED=1" build
#
# `scripts/esp32-proto.sh` passes the radio plus both parts fitted, which is the
# board a real devkit becomes. IDF wants a CMake *list* rather than a string,
# hence the semicolon; the quotes keep it one argument at the shell.
if(DEFINED AK_VARIANT_DEFINES)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ${AK_VARIANT_DEFINES})
endif()
