> Historical notes from the full development tree. Current public targets and commands are in the [firmware guide](../../docs/flight-controller-firmware.md). Host C tests and simulator harnesses referenced below are retained outside this public source tree.

# AerialKit - attribution

Everything AerialKit ships is either written here or listed below. The rule from
[00-decisions.md](00-decisions.md): reading another project's driver to
understand a register sequence, then writing our own, needs no entry; copying
text or a non-obvious algorithm does.

## Current state

**Nothing to list yet.** Every file in this repository is original work, except
the license text and the one file described further down:

| File | Source |
| --- | --- |
| `LICENSE` | GNU GPL v3, verbatim from <https://www.gnu.org/licenses/gpl-3.0.txt> (sha256 `3972dc97…`), fetched 2026-09-14 |

The reference families that informed the *decisions* - Betaflight, INAV,
ArduPilot and PX4 - are checked out under `fc-firmware-workspace/upstream/`. The
first two are on disk; the last two are not (their images are named by
`targets/f405-apm-quad/` and `targets/f405-px4-quad/` but the checkouts are not
here), and the *protocol* they speak is read from a third reference instead:
`upstream/pymavlink-2.4.49/` is pymavlink's generated MAVLink dialect, kept so
that `firmware/tools/mavlink.py` can be checked against the reference
implementation's own frames and constants rather than against its author's
memory of them ([27-configurator.md](27-configurator.md)). pymavlink is
LGPL-3.0-or-later (the wheel's `dist-info/METADATA`); it is a *reference and a
test oracle* - nothing in the firmware links against it, imports it or copies
from it, and what is kept is the pure-Python dialect rather than the library.
The
first four families' behaviour and layout informed the boot and flash decisions
in this repository (link at the base of flash, DFU the flat `.bin`, embed a
build stamp). Those are decisions, not copied code; the bench evidence for the
flash path is in `fc-firmware-workspace/projects/f405-bench/README.md`.

The per-driver *sources* are listed where each driver lives: which reference
each register map, coefficient layout and piece of arithmetic was read from,
and at which revision, with the tables in
[09-sensors.md](09-sensors.md) for the inertial and barometric parts. That is
the "read it, understand it, write it again" half of the rule above, which is
why it is not an entry here - and it is also why a claim like "no code was
copied" in a driver's own comment is worth being able to check.

[22-esp32-survey.md](22-esp32-survey.md) is the same kind of work for the second
target: it reads Betaflight's ESP32 platform, ArduPilot's ESP32 HAL,
Espressif's ESP-Drone and a project search, and records what each decides and
under which licence - GPL-3.0 for all three projects, MIT for most of the
community ones. Nothing was copied for it, and this page stays empty because of
that. [25-at32-survey.md](25-at32-survey.md) is the same work again for the
third target: the wing's board is an AT32F435, and the only reference
implementation for that chip is Artery's own library inside the pinned INAV
checkout, whose header says the code is Artery's copyrighted work. So it is
read for the clock and the register arithmetic, cited by file above, and never
copied - and the port it describes has not been written.

## When something is added

Record: the project, the file, the revision, what was taken, and the license.
Example shape, for a future entry:

```text
src/core/pid.c   Betaflight, src/main/flight/pid.c @ 6dbc4218 (GPLv3)
                 rate-loop structure and the D-term filter layout
```

## The one file that is copied, rather than reimplemented

`src/core/sensors/ak_imu_bmi270_config.c` is Bosch Sensortec's BMI270
configuration file, unchanged: eight thousand one hundred and ninety-two bytes
that the part's own microcontroller runs. Every other driver in this tree is a
register map reimplemented from a datasheet or read out of a reference
implementation; this one cannot be, because the thing being copied is not a
register map, it is a program whose source Bosch does not publish. The part's
data is invalid until it has been uploaded, so the driver is not usable
without it.

```text
src/core/sensors/ak_imu_bmi270_config.c
    Bosch Sensortec, BMI270_SensorAPI, bmi270.c (bmi270_config_file)
    https://github.com/boschsensortec/BMI270_SensorAPI, fetched 2026-09-15
    BSD-3-Clause - compatible with this repository's GPLv3
    changed: the name, an explicit length, and the header carrying this notice
    not changed: the bytes
```

The BSD-3-Clause notice travels with the bytes in that file, as the licence
requires. The driver around it, the upload sequence, the register map and the
status check are written here from the same register facts as everything else.

## Cross-checks against the reference families

A protocol fact is not a licence question, but it is worth writing down which
revision confirmed it, because the next agent will want to re-check it and
"somewhere in Betaflight" is not a place.

| Fact | Confirmed against |
| --- | --- |
| DShot frame layout: `(value << 1) | telemetry`, crc by nibble fold, `(value << 4) | crc`; throttle band 48..2047; 3D-mode forward minimum 1048 | Betaflight 2026.6.1 `src/main/drivers/dshot.c` `prepareDshotPacket` and `src/main/drivers/dshot.h` @ `6dbc4218`, read 2026-09-14 |
| DShot telemetry variants: the crc is inverted when bidirectional telemetry is in use, and the telemetry *response* frame is checked by folding the whole 16-bit word to 0xF | same file, `dshot_bitbang_decode.c` |
| CRSF RC frame: 16 channels packed little-endian at 11 bits each, counts 172..1811 with 992 centred | the CRSF wire format as documented; no reference implementation read for this one |
| STM32F405 write path: flat image at `0x08000000` through the ROM DFU bootloader | `fc-firmware-workspace/projects/f405-bench/README.md`, measured on real hardware |
| ICM-42688-P register map, FSR/ODR encoding, the AFSR workaround | Betaflight 2026.6.1 `src/main/drivers/accgyro/accgyro_spi_icm426xx.c` @ `6dbc4218` |
| ICM-42688-P who-am-i 0x47, ICM-42605 0x42 | Betaflight 2026.6.1 `src/main/drivers/accgyro/accgyro_mpu.h` |
| Gyro scale 16.4 counts per dps at ±2000 dps | INAV 9.1.0 `src/main/drivers/accgyro/accgyro_icm42605.c` @ `e519b69` |
| Quad-X motor mix: the coefficients, the row order (RR, FR, RL, FL) and the direction of the yaw column | Betaflight 2026.6.1 `src/main/flight/mixer_init.c` (`mixerQuadX`) @ `6dbc4218`, and the same signs in INAV 9.1.0 `src/main/target/ALIENFLIGHTF4/config.c` @ `e519b69`. Transcribed into `tests/mixer_reference.h`, and checked by `tests/test_mixer_parity.c` on every build |
| The other five frames - `quadx1234`, `quadp`, `y4`, `vtail4` and the tricopter's three motors | the same file and revision (`mixerQuadX1234`, `mixerQuadP`, `mixerY4`, `mixerVtail4`, `mixerTricopter`), row for row and sign for sign, with the tricopter's tail-tilt servo taken from the same project's servo mixer as a fourth row rather than a motor |
| DPS310 register map, reset sequence, coefficient packing, the sixteen-times oversampling scale and the compensation arithmetic (datasheet 4.9.1/4.9.2) | INAV 9.1.0 `src/main/drivers/barometer/barometer_dps310.c` @ `e519b69` |
| MPU-6000 register map, the initialisation order, the DLPF and full-scale encodings, the interrupt setup and the who-am-i value | Betaflight 2026.6.1 `src/main/drivers/accgyro/accgyro_spi_mpu6000.c` and `accgyro_mpu.h` @ `6dbc4218` |
| MPU-6500 and MPU-9250 who-am-i values (0x70 and 0x71, at the same register 0x75 the 6000 answers at), and that the reference writes the same CONFIG / GYRO_CONFIG / ACCEL_CONFIG / INT_PIN_CFG / INT_ENABLE bytes for a 6500 as for a 6000 with one DLPF table for the family | Betaflight 2026.6.1 `accgyro_mpu.h` (`MPU6500_WHO_AM_I_CONST`, `MPU9250_WHO_AM_I_CONST`), `accgyro_spi_mpu6500.c` (`mpu6500SpiDetect`, the who-am-i switch) and `accgyro_mpu6500.c` (`mpu6500GyroInit`) @ `6dbc4218`. Also checked: the LSM6DSV16X answers 0x70 as well, but at register **0x0F** rather than 0x75, so probing 0x75 cannot mistake one for the other |
| ICM-42605 who-am-i 0x42 | Betaflight 2026.6.1 `src/main/drivers/accgyro/accgyro_mpu.h` @ `6dbc4218` |
| LSM6DSO register map (who-am-i 0x0F = 0x6C, INT1_CTRL 0x0D, INT2_CTRL 0x0E, CTRL1_XL 0x10, CTRL2_G 0x11, CTRL3_C 0x12, CTRL4_C 0x13, CTRL6_C 0x15, CTRL9_XL 0x18, outputs from 0x22), the initialisation order, the 833 Hz / 6664 Hz ODR and 16 g / 2000 dps full-scale encodings, the BDU and IF_INC bits, the accel LPF1 choice, the gyro scale **0.070 dps/LSB**, and `acc_1G` 2048 counts at 16 g | Betaflight 2026.6.1 `src/main/drivers/accgyro/accgyro_spi_lsm6dso_init.c` (the whole sequence), `accgyro_spi_lsm6dso.c` and `accgyro_spi_lsm6dso.h` @ `6dbc4218`. The 0.070 is the one figure of the three below that names its source: that header cites datasheet section 4.1, symbol `G_So`, "70 mdps/LSB" |
| The same map a second time, independently, and the LSM6DSL's differing CTRL6_C mask (0x13 against the DSO's 0x17) - which is why the DSL and the DS3 (who-am-i 0x6A and 0x69, same register) are *not* covered by `ak_imu_lsm6dso.c` the way the MPU-6500 is covered by the MPU driver: a mask difference is a second initialisation, not a second name | INAV 9.1.0 `src/main/drivers/accgyro/accgyro_lsm6dxx.{c,h}` @ `e519b69` |
| The two places the references disagree and this driver departs from both or from one, each written up in `src/core/sensors/ak_imu_lsm6dso.c` rather than smoothed over: **(a)** INAV's one LSM6DXX driver uses `1.0f / 16.4f` dps-per-LSB for every part it detects, the LSM6DSO included - and 16.4 counts per dps is the **MPU-6000's** number, about 14 per cent away from Betaflight's sourced 0.070, so this file takes Betaflight's; **(b)** Betaflight's DSO file writes `INT2_CTRL = 0x02` under a comment reading "Disable interrupt pin 2", while INAV's header names 0x00 as the disable - bit 1 is the gyro's data-ready *enable*, so this file writes 0x00 | INAV 9.1.0 `accgyro_lsm6dxx.c` (the `1.0f/16.4f`) and `accgyro_lsm6dxx.h` (the interrupt values) @ `e519b69`; Betaflight 2026.6.1 `accgyro_spi_lsm6dso_init.c` (the `0.02` and its comment) @ `6dbc4218`. Both figures want a bench check; they are one constant each |
| The third departure, and the only one that is an adaptation rather than a choice between two sources: both references set `CTRL4_C` bit 2 `I2C_DISABLE`, which on a part reached **over I2C** switches off the wire the next write arrives on. `ak_imu_lsm6dso.c` clears it, and `tests/test_lsm6dso.c` starts the fake part with the bit already set and holds the driver to clearing it. Neither reference has ever run this part over I2C, so neither could have caught it - it is a first-class consequence of this board's IMU being a breakout on a cable rather than a chip on the SPI bus | the same two files: Betaflight 2026.6.1 `accgyro_spi_lsm6dso_init.c` (`CTRL4_C` write) @ `6dbc4218` and INAV 9.1.0 `accgyro_lsm6dxx.c` @ `e519b69` |
| The output-register order and byte order for a 12-byte burst: gyro x/y/z at 0x22..0x27, accel x/y/z at 0x28..0x2D, both **little endian** - the opposite of the MPU-6000's arrangement on both counts, which is why the 12-byte read is the thing the host test pins hardest | the same register map in both references (the `0x22` burst base and the `int16_t` little-endian decode) @ `6dbc4218` and `e519b69` |
| UBX framing: sync pair, little-endian length, Fletcher-style checksum | a captured frame in INAV 9.1.0 `src/test/unit/gps_ublox_unittest.cc` @ `e519b69`, used as a test vector |
| NAV-PVT field names, units, meaning **and byte offsets** - the last of these checked by `offsetof()` against a copy of that struct in `tests/ubx_reference.h`, which is also what the simulator builds its frames from | INAV 9.1.0 `src/main/io/gps_ublox.h` (`ubx_nav_pvt`) @ `e519b69` |
| UBX-CFG-VALSET frame layout and the version-1 header | the captured frame in INAV 9.1.0 `src/test/unit/gps_ublox_unittest.cc`, and its `ubx_config_data_header_v1_t` |
| Configuration keys: message rates, NMEA off, rate keys | INAV 9.1.0 `src/main/io/gps_ublox.h` @ `e519b69` |
| Battery thresholds per cell: warn 3.50 V, critical 3.30 V; cell detect 4.30 V; auto-detected counts capped at 8; the count taken when a pack is connected rather than per sample | Betaflight 2026.6.1 `src/main/sensors/battery.h` (`VBAT_CELL_VOLTAGE_DEFAULT_MIN` 330, `VBAT_CELL_VOLTAGE_DEFAULT_MAX` 430, `MAX_AUTO_DETECT_CELL_COUNT` 8), `src/main/config/config.c` (warning 350, full 410) and `src/main/sensors/battery.c` (`autoDetectCellCount`, `batteryUpdatePresence`) @ `6dbc4218`; INAV 9.1.0 `src/main/fc/settings.yaml` (`vbat_warning_cell_voltage` 350, `vbat_min_cell_voltage` 330, `vbat_cell_detect_voltage` 425) @ `e519b69` |
| SBUS frame layout: the 0x0F header and 0x00 footer, sixteen 11-bit channels packed little-endian at bit 11*n in twenty-two bytes, the 25-byte frame, 100000 baud 8E2, and the flags byte's bit 2 signal-loss / bit 3 failsafe | Betaflight 2026.6.1 `src/main/rx/sbus.c` (`SBUS_FRAME_BEGIN_BYTE`, `SBUS_PORT_OPTIONS`, the inter-byte gap) and `src/main/rx/sbus_channels.h` (`sbusChannels_t`, `SBUS_FLAG_SIGNAL_LOSS`, `SBUS_FLAG_FAILSAFE_ACTIVE`) @ `6dbc4218` |
| A receiver whose own failsafe is active or which has dropped frames does not refresh the link's freshness timestamp | the same file, `sbusFrameStatus()`: `lastRcFrameTimeUs` is updated only when the frame is neither `RX_FRAME_FAILSAFE` nor `RX_FRAME_DROPPED` |
| The STM32F4's USART cannot invert its pins, so SBUS needs an inverter in hardware | RM0090 section 30: there is no TXRXINV bit in USART_CR2 on this part (the F0/F3/F7/G0/G4/H7/L4 families have one). Checked by reading the CR2 bit definitions rather than assumed from the F7 |
| What an ESP32 flight target does about its link, as a survey before writing AerialKit's | Espressif's ESP-Drone (`github.com/espressif/esp-drone`, **GPLv3**) `components/drivers/general/wifi/wifi_esp32.c`: `esp_netif_create_default_wifi_ap()` + `WIFI_MODE_AP`, i.e. the drone *is* the access point. Read for the approach, deliberately not copied: AerialKit joins the network its parameters name and only carries an access point when there is nothing to join, because an aircraft whose link is an island is one the companion on the bench cannot reach at the same time, and because a radio in AP mode is a radio inviting connections - which is why this one **refuses to come up open**. **No code was taken**, and nothing else in that repository was used |
| I2C timing arithmetic: CCR for standard mode and for the two fast-mode duties, the minimum periods, the FREQ field, and TRISE for 1000 ns and 300 ns of rise time | RM0090 27.6.8, 27.6.9 and 27.6.10 read directly. Worth saying why the reference implementations were *not* the source: INAV's F4 I2C (`src/main/drivers/bus_i2c_stm32f40x.c` @ `e519b69`) calls ST's standard peripheral library, which computes all of this inside `I2C_Init()` and does not show it, and Betaflight no longer carries an F4 I2C driver at all. The manual is the authority the library itself follows, and the expected numbers in `tests/test_i2c.c` were derived from it by hand rather than from this code's output |
| The master-receiver procedure, including where the NACK goes for a one, two and many-byte read, and what POS is for | RM0090 27.3.3 and 27.6.1. The procedure is the manual's; no code was transcribed, and the sequences are written out in `src/arch/stm32f405/i2c.c` with the case each one is for |
| BMI270 register map, who-am-i 0x24, the configuration-upload sequence (PWR_CONF off, INIT_CTRL 0, the file to INIT_DATA, INIT_CTRL 1), the soft reset, and the ACC_CONF / ACC_RANGE / GYRO_CONF / GYRO_RANGE / INT map encodings | INAV 9.1.0 `src/main/drivers/accgyro/accgyro_bmi270.c` @ `e519b69`, which is a register-level driver and therefore shows the sequence. The eight-kilobyte configuration file is **not** from there - INAV is GPLv3 and the file is Bosch's - it is from Bosch's own BSD-3-Clause SensorAPI, recorded below in full because it is the first copied file in this repository |
| What makes a GPS fix worth navigating on, and the floor that decides it: `fixType == GPS_FIX_3D && numSat >= gpsMinSats`, with `gps_min_sats` defaulting to 6 and constrained to 5..10, and INAV's own reason for it ("some GPS receivers appeared to be very inaccurate with low satellite count") | INAV 9.1.0 `src/main/fc/settings.yaml` (`gps_min_sats`, default 6, min 5, max 10) and `src/main/io/gps.c` (`sensorHasFix`, `gpsSol.numSat >= gpsConfig()->gpsMinSats`) @ `e519b69`. AerialKit had accepted any fresh 2D-or-better fix the receiver called ok; the rule and the default are INAV's, the 3D requirement is also theirs and matters here for a second reason of our own (a 2D fix has no height, and the fused altitude's absolute reference comes from the first fix seen on the ground) |
| TOF10120: the I2C address (0x52), the "sending method" register that has to be written before the part answers, the address register read back as the "is anything there" answer, the two-byte big-endian distance at register 0x00, the rule that at or past 2000 mm there is nothing to report, and the 100 ms task period | INAV 9.1.0 `src/main/drivers/rangefinder/rangefinder_tof10120_i2c.c` @ `e519b69`. The part's 30 mm blind-zone minimum is **not** from there - INAV's driver has no minimum - and is marked as the datasheet's number in `src/core/sensors/ak_rangefinder_tof10120.c` |
| The gates an arm request has to pass, and the one that was missing: a switch, a throttle below a threshold, and an attitude **within `small_angle` of upright** (default 25 degrees, 180 disables it, compared as the cosine of the tilt), plus a named reason for each refusal | Betaflight 2026.6.1 `src/main/fc/core.c` (`ARMING_DISABLED_THROTTLE`, `ARMING_DISABLED_ANGLE`, `isUpright()`) with `src/main/flight/imu.c` (`DEFAULT_SMALL_ANGLE 25`, `smallAngleCosZ`) @ `6dbc4218`; INAV 9.1.0 `src/main/fc/fc_core.c` (`ARMING_DISABLED_NOT_LEVEL`, `_RC_LINK`, `_THROTTLE`, `_SENSORS_CALIBRATING`) @ `e519b69`. The gates and the 25 degrees are theirs; the cosine comparison is the same arithmetic, and `ak_flight_arm_check()` is AerialKit's own list because it had to answer the console as well as the arming path |
| A fence with an altitude **ceiling** as well as a floor and a radius, and the vertical margin a navigator keeps below the ceiling so it does not sit on the line its own trigger fires at | INAV 9.1.0 `src/main/navigation/navigation.h` (`geoZoneConfig_t.maxAltitude` / `.minAltitude`, a zone's altitude band) and `src/main/navigation/navigation_geozone.c` (a position is in a zone only between them) @ `e519b69`; the margin is that project's `geozone_safe_altitude_distance`, "vertical distance that must be maintained to the upper and lower limits of the zone", default 1000 cm = ten metres (`src/main/fc/settings.yaml`) |
| A hand launch as a mode: the throttle held during it, the climb *attitude*, the timeout, and the stick travel that aborts it | INAV 9.1.0 `src/main/fc/settings.yaml` (`nav_fw_launch_thr` 1700 of 1000..2000 = 70 per cent; `nav_fw_launch_climb_angle` 18, "attitude of model, not climb slope"; `nav_fw_launch_timeout` 5000 ms; `nav_fw_launch_land_abort_deadband` 100 of 820 counts) and `src/main/navigation/navigation_fw_launch.c` (the state machine those numbers drive) @ `e519b69`. The four numbers are theirs; the state machine is not - AerialKit's launch is the half a switch can start, and `ak_launch.h` says which half is missing and why (the simulator's wing has no throw in it, so a detection threshold would be a number nobody could measure) |
| The **AT32F435's clock**: the memory map (the APB1/APB2/AHB1 windows and each peripheral's base), the clock unit's layout (`ctrl`, `pllcfg` with `ms`/`ns`/`fr` and the source bit, `cfg` with the selector, its status field and the three bus dividers), the flash controller's own clock divider (`divr` and its in-force status), and the regulator's voltage register - plus the sequence: 1.3 V, then the divider, then the crystal, then the PLL, then the switch | Artery's own CMSIS header and standard-peripheral library, inside the pinned INAV checkout: `lib/main/AT32F43x/Drivers/CMSIS/Device/ST/AT32F43x/at32f435_437.h` and `lib/main/AT32F43x/Drivers/CMSIS/Device/ST/AT32F43x/at32f435_437_clock.c` (`system_clock_config()`), with the bit layouts in `lib/main/AT32F43x/Drivers/AT32F43x_StdPeriph_Driver/inc/at32f435_437_{crm,flash,pwc}.h` @ `e519b69`. **Read, never copied**: Artery's header says its contents are Artery's copyrighted work, so `src/arch/at32f435/regs.h` restates the part rather than reproducing the file, and the port's own numbers are the board's declared 8 MHz crystal with ms 1, ns 72, fr 1 -> 288 MHz *derived* from it (the `(HEXT/ms)*ns/fr` arithmetic and Artery's ranges for it are in `clk.c`, and `tests/test_arch_at32.c` runs the arithmetic, the derivation and the register writes as three separate checks) |
| The **AT32F435's pin functions**: which pin carries which timer channel, UART, SPI or I2C, and at which function number - every mux number in `src/boards/AERIALKIT_GHF435/board.h` and `src/arch/at32f435/output.c` | INAV 9.1.0 `src/main/drivers/timer_def_at32f43x.h` @ `e519b69`, which is the table that target's drivers are built from (`D(2, 4)` = mux 2 on TMR4, and the entries name the pin and the channel), plus the per-bus tables in `bus_spi_at32f43x.c` (SPI1/SPI2 mux 5, SPI3 mux 6), `bus_i2c_at32f43x.c` (I2C2 mux 4) and `serial_uart_at32f43x.c` (USART1/3 mux 7). The receiver's two different numbers are the wing target's own `UART2_RX_AF 6` / `UART2_TX_AF 8` in `src/main/target/TWINWINGS_GHF435V2/target.h`. **The table was in the workspace the whole time**: the port's first pass searched for the calls that configure pins and missed the table they are built from, which cost a session and is written down in the original private bench notes |
| The AT32F435's **interrupt numbers**: USART1/2/3 at 37, 38 and 39, and DMA1's channels 1 and 2 at 56 and 57 - which is why the two parts need two vector tables, and why `scripts/image-facts/at32f435rg.txt` says 74 words where the F405's says 56 | Artery's device header, `lib/main/AT32F43x/Drivers/CMSIS/Device/ST/AT32F43x/at32f435_437.h` (`IRQn_Type`) @ `e519b69` |
| The AT32F435's **flash and RAM geometry**: 1 MB in two 512 KB banks, 2 KB erase pages, and the two RAM regions (64 KB at `0x10000000`, 128 KB at `0x20010000`) that the reference build maps - and which the 64 KB below `0x20010000` is *not* part of, because the reference does not map it either | INAV 9.1.0 `src/main/target/link/at32_flash_f43xG.ld` @ `e519b69`, read for the regions, and `config_streamer_at32f43x.c` for the 2 KB page size on the RG part. AerialKit's own `linker/at32f435rg.ld` reproduces the regions and says in the file that the lower 64 KB is left alone because the only implementation of this chip to read leaves it alone |
| The AT32's DShot path: the compare value reaching the register through **one DMA channel per motor** via the request multiplexer, and the DMAMUX request number for TMR4 channel 1 | INAV 9.1.0 `src/main/drivers/timer_impl_stdperiph_at32.c` and `dma_at32f43x.c` @ `e519b69`. The alternative the F405 port uses - the timer's own burst registers - exists on this part (`dmactrl`/`dmadt`) but has no reference here to check its length encoding against, which is why `src/arch/at32f435/output.c` says so rather than guessing |
| The AT32's **I2C master sequences**: the address, direction and byte count as one ctrl2 write; which flag to wait on before each step (`tdis` before a byte is sent, `tdc` before the repeated start of a read, `rdbf` per byte received, `stopf` at the end); the auto-stop that ends a transfer when the count runs out; and the mask that clears the fields a new transfer owns | INAV's copy of Artery's blocking master routines, `src/main/drivers/i2c_application.c` (`i2c_memory_write`, `i2c_memory_read`, `i2c_transmit_set`, `i2c_data_send`/`i2c_data_receive`, `i2c_wait_flag`), with the register layout in `lib/main/AT32F43x/Drivers/AT32F43x_StdPeriph_Driver/inc/at32f435_437_i2c.h` @ `e519b69`. **Read, never copied**: the sequences are reimplemented as AerialKit's own polled driver (`src/arch/at32f435/i2c.c`), and one number in it was read from the *comment* beside the reference's mask rather than from the mask itself - which the modelled bus caught on its first run, and [20-i2c.md](20-i2c.md) writes up |
| The AT32's **USB device bring-up**: that the core comes out of reset in power-down and which bit takes it out (`gccfg.pwrdown`), that the phy clock is gated (`pcgcctl.stoppclk`), the turn-around time for a 48 MHz clock (`usbtrdtim` = 5, which the reference writes when enumeration completes), that the core has no PHYSEL field, that the USB clock is taken from the PLL and divided by six at 288 MHz, the controller's own clock enable, and that the two data pins are PA11/PA12 on function 10 | Artery's own USB device library and configuration inside the pinned INAV checkout: `lib/main/AT32F43x/Drivers/AT32F43x_StdPeriph_Driver/src/at32f435_437_usb.c` (`usb_global_init`, `usb_open_phy_clk`, `usb_global_set_mode`), `Middlewares/AT/AT32_USB_Device_Library/Core/Src/usbd_core.c` and `usbd_int.c` (`usbtrdtim` on enumeration done), `.../Core/Inc/usb_conf.h` (`OTG_PIN_MUX` = `GPIO_MUX_10`, PA11/PA12), and the clock from INAV's `src/main/drivers/serial_usb_vcp_at32f43x.c` (`usb_clock48m_select`, whose 288 MHz case is `CRM_USB_DIV_6`) @ `e519b69`. The register layout and every bit position were checked against `inc/at32f435_437_usb.h` and are the same as the STM32F405's for the parts the driver touches - which is why `src/arch/at32f435/usb.c` is the F405's driver with this part's clock, pins and three registers |
| The **ROM bootloader's address and how to enter it**: 0x1FFF0000, and the jump that starts it - read the two words there (a stack pointer and a reset handler), set the one and branch to the other | INAV 9.1.0 `src/main/drivers/system_stm32f4xx.c` and `system_at32f43x.c` (`systemBootloaderAddress()`, whose comment for this part reads "AT32 DIAGRAM2-1 AT32F435/437 DFU BOOTLOADER ADDR") with the sequence in `src/main/drivers/system.c` (`bootloaderVector`, `__set_MSP`, `resetHandler`) @ `e519b69`. Both parts answer 0x1FFF0000 and that is a coincidence of two vendors rather than one fact shared, so each port carries its own constant - `src/arch/{stm32f405,at32f435}/regs.h` - with its own citation, and `tests/test_regs.c` pins the F405's while `tests/test_arch_at32.c` pins the AT32's. The jump itself is assembly on both and cannot run in a host test; what the host checks is the address and that the board implements the hand-over |

None of these transferred code. Each is a statement about a wire format or a
memory layout, checked against the thing that implements it.
