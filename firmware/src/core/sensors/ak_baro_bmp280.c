#include "ak_baro.h"

/*
 * Bosch BMP280, and the BME280 that is the same part with a humidity sensor
 * bolted on (whose registers this driver never touches).
 *
 * The other barometer a board this size carries, and the one the wing's own
 * flight controller lists next to the DPS310 - so between them the two drivers
 * here cover the barometer options of the aircraft this firmware is meant to
 * fly, and the probe decides which one answered.
 *
 * The arithmetic is the datasheet's own integer compensation, which is worth
 * saying out loud because it is not the DPS310's shape at all: a BMP280's
 * pressure comes out of a formula with 64-bit intermediates, shifts of 33 and
 * 47 bits, and a fixed-point result in Q24.8. The 64-bit part is why the
 * intermediates here are `int64_t` rather than `int32_t` - the reference
 * implementation uses the same widths for the same reason, and the host test
 * holds the result against values computed separately from the datasheet.
 *
 * Register map, chip ids, the oversampling and filter encodings, the data
 * frame and the compensation arithmetic: INAV 9.1.0
 * src/main/drivers/barometer/barometer_bmp280.c and .h @ e519b69, which follow
 * the datasheet's sections 3.11 and 4.3. No code was copied; the attribution
 * table has the entry.
 */

#define BMP_REG_CHIP_ID   0xD0u
#define BMP_REG_RESET     0xE0u
#define BMP_REG_STATUS    0xF3u
#define BMP_REG_CTRL_MEAS 0xF4u
#define BMP_REG_CONFIG    0xF5u
#define BMP_REG_PRESSURE  0xF7u /* pressure then temperature, 3 bytes each */

#define BMP_WHOAMI_280    0x58u
#define BMP_WHOAMI_BME280 0x60u

#define BMP_CMD_RESET     0xB6u

/* Every coefficient is two bytes, little-endian, from 0x88: temperature first
 * (T1 unsigned, T2 and T3 signed), then pressure (P1 unsigned, P2 to P9
 * signed). Twenty-four bytes, which is also the length the reference reads in
 * one go. */
#define BMP_REG_CALIB     0x88u
#define BMP_CALIB_LENGTH  24u

/*
 * 8x oversampling on pressure and 1x on temperature, in forced mode, which is
 * what the reference uses: the temperature is the slow-moving half of the
 * compensation and the pressure is the one worth averaging. Forced mode means
 * one measurement per trigger, which is what the read below does - trigger,
 * and come back for it.
 */
#define BMP_OSRS_P_8X     0x04u
#define BMP_OSRS_T_1X     0x01u
#define BMP_MODE_FORCED   0x01u
#define BMP_CTRL_MEAS     ((BMP_OSRS_P_8X << 2) | (BMP_OSRS_T_1X << 5) | \
                           BMP_MODE_FORCED)

/* IIR filter 8x, standby time 0.5 ms - the standby only matters in normal
 * mode, and this driver never leaves forced mode. The filter field's encoding
 * is not the oversampling: off, 2, 4, 8 and 16 are 0..4, so 8x is 0x03 and
 * writing 0x04 here would quietly be a 16x filter. */
#define BMP_FILTER_8X     0x03u
#define BMP_CONFIG        (BMP_FILTER_8X << 2)

/* The status register's bit 3 is set while a conversion is running. */
#define BMP_STATUS_MEASURING 0x08u

typedef struct {
    uint16_t t1;
    int16_t  t2;
    int16_t  t3;
    uint16_t p1;
    int16_t  p2;
    int16_t  p3;
    int16_t  p4;
    int16_t  p5;
    int16_t  p6;
    int16_t  p7;
    int16_t  p8;
    int16_t  p9;
    int32_t  t_fine;
} bmp_calib_t;

/* One part per bus, the same way the DPS310 driver keeps its coefficients:
 * a board has one barometer, and the coefficients are read once at open. */
static bmp_calib_t bmp_cal;

static uint16_t u16_le(const uint8_t *b)
{
    return (uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

static int16_t i16_le(const uint8_t *b)
{
    return (int16_t)u16_le(b);
}

static int bmp_read_calibration(const ak_bus_t *bus)
{
    uint8_t cal[BMP_CALIB_LENGTH];

    if (ak_bus_read(bus, BMP_REG_CALIB, cal, sizeof cal) != 0) {
        return -1;
    }

    bmp_cal.t1 = u16_le(&cal[0]);
    bmp_cal.t2 = i16_le(&cal[2]);
    bmp_cal.t3 = i16_le(&cal[4]);
    bmp_cal.p1 = u16_le(&cal[6]);
    bmp_cal.p2 = i16_le(&cal[8]);
    bmp_cal.p3 = i16_le(&cal[10]);
    bmp_cal.p4 = i16_le(&cal[12]);
    bmp_cal.p5 = i16_le(&cal[14]);
    bmp_cal.p6 = i16_le(&cal[16]);
    bmp_cal.p7 = i16_le(&cal[18]);
    bmp_cal.p8 = i16_le(&cal[20]);
    bmp_cal.p9 = i16_le(&cal[22]);
    bmp_cal.t_fine = 0;
    return 0;
}

/*
 * The datasheet's compensation is written with signed left shifts, and most of
 * these operands are negative for a real part's calibration (a `t_fine` below
 * the 128000 reference, a negative P4, a negative P7) - which is undefined
 * behaviour in C rather than "multiply by a power of two". It was found by
 * running the host suite under -fsanitize=undefined, on the bench part's own
 * calibration numbers: two "left shift of negative value" reports, and the
 * other three shifts in this function are the same shape and reachable with
 * other calibrations.
 *
 * The numbers stay the datasheet's. The shift is done on the unsigned bit
 * pattern and converted back, which is the arithmetic every compiler this
 * firmware is built with has always produced for it, and which the sanitizer
 * can see is defined. (The right shifts are the datasheet's too, and those are
 * arithmetic - implementation-defined, not undefined, and the same everywhere
 * here.)
 */
static int64_t bmp_shl(int64_t value, unsigned bits)
{
    return (int64_t)((uint64_t)value << bits);
}

/*
 * The datasheet's temperature compensation: hundredths of a degree, and a
 * `t_fine` that the pressure formula needs as well - which is why the
 * temperature is not optional even for a driver that only wants a height.
 */
static void bmp_compensate_temperature(int32_t adc_t)
{
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)bmp_cal.t1 << 1))) *
                    ((int32_t)bmp_cal.t2)) >> 11;
    int32_t var2 = (((((adc_t >> 4) - ((int32_t)bmp_cal.t1)) *
                      ((adc_t >> 4) - ((int32_t)bmp_cal.t1))) >> 12) *
                    ((int32_t)bmp_cal.t3)) >> 14;

    bmp_cal.t_fine = var1 + var2;
}

/*
 * And the pressure: pascals, through Q24.8. Every shift and every width is the
 * datasheet's; a narrower intermediate here is a number that looks like a
 * pressure and is not one, which is exactly the kind of bug that flies.
 */
static uint32_t bmp_compensate_pressure(int32_t adc_p)
{
    int64_t var1 = ((int64_t)bmp_cal.t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bmp_cal.p6;
    int64_t p;

    var2 = var2 + bmp_shl(var1 * (int64_t)bmp_cal.p5, 17);
    var2 = var2 + bmp_shl((int64_t)bmp_cal.p4, 35);
    var1 = ((var1 * var1 * (int64_t)bmp_cal.p3) >> 8) +
           bmp_shl(var1 * (int64_t)bmp_cal.p2, 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp_cal.p1) >> 33;

    if (var1 == 0) {
        /* The datasheet's own guard: a coefficient set that makes this zero is
         * a part that is not there, and dividing by it is a fault. */
        return 0;
    }

    p = 1048576 - adc_p;
    p = ((bmp_shl(p, 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp_cal.p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp_cal.p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + bmp_shl((int64_t)bmp_cal.p7, 4);

    return (uint32_t)p;
}

static int bmp_trigger(const ak_bus_t *bus)
{
    return ak_bus_write(bus, BMP_REG_CTRL_MEAS, BMP_CTRL_MEAS) == 0 ? 0 : -1;
}

static int bmp_init(const ak_bus_t *bus, ak_printf_fn out)
{
    /* The reset procedure: write the command and wait for the part to come
     * back, which the datasheet puts at two milliseconds. A barometer that was
     * mid-measurement when the flight controller reset would otherwise keep
     * whatever it had. */
    if (ak_bus_write(bus, BMP_REG_RESET, BMP_CMD_RESET) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, 2);

    if (bmp_read_calibration(bus) != 0) {
        return -1;
    }

    if (ak_bus_write(bus, BMP_REG_CONFIG, BMP_CONFIG) != 0) {
        return -1;
    }
    if (bmp_trigger(bus) != 0) {
        return -1;
    }

    if (out != 0) {
        out("baro:      configured for 8x pressure, 1x temperature, forced mode\n");
    }
    return 0;
}

static int bmp_read(const ak_bus_t *bus, ak_baro_sample_t *sample)
{
    uint8_t status = 0;
    uint8_t raw[6];

    if (ak_bus_read(bus, BMP_REG_STATUS, &status, 1) != 0) {
        return -1;
    }
    if ((status & BMP_STATUS_MEASURING) != 0u) {
        /* Still converting. Not an error, and not a sample: the caller is
         * polled, and a part that is asked while it is measuring would hand
         * back the last frame and be believed. */
        return 0;
    }

    if (ak_bus_read(bus, BMP_REG_PRESSURE, raw, sizeof raw) != 0) {
        return -1;
    }

    /* Twenty bits each, left-aligned with the low four of the third byte
     * unused - which is why the last byte is shifted rather than masked. */
    int32_t adc_p = (int32_t)(((uint32_t)raw[0] << 12) |
                              ((uint32_t)raw[1] << 4) |
                              ((uint32_t)raw[2] >> 4));
    int32_t adc_t = (int32_t)(((uint32_t)raw[3] << 12) |
                              ((uint32_t)raw[4] << 4) |
                              ((uint32_t)raw[5] >> 4));

    bmp_compensate_temperature(adc_t);

    int32_t temperature_centi = (bmp_cal.t_fine * 5 + 128) >> 8;
    uint32_t pressure_q24_8 = bmp_compensate_pressure(adc_p);

    sample->pressure_pa = (float)(pressure_q24_8 >> 8) +
                          (float)(pressure_q24_8 & 0xFFu) / 256.0f;
    sample->temperature_c = (float)temperature_centi * 0.01f;
    sample->valid = 1;

    /* And start the next one, so the part is measuring while the aircraft is
     * doing something else rather than while this driver waits. */
    if (bmp_trigger(bus) != 0) {
        return -1;
    }
    return 1;
}

const ak_baro_driver_t ak_baro_bmp280 = {
    "bmp280", BMP_REG_CHIP_ID, BMP_WHOAMI_280, bmp_init, bmp_read,
};

/* The BME280 answers a different chip id and has a humidity sensor this
 * firmware never asks about. The pressure and temperature registers, the
 * coefficients and the arithmetic are the same part. */
const ak_baro_driver_t ak_baro_bme280 = {
    "bme280", BMP_REG_CHIP_ID, BMP_WHOAMI_BME280, bmp_init, bmp_read,
};
