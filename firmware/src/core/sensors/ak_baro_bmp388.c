#include "ak_baro.h"

/*
 * Bosch BMP388, and the BMP390 that is the same part with a different chip id.
 *
 * The fourth barometer this firmware knows, and the last one the plan names.
 * It is a different arithmetic from the BMP280 in a way worth stating: the
 * BMP388's compensation comes in two variants - a float one with scaled
 * coefficients, and a fixed-point one in 64-bit integers - and this driver is
 * the fixed-point one, because the target has no double precision unit and the
 * core links no libm. The result of the integer variant is in *hundredths* of
 * a pascal, which is not a detail: the Bosch API's own comment says "if
 * pressure is 9528709 which is 9528709/100 = 95287.09 Pascal", and its limits
 * for that variant are 3000000 and 12500000 where the float variant's are
 * 30000.0 and 125000.0. A hundredfold error here is an altitude out by about
 * eleven kilometres, in the right units, and nothing downstream would notice.
 *
 * Register map, chip ids, trimming data, and the compensation arithmetic:
 * INAV 9.1.0 src/main/drivers/barometer/barometer_bmp388.c @ e519b69 - which
 * follows the BMP388 datasheet's sections 3.11 and 9.1 - checked against
 * Bosch's own BMP3-Sensor-API bmp3.c, whose integer variant has the same
 * operations and the units comment quoted above. No code was copied; the
 * attribution table has the entry.
 *
 * The two references disagree about one thing, and the datasheet settles it:
 * trimming words P5 and P6 are *unsigned* (both INAV and the Bosch API say so;
 * ArduPilot's float driver reads them signed, which is harmless for the values
 * real parts have). They are unsigned here.
 */

#define BMP388_REG_CHIP_ID 0x00u
#define BMP388_REG_STATUS  0x03u
#define BMP388_REG_DATA    0x04u /* pressure then temperature, 3 bytes each */
#define BMP388_REG_PWR_CTRL 0x1Bu
#define BMP388_REG_OSR     0x1Cu
#define BMP388_REG_ODR     0x1Du
#define BMP388_REG_CONFIG  0x1Fu
#define BMP388_REG_CMD     0x7Eu

#define BMP388_WHOAMI_388  0x50u
#define BMP388_WHOAMI_390  0x60u
#define BMP388_CMD_RESET   0xB6u

/* The trimming data: temperature from 0x31 (five bytes), pressure from 0x36
 * (sixteen), which is the packing the datasheet's memory map describes and
 * both references read in one go. */
#define BMP388_REG_CAL_T   0x31u
#define BMP388_CAL_T_LENGTH 5u
#define BMP388_REG_CAL_P   0x36u
#define BMP388_CAL_P_LENGTH 16u

/* STATUS: bit 5 says pressure data is ready, bit 6 temperature. In forced mode
 * the part measures once and goes back to sleep, which is what a polled driver
 * wants - the same shape as the BMP280 driver next door. */
#define BMP388_STATUS_PRESS 0x20u
#define BMP388_STATUS_TEMP  0x40u

/* Oversampling ×8 on pressure and ×1 on temperature, forced mode: the
 * temperature is the slow-moving half of the compensation and the pressure is
 * the one worth averaging. The OSR field is 0 for ×1, 1 for ×2 and so on. */
#define BMP388_OSR_P_8X    0x03u
#define BMP388_OSR_T_1X    0x00u
#define BMP388_OSR         (BMP388_OSR_P_8X | (BMP388_OSR_T_1X << 3))

/* PWR_CTRL: enable pressure, enable temperature, and the mode in bits 4-5.
 * Forced is 01; normal mode (10) would convert forever behind the driver's
 * back and draw current the aircraft does not need. */
#define BMP388_MODE_FORCED 0x01u
#define BMP388_PWR_CTRL    ((BMP388_MODE_FORCED << 4) | 0x02u | 0x01u)

/* IIR filter ×4 and a standby time that only matters in normal mode. */
#define BMP388_CONFIG      0x04u

/* The Bosch API's own limits for the compensated value, in hundredths of a
 * pascal: 30000.0 to 125000.0 Pa. Outside them the part is not telling us
 * about this atmosphere, and a reading that passes them into the altitude
 * arithmetic is worse than no reading. */
#define BMP388_MIN_HUNDREDTHS 3000000
#define BMP388_MAX_HUNDREDTHS 12500000

typedef struct {
    uint16_t t1;
    uint16_t t2;
    int8_t   t3;
    int16_t  p1;
    int16_t  p2;
    int8_t   p3;
    int8_t   p4;
    uint16_t p5; /* unsigned, see the note at the top */
    uint16_t p6;
    int8_t   p7;
    int8_t   p8;
    int16_t  p9;
    int8_t   p10;
    int8_t   p11;
    int64_t  t_lin; /* the temperature, in the fixed-point the pressure needs */
} bmp388_calib_t;

/* One part per bus, like the other barometer drivers: a board has one. */
static bmp388_calib_t bmp388_cal;

static uint16_t u16_le(const uint8_t *b)
{
    return (uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

static int16_t i16_le(const uint8_t *b)
{
    return (int16_t)u16_le(b);
}

static int bmp388_read_calibration(const ak_bus_t *bus)
{
    uint8_t t[BMP388_CAL_T_LENGTH];
    uint8_t p[BMP388_CAL_P_LENGTH];

    if (ak_bus_read(bus, BMP388_REG_CAL_T, t, sizeof t) != 0) {
        return -1;
    }
    if (ak_bus_read(bus, BMP388_REG_CAL_P, p, sizeof p) != 0) {
        return -1;
    }

    bmp388_cal.t1 = u16_le(&t[0]);
    bmp388_cal.t2 = u16_le(&t[2]);
    bmp388_cal.t3 = (int8_t)t[4];

    bmp388_cal.p1 = i16_le(&p[0]);
    bmp388_cal.p2 = i16_le(&p[2]);
    bmp388_cal.p3 = (int8_t)p[4];
    bmp388_cal.p4 = (int8_t)p[5];
    bmp388_cal.p5 = u16_le(&p[6]);
    bmp388_cal.p6 = u16_le(&p[8]);
    bmp388_cal.p7 = (int8_t)p[10];
    bmp388_cal.p8 = (int8_t)p[11];
    bmp388_cal.p9 = i16_le(&p[12]);
    bmp388_cal.p10 = (int8_t)p[14];
    bmp388_cal.p11 = (int8_t)p[15];
    bmp388_cal.t_lin = 0;
    return 0;
}

/*
 * The datasheet's fixed-point temperature: hundredths of a degree, and a
 * `t_lin` the pressure formula needs as well - which is why a driver that only
 * wanted a height still has to compensate the temperature first.
 */
static int64_t bmp388_compensate_temperature(uint32_t raw)
{
    int64_t partial1 = (int64_t)raw - (256 * (int64_t)bmp388_cal.t1);
    int64_t partial2 = (int64_t)bmp388_cal.t2 * partial1;
    int64_t partial3 = partial1 * partial1;
    int64_t partial4 = partial3 * (int64_t)bmp388_cal.t3;
    int64_t partial5 = (partial2 * 262144) + partial4;

    bmp388_cal.t_lin = partial5 / 4294967296;
    return (bmp388_cal.t_lin * 25) / 16384;
}

/*
 * And the pressure, in hundredths of a pascal. Every shift, every multiplier
 * and the order of the divisions are the datasheet's; the interim values reach
 * about 2e16 for a 24-bit raw reading, which is inside an int64 with three
 * orders of magnitude to spare - and that is worth knowing rather than
 * assuming, because the Bosch API splits one division in two ("dividing by 10
 * followed by multiplying by 10") to dodge an overflow it can see coming on
 * this arithmetic.
 */
static int64_t bmp388_compensate_pressure(uint32_t raw)
{
    int64_t t = bmp388_cal.t_lin;
    int64_t partial1 = t * t;
    int64_t partial2 = partial1 / 64;
    int64_t partial3 = (partial2 * t) / 256;
    int64_t partial4 = ((int64_t)bmp388_cal.p8 * partial3) / 32;
    int64_t partial5 = ((int64_t)bmp388_cal.p7 * partial1) * 16;
    int64_t partial6 = ((int64_t)bmp388_cal.p6 * t) * 4194304;
    int64_t offset = ((int64_t)bmp388_cal.p5 * 140737488355328) + partial4 +
                     partial5 + partial6;
    int64_t sensitivity;

    partial2 = ((int64_t)bmp388_cal.p4 * partial3) / 32;
    partial4 = ((int64_t)bmp388_cal.p3 * partial1) * 4;
    partial5 = (((int64_t)bmp388_cal.p2 - 16384) * t) * 2097152;
    sensitivity = (((int64_t)bmp388_cal.p1 - 16384) * 70368744177664) +
                  partial2 + partial4 + partial5;

    partial1 = (sensitivity / 16777216) * (int64_t)raw;
    partial2 = (int64_t)bmp388_cal.p10 * t;
    partial3 = partial2 + (65536 * (int64_t)bmp388_cal.p9);
    partial4 = (partial3 * (int64_t)raw) / 8192;

    /*
     * The step after this one multiplies by the raw reading *again*, and the
     * datasheet's arithmetic assumes that product fits a signed 64-bit. It
     * does until the raw reading passes about twelve million: with the bench
     * part's calibration `partial4` is 1.14e12 there and the product wants
     * 1.9e19 against a ceiling of 9.2e18. That is UB, not a wrap, and it was
     * found by running this suite under -fsanitize=undefined - the reading
     * that triggers it is a full-scale one, which the caller refuses anyway,
     * so the bug was "the answer is refused after the undefined thing has
     * already happened".
     *
     * So the multiplication is guarded, and a reading that cannot be carried
     * gets the same answer the datasheet's own zero-division guard gives
     * below: a value the caller's range check refuses. Bosch's API and
     * Betaflight's port both write the unguarded product; this firmware does
     * not, because the arithmetic here decides an altitude.
     */
    if (raw != 0u && (partial4 > INT64_MAX / (int64_t)raw ||
                      partial4 < INT64_MIN / (int64_t)raw)) {
        return 0;
    }
    partial5 = (partial4 * (int64_t)raw) / 512;
    partial6 = (int64_t)raw * (int64_t)raw;
    partial2 = ((int64_t)bmp388_cal.p11 * partial6) / 65536;
    partial3 = (partial2 * (int64_t)raw) / 128;
    partial4 = (offset / 4) + partial1 + partial5 + partial3;

    /*
     * The last step is unsigned, and that is not tidiness: the sum above is
     * about 4.4e17 at sea level, and multiplying *that* by 25 overflows a
     * signed 64-bit - it went negative here, which reads as a pressure below
     * absolute zero and is what the host test caught. The Bosch API casts to
     * uint64_t for the same reason. 1.1e19 is still inside an unsigned 64-bit
     * by a comfortable margin.
     */
    return (int64_t)(((uint64_t)partial4 * 25u) / 1099511627776u);
}

static int bmp388_trigger(const ak_bus_t *bus)
{
    return ak_bus_write(bus, BMP388_REG_PWR_CTRL, BMP388_PWR_CTRL) == 0 ? 0
                                                                       : -1;
}

static int bmp388_init(const ak_bus_t *bus, ak_printf_fn out)
{
    /* The soft reset first, so a part that was mid-conversion when the flight
     * controller reset starts from a known state. The datasheet allows two
     * milliseconds and the references wait ten. */
    if (ak_bus_write(bus, BMP388_REG_CMD, BMP388_CMD_RESET) != 0) {
        return -1;
    }
    ak_bus_delay_ms(bus, 10);

    if (bmp388_read_calibration(bus) != 0) {
        return -1;
    }
    if (ak_bus_write(bus, BMP388_REG_CONFIG, BMP388_CONFIG) != 0) {
        return -1;
    }
    if (ak_bus_write(bus, BMP388_REG_ODR, 0x00u) != 0) {
        return -1;
    }
    if (ak_bus_write(bus, BMP388_REG_OSR, BMP388_OSR) != 0) {
        return -1;
    }
    if (bmp388_trigger(bus) != 0) {
        return -1;
    }

    if (out != 0) {
        out("baro:      configured for 8x pressure, 1x temperature, forced mode\n");
    }
    return 0;
}

static int bmp388_read(const ak_bus_t *bus, ak_baro_sample_t *sample)
{
    uint8_t status = 0;
    uint8_t raw[6];

    if (ak_bus_read(bus, BMP388_REG_STATUS, &status, 1) != 0) {
        return -1;
    }
    if ((status & (BMP388_STATUS_PRESS | BMP388_STATUS_TEMP)) !=
        (BMP388_STATUS_PRESS | BMP388_STATUS_TEMP)) {
        /* Still converting. Not an error and not a sample: a part asked while
         * it is measuring would hand back the frame before this one. */
        return 0;
    }

    if (ak_bus_read(bus, BMP388_REG_DATA, raw, sizeof raw) != 0) {
        return -1;
    }

    /* Three bytes each, least significant first, in the order the part writes
     * them: pressure at 0x04, temperature at 0x07. */
    uint32_t adc_p = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
                     ((uint32_t)raw[2] << 16);
    uint32_t adc_t = (uint32_t)raw[3] | ((uint32_t)raw[4] << 8) |
                     ((uint32_t)raw[5] << 16);

    int64_t temperature_hundredths = bmp388_compensate_temperature(adc_t);
    int64_t pressure_hundredths = bmp388_compensate_pressure(adc_p);

    if (pressure_hundredths < BMP388_MIN_HUNDREDTHS ||
        pressure_hundredths > BMP388_MAX_HUNDREDTHS) {
        /* Outside the range the part can measure. Said as an error rather than
         * handed on: the altitude arithmetic would turn it into a height, and
         * a height nobody can fly to. */
        return -1;
    }

    sample->pressure_pa = (float)pressure_hundredths / 100.0f;
    sample->temperature_c = (float)temperature_hundredths / 100.0f;
    sample->valid = 1;

    /* And start the next one, so the part measures while the aircraft does
     * something else rather than while this driver waits. */
    if (bmp388_trigger(bus) != 0) {
        return -1;
    }
    return 1;
}

const ak_baro_driver_t ak_baro_bmp388 = {
    "bmp388", BMP388_REG_CHIP_ID, BMP388_WHOAMI_388, bmp388_init, bmp388_read,
};

/* The BMP390 is the same part with a different chip id - the register map, the
 * trimming data and the arithmetic are identical, and the datasheet is the
 * same document. */
const ak_baro_driver_t ak_baro_bmp390 = {
    "bmp390", BMP388_REG_CHIP_ID, BMP388_WHOAMI_390, bmp388_init, bmp388_read,
};
