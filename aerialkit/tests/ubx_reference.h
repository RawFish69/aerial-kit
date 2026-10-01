#ifndef AK_TESTS_UBX_REFERENCE_H
#define AK_TESTS_UBX_REFERENCE_H

/*
 * The NAV-PVT payload, as a receiver lays it out.
 *
 * Copied field for field from INAV's ubx_nav_pvt at the pinned revision
 * (upstream/inav-9.1.0/src/main/io/gps_ublox.h; the revision is recorded in
 * docs/03-attribution.md). Every field of that struct lands on the offset the
 * u-blox interface description gives it, so offsetof() here is the offset in
 * the frame - and that is the point of this file.
 *
 * The parser's field offsets used to be two bytes too high, all ten of them,
 * and nothing caught it: the unit test built its payload from the same wrong
 * numbers the parser read, and the simulator built its frames from them too. A
 * copy of the upstream layout is something the parser cannot drift away from
 * quietly - the test checks offsetof() against the constants, and the simulator
 * builds frames from the layout rather than from the constants, so a wrong
 * constant fails a check instead of passing one.
 *
 * INAV is GPL-3.0-or-later and this firmware is too; this is a data layout
 * dictated by a third party's wire protocol, not code, and it is written down
 * here rather than vendored.
 */

#include <stdint.h>

typedef struct {
    uint32_t i_tow;   /* 0  */
    uint16_t year;    /* 4  */
    uint8_t  month;   /* 6  */
    uint8_t  day;     /* 7  */
    uint8_t  hour;    /* 8  */
    uint8_t  min;     /* 9  */
    uint8_t  sec;     /* 10 */
    uint8_t  valid;   /* 11 */
    uint32_t t_acc;   /* 12 */
    int32_t  nano;    /* 16 */
    uint8_t  fix_type;    /* 20 */
    uint8_t  flags;       /* 21 */
    uint8_t  reserved1;   /* 22, flags2 in the interface description */
    uint8_t  satellites;  /* 23 */
    int32_t  longitude;         /* 24 */
    int32_t  latitude;          /* 28 */
    int32_t  altitude_ellipsoid;/* 32 */
    int32_t  altitude_msl;      /* 36 */
    uint32_t horizontal_accuracy; /* 40 */
    uint32_t vertical_accuracy;   /* 44 */
    int32_t  ned_north;   /* 48 */
    int32_t  ned_east;    /* 52 */
    int32_t  ned_down;    /* 56 */
    int32_t  speed_2d;    /* 60 */
    int32_t  heading_2d;  /* 64 */
    uint32_t speed_accuracy;   /* 68 */
    uint32_t heading_accuracy; /* 72 */
    uint16_t position_dop;     /* 76 */
    uint16_t reserved2;        /* 78 */
    uint16_t reserved3;        /* 80 */
    /* The interface description carries four more fields after this one, to
     * make the 92 bytes: reserved1[4], headVeh, magDec and magAcc. This struct
     * stops where INAV's does, at 84, and the test and the simulator both put
     * it into a zeroed 92-byte payload. */
} ak_ubx_nav_pvt_t;

#endif /* AK_TESTS_UBX_REFERENCE_H */
