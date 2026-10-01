#ifndef AK_CORE_AK_VERSION_H
#define AK_CORE_AK_VERSION_H

/*
 * Build identity, embedded in the image.
 *
 * The Makefile passes bare tokens on the command line and they are stringified
 * here, because -D with a quoted C literal is a quoting minefield and the
 * stamp contains spaces.
 *
 * Why the stamp exists at all: fc-firmware-workspace/scripts/compare-firmware.sh
 * separates "same code, different build time" from "different code" by looking
 * for a handful of changed bytes in one short window. Keeping the same shape
 * here means that tool keeps working for this firmware too.
 */

#define AK_STR_(x) #x
#define AK_STR(x) AK_STR_(x)

#ifndef AK_PRODUCT
#define AK_PRODUCT aerialkit
#endif

#ifndef AK_BOARD
#define AK_BOARD unknown
#endif

#ifndef AK_REV
#define AK_REV unknown
#endif

#ifndef AK_STAMP_VALUE
#define AK_STAMP_VALUE unknown
#endif

#define AK_PRODUCT_STR AK_STR(AK_PRODUCT)
#define AK_BOARD_STR   AK_STR(AK_BOARD)
#define AK_REV_STR     AK_STR(AK_REV)
#define AK_STAMP_STR   AK_STR(AK_STAMP_VALUE)

#endif /* AK_CORE_AK_VERSION_H */
