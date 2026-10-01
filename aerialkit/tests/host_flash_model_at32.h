#ifndef AK_HOST_FLASH_MODEL_AT32_H
#define AK_HOST_FLASH_MODEL_AT32_H

/*
 * The AT32F435's flash controller, modelled, for the host build of the port's
 * checks.
 *
 * The same seam as the F405's model, and for the same reason: `flash.c` routes
 * its register accesses and its data words through these three functions when
 * it is compiled with -DAK_HOST_FLASH_AT32, so the driver's *success* path - a
 * page that erases, a word that lands where it was addressed, the flags after
 * either - runs without silicon. What is modelled is this part: **2 KB pages**,
 * **two banks of 512 KB** with their own registers, the unlock keys, and the
 * flags. A model that assumed the F405's geometry would erase the wrong region
 * and say it had succeeded - which is exactly the bug the F405's own model was
 * written after.
 *
 * The flash is the region a test maps at 0x08000000: the model erases and
 * programs into that memory, so what the driver reads back is what is really
 * there.
 */

#include <stdint.h>

uint32_t host_at32_flash_reg_read(uint32_t offset);
void     host_at32_flash_reg_write(uint32_t offset, uint32_t value);
void     host_at32_flash_word_write(uint32_t address, uint32_t value);

/* A part at power-on: both banks locked, flags clear, every byte 0xFF. */
void host_at32_flash_model_reset(void);

/* So the failure paths stay reachable: a controller that will not unlock, and
 * one that reports a programming error. Neither is a behaviour a real part has
 * on demand, and both are branches the driver has to get right. */
void host_at32_flash_model_set_unlock_refused(int refused);
void host_at32_flash_model_set_program_error(int error);

/* A controller whose current operation never finishes: the busy bit stays set,
 * the page erase does not happen, and the word does not land. This is the
 * F405's model's `set_stuck_busy` for the other part, and it is here for the
 * same reason - the driver's wait is a loop with a bound in it, and a bound
 * nothing ever reaches is a bound nobody has ever run. */
void host_at32_flash_model_set_stuck_busy(int stuck);

/* The keys the last unlock wrote, for the checks that are about what reached
 * the controller rather than about what ended up in the flash. */
uint32_t host_at32_flash_model_keys(void);

#endif /* AK_HOST_FLASH_MODEL_AT32_H */
