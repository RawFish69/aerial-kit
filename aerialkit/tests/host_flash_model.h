#ifndef AK_HOST_FLASH_MODEL_H
#define AK_HOST_FLASH_MODEL_H

/*
 * The flash controller, modelled, for the host build of tests/test_arch.c.
 *
 * flash.c routes its own register accesses through host_flash_reg_read() and
 * host_flash_reg_write() when it is compiled with -DAK_HOST_FLASH, and the
 * words it programs through host_flash_word_write(). That seam is the whole
 * of it, and this model is what makes the driver's *success* path - a sector
 * that erases, a word that lands where it was addressed, the flags after
 * either - runnable without silicon. It is the one part of the port the
 * register-block trick could not reach, because it needs the part to act on
 * what is written rather than to store it.
 *
 * What it models, from RM0090 section 3: the unlock sequence and the lock bit
 * it clears, the status flags and their clear-by-writing-one, the two control
 * bits that start an operation, and the sector geometry of a 1 MB part. The
 * geometry is not a detail - four 16 KB sectors, one of 64 KB and seven of
 * 128 KB - because a model that assumed every sector was the same size erased
 * the wrong region and reported success. That is why the tests check the
 * flash's *contents* rather than anything this model says about itself.
 *
 * The flash is the region the test maps at 0x08000000: the model erases and
 * programs into that memory, so what the driver reads back is what is really
 * there, and a test can read it byte for byte afterwards.
 */

#include <stdint.h>

uint32_t host_flash_reg_read(uint32_t offset);
void     host_flash_reg_write(uint32_t offset, uint32_t value);
void     host_flash_word_write(uint32_t address, uint32_t value);

/* A part at power-on: locked, flags clear, every byte of the flash 0xFF. */
void host_flash_model_reset(void);

/* What a test can make the part do, so the failure paths stay reachable.
 * `refused` is the one behaviour no real part has, and it is here on purpose:
 * it is how the driver's "this controller will not unlock" branch is still
 * exercised now that the model does clear the lock bit. */
void host_flash_model_set_locked(int locked);
void host_flash_model_set_unlock_refused(int refused);

/* And a controller whose current operation never finishes: the busy bit stays
 * set for as long as anybody asks, and the erase it was asked for does not
 * happen. That is the one behaviour that turns this driver from slow into a
 * board that never boots again, because the wait for the flag to clear is a
 * loop - so the loop is bounded (`AK_FLASH_TIMEOUT`) and this is how the bound
 * is executed rather than trusted. */
void host_flash_model_set_stuck_busy(int stuck);

/* And one that erases happily and refuses to program - a worn cell, a supply
 * that sagged in the middle of a save, the state that used to cost the
 * aircraft its settings because the record it was replacing had just been
 * erased. Every word write raises the programming error and lands nowhere. */
void host_flash_model_set_program_refused(int refused);

/* The last key written to KEYR, for the checks that are about what reached the
 * controller rather than about what ended up in the flash. */
uint32_t host_flash_model_keys(void);

/* How many sector erases this controller has performed since the reset, which
 * is the other thing that cannot be read off the flash: an erase leaves the
 * same 0xFF whether it happened once or a thousand times. The configuration
 * ring's whole hazard is *how often* it has to erase - once every thirty-two
 * saves, and everything is in the sector that erase destroys - so the count is
 * the measurement rather than a way to find one. */
uint32_t host_flash_model_erases(void);

#endif /* AK_HOST_FLASH_MODEL_H */
