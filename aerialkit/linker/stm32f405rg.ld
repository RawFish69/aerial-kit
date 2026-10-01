/*
 * STM32F405RGT6: 1 MB flash, 128 KB SRAM, 64 KB CCM.
 *
 * The image is linked at the base of flash with the vector table at offset 0,
 * so the flat .bin is exactly what the STM32F405 ROM bootloader wants written
 * to 0x08000000. That is the path fc-firmware-workspace/projects/f405-bench/
 * proved on the WeAct board.
 *
 * AerialKit has no bootloader of its own yet. When it gets one, this script
 * and that write address change together, and the new layout goes in
 * docs/02-hardware.md.
 */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
  CCM   (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Top of SRAM, where the hardware loads the initial stack pointer from. */
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  .isr_vector : {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
  } >FLASH

  .text : {
    . = ALIGN(4);
    *(.text*)
    *(.rodata*)
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  _sidata = LOADADDR(.data);

  .data : {
    . = ALIGN(4);
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  .bss : {
    . = ALIGN(4);
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } >RAM

  /* Core-coupled memory: nothing lives here yet, but it is mapped so the
     first thing that wants it (a stack, a hot loop) has somewhere to go. */
  .ccm (NOLOAD) : {
    . = ALIGN(4);
    _sccm = .;
    *(.ccm*)
    . = ALIGN(4);
    _eccm = .;
  } >CCM

  /* Retained: startup does not clear it, so a fault record outlives the reset
     that follows. It is in SRAM rather than the CCM because it must be there
     before anything has had a chance to enable CCM properly. */
  .noinit (NOLOAD) : {
    . = ALIGN(4);
    _snoinit = .;
    *(.noinit*)
    . = ALIGN(4);
    _enoinit = .;
  } >RAM

  /DISCARD/ : {
    *(.comment)
    *(.ARM.exidx*)
    *(.note.gnu.build-id)
  }
}

/*
 * The layout of the 1 MB part, asserted rather than remembered:
 *
 *   0x08000000 .. 0x0801FFFF   the image (128 KB, and about 73 KB today)
 *   0x08020000 .. 0x080BFFFF   the blackbox in flash: five 128 KB sectors,
 *                              which is the region src/boards/AERIALKIT_F405/
 *                              board.c hands to the log
 *   0x080C0000 .. 0x080FFFFF   saved configuration: the last two 128 KB
 *                              sectors, the two banks the record alternates
 *                              between so that the erase a recycle needs never
 *                              lands on the one holding the newest record
 *
 * The log was six sectors until the configuration's second bank took one; see
 * the section comment in the board file for what that bought.
 *
 * An image that grew into the blackbox region would be erased out from under
 * itself by the first log wrap, and one that grew into the configuration
 * sectors would be wiped out by the first `save`. Both are link errors rather
 * than puzzles after the fact - and if the image ever needs the room, the log
 * region moves down and this line moves with it, deliberately.
 */
_flash_used_end = LOADADDR(.data) + SIZEOF(.data);
ASSERT(_flash_used_end <= 0x08020000, "image overlaps the blackbox region")
