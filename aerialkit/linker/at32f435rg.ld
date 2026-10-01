/*
 * AT32F435RGT7: 1 MB flash in two 512 KB banks, 2 KB pages, and the two RAM
 * regions the reference implementation for this part uses.
 *
 * The image is linked at the base of flash with the vector table at offset 0,
 * so the flat .bin is what Artery's ROM DFU bootloader wants written to
 * 0x08000000. That path is *not* proven on this board yet - the F405's is
 * (projects/f405-bench/), and this one is the same shape rather than the same
 * evidence.
 *
 * AerialKit has no bootloader of its own. When it gets one, this script and
 * that write address change together, and the new layout goes in
 * docs/02-hardware.md.
 *
 * Two things here are this part's rather than a renamable copy of the F405's:
 *
 *   - **Flash is 2 KB pages in two banks** (bank 0 from 0x08000000, bank 1 from
 *     0x08080000). A "sector" in the F405's sense does not exist - the region
 *     the blackbox owns is a run of pages, and src/boards/AERIALKIT_GHF435/
 *     board.c erases them one at a time. The flash controller refuses a
 *     program that crosses a bank, which is why the image is kept inside bank 0
 *     by the assertion at the bottom of this file rather than by luck.
 *   - **RAM is two regions, and the first 64 KB of the address space is not
 *     mapped here.** The reference's own script for this part
 *     (upstream/inav-9.1.0/src/main/target/link/at32_flash_f43xG.ld, pinned)
 *     maps 0x20010000 + 128 KB and 0x10000000 + 64 KB and nothing else, and
 *     INAV is the only implementation of this chip there is to read. So this
 *     port follows it rather than guessing at the 64 KB below 0x20010000 - if
 *     the part hands it to the application, this line moves down and the
 *     stack top moves with it. (What the two regions actually are on the
 *     silicon - which one is fast, and whether the lower one is the ROM
 *     bootloader's - is worth reading off the datasheet once, and the answer
 *     goes here.)
 */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   (rwx) : ORIGIN = 0x20010000, LENGTH = 128K
  FAST  (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Top of SRAM, where the hardware loads the initial stack pointer from.
 * scripts/image-facts/at32f435rg.txt asserts this same number against the
 * built image, so a change here that is not made there fails the build. */
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

  /* The second RAM region: nothing lives here yet, but it is mapped so the
     first thing that wants it (a stack, a hot loop, the DMA buffers) has
     somewhere to go without a second linker script. The F405's script calls
     this region CCM; this part's name for it is not the same and its address
     is not either. */
  .fastram (NOLOAD) : {
    . = ALIGN(4);
    _sfastram = .;
    *(.fastram*)
    . = ALIGN(4);
    _efastram = .;
  } >FAST

  /* Retained: startup does not clear it, so a fault record outlives the reset
     that follows. It is in SRAM rather than the second region because it must
     be there before anything has had a chance to choose one deliberately. */
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
 *   0x08000000 .. 0x0801FFFF   the image (128 KB)
 *   0x08020000 .. 0x080DFFFF   the blackbox in flash: six 128 KB regions, 768
 *                              KB, which src/boards/AERIALKIT_GHF435/board.c
 *                              hands to the log (each on this part is 64 of
 *                              its 2 KB pages, erased one at a time)
 *   0x080E0000 .. 0x080FFFFF   saved configuration: the last 128 KB of bank 1,
 *                              of which one 2 KB page is used today
 *
 * An image that grew into the blackbox region would be erased out from under
 * itself by the first log wrap, and one that grew into the configuration
 * region would be wiped out by the first `save`. Both are link errors rather
 * than puzzles after the fact - and if the image ever needs the room, the log
 * region moves down and this line moves with it, deliberately.
 */
_flash_used_end = LOADADDR(.data) + SIZEOF(.data);
ASSERT(_flash_used_end <= 0x08020000, "image overlaps the blackbox region")
