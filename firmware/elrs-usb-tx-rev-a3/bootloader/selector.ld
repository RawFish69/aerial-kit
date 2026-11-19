ENTRY(aerialkit_select_boot_partition)

SECTIONS
{
    .trampoline @BOOTLOADER_SELECT_FUNCTION@ : {
        KEEP(*(.trampoline))
    }

    .selector 0x403cc000 : {
        KEEP(*(.literal .literal.* .text .text.*))
    }

    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
        *(.xtensa.info)
    }
}

bootloader_mmap = @BOOTLOADER_MMAP@;
bootloader_munmap = @BOOTLOADER_MUNMAP@;
