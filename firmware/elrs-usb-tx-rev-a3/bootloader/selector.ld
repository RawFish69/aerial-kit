ENTRY(aerialkit_select_boot_partition)

SECTIONS
{
    .trampoline 0x403cd868 : {
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

bootloader_mmap = 0x403cd374;
bootloader_munmap = 0x403cd3dc;
