    .section .trampoline, "ax"
    .global bootloader_utility_get_selected_boot_partition
    .type bootloader_utility_get_selected_boot_partition, @function
bootloader_utility_get_selected_boot_partition:
    j aerialkit_select_boot_partition
    .size bootloader_utility_get_selected_boot_partition, .-bootloader_utility_get_selected_boot_partition
