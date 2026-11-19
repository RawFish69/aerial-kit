.section .jump,"ax",@progbits
.global aerialkit_selector_jump
.type aerialkit_selector_jump,@function
aerialkit_selector_jump:
    j 0x403cc010
.size aerialkit_selector_jump, . - aerialkit_selector_jump
