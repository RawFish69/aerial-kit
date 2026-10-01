# AK Firmware - the Feather F405 image. ESP32DEV uses ports/esp32/.
BOARD ?= FEATHER_F405
ARCH ?= stm32f405
PART ?= stm32f405rg
PRODUCT ?= aerialkit-feather-f405
OUT ?= build/feather
CROSS ?= arm-none-eabi-
REV ?= $(shell git -C $(CURDIR) describe --always --dirty 2>/dev/null || echo unknown)
STAMP ?= $(shell date +'%b %d %Y / %H:%M:%S')

ifneq ($(BOARD),FEATHER_F405)
$(error This public build supports BOARD=FEATHER_F405; ESP32DEV uses ESP-IDF)
endif
ifneq ($(ARCH),stm32f405)
$(error FEATHER_F405 requires ARCH=stm32f405)
endif
ifneq ($(PART),stm32f405rg)
$(error FEATHER_F405 requires PART=stm32f405rg)
endif

CC := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE := $(CROSS)size
LDSCRIPT := linker/$(PART).ld
INCLUDES := src/core src/core/flight src/core/sensors src/arch/$(ARCH) src/arch/arm/cortex-m4 src/boards/$(BOARD)
SRCS := $(wildcard src/core/*.c) $(wildcard src/core/flight/*.c) $(wildcard src/core/sensors/*.c) $(wildcard src/arch/arm/cortex-m4/*.c) $(wildcard src/arch/$(ARCH)/*.c) $(wildcard src/boards/$(BOARD)/*.c)
OBJS := $(SRCS:%.c=$(OUT)/%.o)
DEPS := $(OBJS:.o=.d)
CFLAGS := -std=c11 -O2 -g3 \
          -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 \
          -ffreestanding -fno-common -ffunction-sections -fdata-sections \
          -fstack-usage -Wall -Wextra -Wshadow -Wundef -Wdouble-promotion -Wconversion \
          $(addprefix -I,$(INCLUDES)) \
          -DAK_BOARD=$(BOARD) -DAK_PRODUCT=$(PRODUCT) -DAK_REV=$(REV) $(EXTRA_CFLAGS)
CFLAGS_KEYED := $(CFLAGS)
CFLAGS += -DAK_STAMP_VALUE="$(STAMP)"
LDFLAGS := -T $(LDSCRIPT) -nostdlib -Wl,--gc-sections -Wl,-Map=$(OUT)/$(PRODUCT).map -Wl,--print-memory-usage
LIBS := -lgcc

.DEFAULT_GOAL := all
.PHONY: all check size clean FORCE
all: $(OUT)/$(PRODUCT).bin $(OUT)/$(PRODUCT).hex size
-include $(DEPS)

$(OUT)/$(PRODUCT).elf: $(OBJS) $(LDSCRIPT)
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

$(OUT)/%.bin: $(OUT)/%.elf
	$(OBJCOPY) -O binary $< $@
$(OUT)/%.hex: $(OUT)/%.elf
	$(OBJCOPY) -O ihex $< $@
$(OUT)/%.o: %.c $(OUT)/.cflags
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
$(OUT)/.cflags: FORCE
	@mkdir -p $(@D)
	@printf '%s\n' '$(CFLAGS_KEYED)' | cmp -s - $@ || printf '%s\n' '$(CFLAGS_KEYED)' > $@
FORCE:
size: $(OUT)/$(PRODUCT).elf
	@$(SIZE) $<
check: $(OUT)/$(PRODUCT).bin
	@./scripts/check-image.sh $< scripts/image-facts/$(PART).txt $(OUT)/$(PRODUCT).elf $(if $(findstring AK_BOOT_STAGE=1,$(EXTRA_CFLAGS)),--telltale,) $(if $(findstring AK_USB_TRACE=1,$(EXTRA_CFLAGS)),--usb-trace,)
clean:
	@python3 -c 'from pathlib import Path; import shutil; p=Path("$(OUT)").resolve(); root=Path.cwd().resolve(); assert p != root and root in p.parents, "OUT must be a child of firmware/"; shutil.rmtree(p, ignore_errors=True)'
