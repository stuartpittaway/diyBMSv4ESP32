# DIYBMS ATtiny1624 Bootloader
#   make                -> builds 5000 and 10000 baud
#   make BAUDS=9600     -> single variant
#
# Needs avr-gcc with tinyAVR-2 support (e.g. avr-gcc 14 / avr-libc 2.2
# or the PlatformIO toolchain "toolchain-atmelavr").

AVRGCC ?= avr-gcc
OBJCOPY ?= avr-objcopy
SIZE ?= avr-size
MCU = attiny1624
F_CPU = 5000000UL
BAUDS ?= 5000 10000

CFLAGS = -mmcu=$(MCU) -Os -std=gnu11 -Wall -Wextra -DF_CPU=$(F_CPU) \
         -I src -I ../common -ffunction-sections -fdata-sections -mrelax \
         -fno-jump-tables -flto
LDFLAGS = -nostartfiles -Wl,--gc-sections -Wl,-Map,$(basename $@).map

all: $(foreach b,$(BAUDS),build/diybms_bootloader_$(MCU)_$(b).hex)

build/diybms_bootloader_$(MCU)_%.elf: src/main.c src/bl_core.c src/bl_core.h ../common/fwupdate_protocol.h
	@mkdir -p build
	$(AVRGCC) $(CFLAGS) -DBL_BAUD=$*UL $(LDFLAGS) -o $@ src/main.c src/bl_core.c
	@$(SIZE) $@
	@sz=$$($(SIZE) -A $@ | awk '/^\.text|^\.data|^\.init0/ {s+=$$2} END {print s}'); \
	 if [ $$sz -gt 1536 ]; then echo "ERROR: bootloader $$sz bytes > 1536"; rm -f $@; exit 1; fi; \
	 echo "Bootloader: $$sz of 1536 bytes"


build/%.hex: build/%.elf
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse $< $@

clean:
	rm -rf build

.SECONDARY:
.PHONY: all clean
