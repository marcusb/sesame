# Copyright 2008-2019 Marvell International Ltd.
# All Rights Reserved.

OS := $(shell uname)

ifneq ($(NOISY),1)
AT=@
else
AT=
endif

TARGET ?= axf2firmware
CC ?= gcc
SDK_PATH ?= $(CURDIR)/../../../../..

ifeq ($(TOOLCHAIN),iar)
  CFLAGS := -DAXF_BUILT_BY_IAR
else
  CFLAGS := -DAXF_BUILT_BY_ARM_GCC
endif

CFLAGS += -O2 -Wall -fdata-sections -ffunction-sections

ifneq ($(OS), Darwin)
LDFLAGS = -static -s -Wl,--gc-sections
endif

all: $(TARGET)

$(TARGET): soft_crc32.c axf2firmware.c
	@echo " [host][ld] $@"
	$(AT)$(CC) -O2 -Wall $(CFLAGS) $(LDFLAGS) -I. -o $@ $^

clean:
	$(AT)rm -f $(TARGET) $(wildcard *.o)
