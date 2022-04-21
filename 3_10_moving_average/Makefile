PROJECT = 3_10_moving_average
BUILD_DIR = bin

SHARED_DIR = ../common
CFILES = 3_10_moving_average.c wm8960.c i2s.c clock.c gpio.c i2c.c

DEVICE=stm32f407vet6
OOCD_FILE = ../waveshare_core407v.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk

# run in a separate window while debugging
openocd:
	$(OOCD) -f $(OOCD_FILE)
.PHONY: openocd

# run while openocd is running
debug:		$(PROJECT).debug
%.debug:	%.elf
	$(PREFIX)gdb $(PROJECT).elf -x ../gdbinit.gdb
# %.debug:	%.elf
# 	($(OOCD) -f $(OOCD_FILE) &); \
# 	    $(PREFIX)gdb $(PROJECT).elf -x ../gdbinit.gdb;
.PHONY:	debug

include $(OPENCM3_DIR)/mk/genlink-rules.mk
