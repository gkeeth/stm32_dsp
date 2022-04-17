PROJECT = 2_14_sweep
BUILD_DIR = bin

SHARED_DIR = ../common
CFILES = 2_14_sweep.c wm8960.c i2s.c clock.c gpio.c i2c.c

DEVICE=stm32f407vet6
OOCD_FILE = ../waveshare_core407v.cfg

CMSIS_DIR = ../CMSIS_5/CMSIS
CMSIS_CORE_DIR = $(CMSIS_DIR)/Core
CMSIS_DSP_DIR = $(CMSIS_DIR)/DSP
CMSIS_INC += $(CMSIS_CORE_DIR)/Include
CMSIS_INC += $(CMSIS_DSP_DIR)/Include
INCLUDES += $(patsubst %,-I%, . $(CMSIS_INC))

# CMSIS_DSP_BUILD_DIR = ../cmsis_dsp_build/build/bin_dsp
# LDLIBS += -L$(CMSIS_DSP_BUILD_DIR)/BasicMathFunctions -lCMSISDSPBasicMath
# LDLIBS += -L$(CMSIS_DSP_BUILD_DIR)/FastMathFunctions -lCMSISDSPFastMath
# LDLIBS += -L$(CMSIS_DSP_BUILD_DIR)/CommonTables -lCMSISDSPCommon

# LDLIBS += -lm

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
