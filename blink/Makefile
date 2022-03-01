PROJECT = blink
BUILD_DIR = bin

SHARED_DIR = ../common
CFILES = blink.c
# CFILES += api.c
# AFILES += api-asm.S

# TODO - you will need to edit these two lines!
DEVICE=stm32f407vet6
OOCD_FILE = ../waveshare_core407v.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
