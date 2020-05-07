PROJECT = hts221_temp_humidity
BUILD_DIR = bin

SHARED_DIR = ../my-common-code
CFILES = main.c
CFILES += hts221.c
CFILES += systick.c
CFILES += api.c
AFILES += api-asm.S

# TODO - you will need to edit these two lines!
DEVICE=stm32l475vg
# OOCD_FILE = board/stm32f4discovery.cfg

LDSCRIPT = ./stm32-clicker.ld
LDFLAGS += -u _printf_float

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ./rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
