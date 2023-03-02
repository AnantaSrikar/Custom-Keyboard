##############################################################################
BUILD = build
BIN = USB

##############################################################################
.PHONY: all directory clean size

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

ifeq ($(OS), Windows_NT)
  MKDIR = gmkdir
else
  MKDIR = mkdir
endif

CFLAGS += -g3
CFLAGS += -W -Wall --std=gnu99 -Os
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m0plus -mthumb
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m0plus -mthumb
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--script=../linker/samd10d14.ld

INCLUDES += \
  -I../include \
  -I.. \
  -I../grainuum

SRCS += \
  ../main.c \
  ../startup_samd10.c \
  ../grainuum/grainuum-phy.c \
  ../grainuum/grainuum-state.c

DEFINES += \
  -D__SAMD10C13A__ \
  -DDONT_USE_CMSIS_INIT \
  -DF_CPU=8000000

CFLAGS += $(INCLUDES) $(DEFINES)


# Make a list from SRC.c to SRC.o
OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))

all: directory $(BUILD)/grainuum-phy-ll.o $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

$(BUILD)/$(BIN).elf: $(OBJS) $(BUILD)/grainuum-phy-ll.o
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(BUILD)/grainuum-phy-ll.o $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

$(BUILD)/grainuum-phy-ll.o:
	@echo CC grainuum-phy-ll.o
	@$(CC) $(CFLAGS) ../grainuum/grainuum-phy-ll.s -c -o $@

directory:
	@$(MKDIR) -p $(BUILD)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)

