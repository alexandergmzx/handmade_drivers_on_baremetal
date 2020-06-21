##############################################################################
# Makefile for baremetal compiling
#
# Use: make [target]
#
# Build targets:
#      <FILE>.o - Builds a <FILE>.o object file
#      <FILE>.i - Builds a <FILE>.i preprocessed file
#      <FILE>.asm - Dumps <FILE>.asm assembly file
#      <FILE>.d - Builds <FILE>.d dependency file
#      semi - XX Compiles all source files in project, but does not link.
#      all -  XX Builds all object files in project (links as well)
#      clean - Removes all generated files
#
# Example : make 
#
# MIT License
#
# Copyright (c) 2020 J. Alexander Gómez G.                             
# @date 	2020-Jun-12
#
##############################################################################
SRCS = main.c \
	led.c \
	stm32_startup.c \
	syscalls.c 

INCLUDES = main.h \
	led.h

# PRJ_NAME.elf 
PRJ_NAME = final

CC 	= arm-none-eabi-gcc # Compiler
LD 	= arm-none-eabi-ld # Loader
SIZE 	= arm-none-eabi-size	
OBJDMP	= arm-none-eabi-objdump
OBJCOPY	= arm-none-eabi-objcopy # for Hex

# General GCC compiler flags for both platforms
CFLAGS = -Wall \
	-g \
	-O0 \
	-std=gnu11

CPPFLAGS = -MMD -MP $(INCLUDES)

# Programmer flags
PROGRAMMER = openocd
PGFLAGS    = -f board/st_nucleo_f4.cfg -c "program $(PRJ_NAME).elf verify reset" -c shutdown

#openocd.cfg 

# Architectures Specific Flags
CPU = cortex-m4 		
ARCH = armv7e-m 		
SPECS = nano.specs 
FLOATABI = soft
ISAFLAG = -mthumb 

# Compiler Flags and Defines
CFLAGS += $(ISAFLAG) \
	-mcpu=$(CPU) \
	-march=$(ARCH) \
	-mfloat-abi=$(FLOATABI) # Compiler Flags 

#Linker & Loader Flags
LINKER_FILE = stm32_ls.ld

# This also generates the Map file for the full build

LDFLAGS = --specs=$(SPECS) -T$(LINKER_FILE) -Wl,-Map=$(PRJ_NAME).map

#LDFLAGS_SH = --specs=rdimon.specs -T$(LINKER_FILE) -Wl,-Map=$(PRJ_NAME).map


all: $(PRJ_NAME).elf
semi: $(PRJ_NAME)_sh.elf 
#semi stands for Semi Hosting debugger

# Pattern Matching - Associate source files with:
OBJS = $(SRCS:.c=.o)	# Object files
DEPS = $(SRCS:.c=.d)	# Dependency files
ASMS = $(SRCS:.c=.asm)	# Assembly files
PREP = $(SRCS:.c=.i)	# Preprocessed files

# Dependency Files for each source file 
%.d: %.c
	$(CC) -E -M $< $(CPPFLAGS) -o $@
# Preprocessed output of implementation files
%.i: %.c
	$(CC) -E $< $(CPPFLAGS) -o $@
# Assembly output files
%.asm: %.c
	$(CC) -S $< $(CFLAGS) $(CPPFLAGS) -o $@
# Individual object files
%.o: %.c
	$(CC) -c $< $(CFLAGS) -o $@

$(PRJ_NAME).elf: $(OBJS)
	$(CC) $^ $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) -o $@ # instead of $(OBJS), uses $^
	$(SIZE) $< $@
	
$(PRJ_NAME)_sh.elf: main.o led.o stm32_startup.o
	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS_SH) -o $@ $^ 
	$(SIZE) $< $@

$(PRJ_NAME).asm: $(PRJ_NAME).elf
	$(OBJDMP) --disassemble-all $(PRJ_NAME).elf > $(PRJ_NAME).asm

flash: $(PRJ_NAME).elf
	$(PROGRAMMER) $(PGFLAGS) 

hex: $(PRJ_NAME).elf
	$(OBJCOPY) -O ihex $(PRJ_NAME).elf $(PRJ_NAME).hex

bin: $(PRJ_NAME).elf
	$(OBJCOPY) -O binary $(PRJ_NAME).elf $(PRJ_NAME).bin

clean:
	rm -f $(OBJS) $(DEPS) $(ASMS) $(PREP) $(PRJ_NAME).elf $(PRJ_NAME)_sh.elf $(PRJ_NAME).d $(PRJ_NAME).map $(PRJ_NAME).asm

