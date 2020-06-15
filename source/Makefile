##############################################################################
# Makefile for baremetal compiling
#
# Use: make [TARGET]
#
# Build Targets:
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

# Target.elf 
TARGET = final

CC = arm-none-eabi-gcc # Compiler
LD = arm-none-eabi-ld # Loader
SIZE = arm-none-eabi-size	
OBJDMP = arm-none-eabi-objdump

MACH=cortex-m4

# General GCC compiler flags for both platforms
CFLAGS = -Wall \
	-g \
	-O0 \
	-std=gnu11

CPPFLAGS = -MMD -MP $(INCLUDES)

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

LDFLAGS = --specs=$(SPECS) -T$(LINKER_FILE) -Wl,-Map=$(TARGET).map

#LDFLAGS_SH = --specs=rdimon.specs -T$(LINKER_FILE) -Wl,-Map=$(TARGET).map


all: $(TARGET).elf

#semi: main.o led.o stm32_startup.o $(TARGET)_sh.elf

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

$(TARGET).elf: $(OBJS)
	$(CC) $^ $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) -o $@ # instead of $(OBJS), uses $^
	$(SIZE) $< $@
	
#$(TARGET)_sh.elf: main.o led.o stm32_startup.o
#	$(CC) $(CFLAGS) $(CPPFLAGS) $(LDFLAGS_SH) -o $@ $^ 
#	$(SIZE) $< $@

$(TARGET).asm: $(TARGET).elf
	$(OBJDMP) --disassemble-all $(TARGET).elf > $(TARGET).asm

clean:
	rm -f $(OBJS) $(DEPS) $(ASMS) $(PREP) $(TARGET).elf $(TARGET)_sh.elf $(TARGET).d $(TARGET).map $(TARGET).asm

load:
	openocd -f board/stm32f4discovery.cfg 


