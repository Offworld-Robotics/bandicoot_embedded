# Makefile for feedback_io project
#
# Author: Aaron Lucas
# Date Created: 2020/07/27
#
# Written for the Off-World Robotics Team

# ==============================================================================
# Directory Definitions
# ==============================================================================

# Executable output files
OUT_DIR=bin

# Library archives
LIB_DIR=lib

# Source code
SRC_DIR=src

# Source code for testing programs and other testing files
TEST_DIR=test

# Object files
OBJ_DIR=obj

# ==============================================================================
# Project Definitions
# ==============================================================================

ELFS=blink pwmTest simulateMotor

TIVAWARE=$(SRC_DIR)/tivaware
DRIVERLIB=$(TIVAWARE)/driverlib

INC_DIRS=$(TIVAWARE) src
INC_FLAGS=$(patsubst %,-I%,$(INC_DIRS))

LINKER_SCRIPT=linker.ld

STARTUP_SCRIPT=$(SRC_DIR)/startup.c
STARTUP_OBJ=$(OBJ_DIR)/startup.o

# _DEPS=pwmControl.c qeiControl.c
# DEPS=$(patsubst %,$(SRC_DIR)/%,$(_DEPS))

# ==============================================================================
# Chip-Specific Configuration
# ==============================================================================

PART=TM4C123GH6PM

CPU=-mcpu=cortex-m4
FPU=-mfpu=fpv4-sp-d16 -mfloat-abi=hard

# ==============================================================================
# Toolchain Binaries
# ==============================================================================

PREFIX=arm-none-eabi

CC=$(PREFIX)-gcc
LD=$(PREFIX)-ld
AS=$(PREFIX)-as
AR=$(PREFIX)-ar

# ==============================================================================
# Toolchain Flags
# ==============================================================================

# Flag explanations
# ------------------
# -mthumb: 				16-bit thumb instructions for smaller code size
# $(CPU), $(FPU):		Tell compiler which CPU is on the chip and configure FPU
# -ffunction-sections: 	Put each function in its own section
# -fdata-sections:		Put each data item in its own section
# -MD:					Auto-generate dependency files
# -std:					Set C language standard for compilation
# -W					Show all warnings and make them errors
# -DPART_:				Define part number for tivaware library
# -c:					Compile and assemble into object files
# $(INC_FLAGS):			Search for header files in non-standard directories

CFLAGS=-mthumb $(CPU) $(FPU) -ffunction-sections -fdata-sections -MD -std=c99 \
	-Wall  -Wpedantic -DPART_$(PART) -c $(INC_FLAGS) -Dgcc

LDFLAGS=--gc-sections

ASFLAGS=-mthumb $(CPU) $(FPU) -MD $(INC_FLAGS)


# ==============================================================================
# Libraries
# ==============================================================================

# GCC low-level runtime library
LIBGCC=$(shell $(CC) $(CFLAGS) -print-libgcc-file-name)

# Standard C library
LIBC=$(shell $(CC) $(CFLAGS) -print-file-name=libc.a)

# Math library
LIBM=$(shell $(CC) $(CFLAGS) -print-file-name=libm.a)

# Tivaware driver library
LIBDRIVER=$(LIB_DIR)/libdriver.a
# LIBDRIVER=$(TIVAWARE)/driverlib/gcc/libdriver.a

# All libraries
LIBS=$(LIBGCC) $(LIBC) $(LIBM) $(LIBDRIVER)

# ==============================================================================
# Make Rules
# ==============================================================================

COMMON_DEPS=$(LIBDRIVER) $(STARTUP_OBJ) $(OBJ_DIR)/common.o

.PHONY: all debug clean cleanlib

all: $(patsubst %,$(OUT_DIR)/%.elf,$(ELFS))

debug: CFLAGS+=-g -DDEBUG -O0
debug: all

# Rule to create startup object file
# Uses different set of flags
$(OBJ_DIR)/startup.o: $(SRC_DIR)/startup.c | $(OBJ_DIR)
	$(CC) $(ASFLAGS) -c -o $@ $<

# Rule to create object files from C files in source directory
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -o $@ $<

# Rule to create object files from C files in test directory
$(OBJ_DIR)/%.o: $(TEST_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -o $@ $<

# Rule to create tivaware driver library object files
$(OBJ_DIR)/%.o: $(DRIVERLIB)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -o $@ $<

# Rule to create driver library archive
# Object files are created in tivaware folder so the library only needs to be
# built once.
DRIVERLIB_SRC=$(wildcard $(DRIVERLIB)/*.c)
$(LIBDRIVER): $(patsubst $(DRIVERLIB)/%.c,$(DRIVERLIB)/%.o,$(DRIVERLIB_SRC)) | $(LIB_DIR)
	$(AR) -cr $@ $^

# Rules to create executable ELF files

# Generic
$(OUT_DIR)/%.elf: $(OBJ_DIR)/%.o $(COMMON_DEPS) | $(OUT_DIR)
	$(LD) -T $(LINKER_SCRIPT) $(LDFLAGS) -o $@ $^ $(LIBS)

# Specialised
_PWM_TEST_DEPS=pwmTest PWMControl
PWM_TEST_DEPS=$(patsubst %,$(OBJ_DIR)/%.o,$(_PWM_TEST_DEPS))
$(OUT_DIR)/pwmTest.elf: $(PWM_TEST_DEPS) $(COMMON_DEPS) | $(OUT_DIR)
	$(LD) -T $(LINKER_SCRIPT) $(LDFLAGS) -o $@ $^ $(LIBS)

_SIM_MOTOR_DEPS=simulateMotor Motor PIDController fix_t
SIM_MOTOR_DEPS=$(patsubst %,$(OBJ_DIR)/%.o,$(_SIM_MOTOR_DEPS))
$(OUT_DIR)/simulateMotor.elf: $(SIM_MOTOR_DEPS) $(COMMON_DEPS) | $(OUT_DIR)
	$(LD) -T $(LINKER_SCRIPT) $(LDFLAGS) -o $@ $^ $(LIBS)

$(OBJ_DIR): 
	mkdir -p $@
$(OUT_DIR): 
	mkdir -p $@
$(LIB_DIR): 
	mkdir -p $@

clean:
	rm -rf $(OBJ_DIR) $(OUT_DIR)

cleanlib:
	rm -rf $(LIB_DIR)

