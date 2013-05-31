# Compiles firmware written in C and assembler for NXP's LPC chips
# Copyright (C) 2013  Richard Meadows
# 
# Permission is hereby granted, free of charge, to any person obtaining
# a copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
# 
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
# LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#
# The primary targets in this file are:
#
# [none]	Compiles the source to create an .elf in the output directory
# sources	Creates sources.mk from all the .c files in the src directory
# download	Compiles and downloads over lpc-link
# lpc-link 	Blocking - Initialises an lpc-link device and acts as a debug server
# clean		Removes generated files
#
# This makefile is intended to be run from the root of the project.
#

# External makefile.conf
#
# Edit the project name, chip, includes directories and so on in this file.
#
-include makefile.conf

# Directories
#
# These define the locations of the source and output trees.
#
OUTPUT_DIR	:= out
SOURCE_DIR	:= src

# Shell Commands
#
# Listed here for portability.
#
ECHO	:= echo
FIND	:= find
GREP	:= grep
MKDIR	:= mkdir -p
RM	:= rm -r
SED	:= sed

# ARM GNU Toolchain
#
# These tools are available from https://launchpad.net/gcc-arm-embedded/ and
# should be placed on your path. ALternatively you could compile your own.
#
TARGET  := arm-none-eabi
AS	:= $(TARGET)-as
CC	:= $(TARGET)-gcc
CXX	:= $(TARGET)-g++
OBJCOPY	:= $(TARGET)-objcopy
SIZE	:= $(TARGET)-size

# Download Tools
#
# Binaries supplied with LPCXpresso used for downloading.
#
LPCINSTALL	:= /usr/local/lpcxpresso_4.2.3_255/lpcxpresso/bin/
CHECKSUM	:= $(LPCINSTALL)checksum
LPCLINK		:= $(LPCINSTALL)$(DEBUG)
DFUUTIL		:= $(LPCINSTALL)dfu-util
DFUFIRMWARE	:= $(LPCINSTALL)LPCXpressoWIN.enc

# Compilation Flags
#
# Display all warnings. Compile functions and data into their own sections so
# they can be discarded if unused.  The linker performs garbage collection of
# unused input sections.
#
CFLAGS	= $(FLAGS) -Wall -Wextra -std=gnu99 -ffunction-sections -fdata-sections $(ARCH_FLAGS)
ASFLAGS	= $(FLAGS) -Wall $(ARCH_FLAGS)
LDFLAGS = $(FLAGS) $(LINKER_FLAGS) -Wextra $(ARCH_FLAGS)

# Default target
all: $(OUTPUT_DIR)/$(PROJECT_NAME).elf

# Create a definitive list of sources for the project by
# combining OTHER_SOURCES with sources.mk
SOURCES := $(OTHER_SOURCES)
include sources.mk
# Translate this list of sources into a list of required objects
# in the output directory
objects = $(patsubst %.c,%.o,$(patsubst %.S,%.o,$(SOURCES)))
OBJECTS = $(addprefix $(OUTPUT_DIR)/,$(objects))

# Rule for generating object and dependancy files from source files
#
# Creates a directory in the output tree if nessesary. File is only compiled,
# not linked. Dependancy generation is automatic, but only for user header
# files. Every depandancy in the .d is appended to the .d as a target, so that
# if they stop existing the corresponding object file will be re-compiled.
#
$(OUTPUT_DIR)/%.o: %.c
	@echo
	@echo 'Compiling $<...'
	@$(MKDIR) $(OUTPUT_DIR)/$(dir $<)
	$(CC) -c -MMD $(CPPFLAGS) $(CFLAGS) $(addprefix -I,$(INCLUDES)) -o $@ $<
	@$(SED) -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' -e '/^$$/ d' -e 's/$$/ :/' < $(OUTPUT_DIR)/$*.d >> $(OUTPUT_DIR)/$*.d;

# Attempt to include the dependany makefiles for every object in this makefile.
#
# This means that object files depend on the header files they include.
#
-include $(OBJECTS:.o=.d)

# Rule for generating object files from assembler files
#
# Creates a directory in the output tree if nessesary. The file is only
# assembled, not linked.
#
$(OUTPUT_DIR)/%.o: %.s
	@echo
	@echo 'Assembling $<...'
	@$(MKDIR) $(OUTPUT_DIR)/$(dir $<)
	$(AS) $(ASFLAGS) -o $@ $<

# Generate the main build artifact.
#
# A .elf containing all the symbols (i.e. debugging information if the compiler
# / linker was run with -g) is created, alongside .hex and .bin files. A just
# about human-readable .map is also created.
#
$(OUTPUT_DIR)/$(PROJECT_NAME).elf: $(OBJECTS) $(LINKERS)
	@echo
	@echo 'Linking $@...'
	$(CC) $(LDFLAGS) $(addprefix -T,$(LINKERS)) -Wl,-Map,$(@:.elf=.map) -o $@ $(OBJECTS)
	@$(OBJCOPY) -O binary $@ $(@:.elf=.bin)
	@$(OBJCOPY) -O ihex $@ $(@:.elf=.hex)
	@echo
	$(SIZE) $@
	@echo
	@$(SIZE) $@|tail -1 -|awk '{print "ROM Usage: "int($$1/10.24)/100"K / $(ROM_SIZE)"}'
	@$(SIZE) $@|tail -1 -|awk '{print "RAM Usage: "int($$2/10.24)/100"K / $(RAM_SIZE)"}'

# Creates sources.mk
#
# All C and S files in the sources directory are compiled into a makefile.  This
# makefile should be audited to check that only the required code is linked into
# the build.
#
.PHONY: sources
sources:
	@echo 'Building sources.mk...' 
	@echo
	@$(FIND) $(SOURCE_DIR)/ | $(GREP) \\.[cS]$ > sources.mk
	@cat sources.mk
	@$(SED) -i '1s/^/SOURCES += /' sources.mk
	@$(SED) -i 's/$$/ \\/' sources.mk

# Downloads the firmware over lpclink
#
# The 'symbol-file' command in the .gdbscript has the correct symbol file
# written to it.
#
.PHONY: download
download: all gdbscript
	@$(SED) -i 's/^file.*$$/file $(OUTPUT_DIR)\/$(PROJECT_NAME)\.elf/' gdbscript
	$(CHECKSUM) -p $(CHIP) -d $(OUTPUT_DIR)/$(PROJECT_NAME).bin
	@echo
	@echo
	$(LPCLINK) -wire=winusb -p$(CHIP) -vendor=NXP -flash-load-exec=$(OUTPUT_DIR)/$(PROJECT_NAME).bin -g

# Creates a gdb script if required
#
#
#
gdbscript:
	@$(ECHO) "# Load our .elf file into GDB" >> gdbscript
	@$(ECHO) "file" > gdbscript
	@$(ECHO) "# Define a target description to override the lpc-link default" >> gdbscript
	@$(ECHO) "set tdesc filename arm-core.xml" >> gdbscript
	@$(ECHO) "# Required for semihosting" >> gdbscript
	@$(ECHO) "set mem inaccessible-by-default off" >> gdbscript
	@$(ECHO) "# Connect to the debug server launched by make lpc-link" >> gdbscript
	@$(ECHO) "target extended-remote" >> gdbscript
	@$(ECHO) "# Enable semihosting" >> gdbscript
	@$(ECHO) "monitor semihosting enable" >> gdbscript

# Flashes the firmware to the lpc-link and starts the debug server
#
# Blocking. A random port is used and substituted into the .gdbscript
#
.PHONY: lpc-link
lpc-link: gdbscript
	$(eval PORT := $(shell shuf -i 2000-65000 -n 1))
	@$(SED) -i 's/^target extended-remote.*$$/target extended-remote :$(PORT)/' gdbscript
	$(DFUUTIL) -d 0x471:0xdf55 -c 0 -t 2048 -R -D $(DFUFIRMWARE)
	$(LPCLINK) -wire=winusb -p$(CHIP) -vendor=NXP -server=:$(PORT)

# Removes everything in the output directory
#
#
#
.PHONY: clean
clean:
	$(RM) $(OUTPUT_DIR)/*
	$(RM) gdbscript