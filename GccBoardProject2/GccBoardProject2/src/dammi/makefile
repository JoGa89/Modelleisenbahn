#*******************************************************************************
#    Copyright (C) 2006, 2011,2012 Olaf Funke, Martin Pischky
#
#    This library is free software; you can redistribute it and/or
#    modify it under the terms of the GNU Lesser General Public
#    License as published by the Free Software Foundation; either
#    version 2.1 of the License, or (at your option) any later version.
#
#    This library is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#    Lesser General Public License for more details.#
#
#    You should have received a copy of the GNU Lesser General Public
#    License along with this library; if not, write to the Free Software
#    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
#    Base on WinAVR Sample makefile written by Eric B. Weddington, 
#    Jörg Wunsch, et al.
#
#    $Id: makefile,v 1.18 2013/11/18 22:44:29 pischky Exp $
#
#    On command line:
#
#    make allprocessors     = Make software for all processors
#
#    make all               = Make software for current processors (ATmega328p)
#
#    make MCU=atmega168 all = Make software for ATmega168.
#
#    make wbl               = Make hex file (PROGRAM + CHECKSUM + BOOTLOADER)
#
#    make dmf               = Make file fpr JMRI update
#
#    make elf               = Make elf file
#
#    make hex               = Make hex file (PROGRAM only)
#
#    make chk               = Make hex file (PROGRAM + CHECKSUM)
#
#    make eep               = Make hex file for EEPROM 
#
#    make lss               = Make lss file
#
#    make sym               = Make sym file
#
#    make fuses-hex         = Make hex files for FUSES
#
#    make boot              = Make hex files for BOOTLOADER
#
#    make coff              = Make coff file
#
#    make extcoff           = Make extcoff file
#
#    make program           = Program flash memory (PROGRAM + CHECKSUM + 
#                                                   BOOTLOADER)
#
#    make progboot          = Program flash memory bith bootloader only
#
#    make progall           = Program flash memory, eeprom and fuses
#
#    make progeeprom        = Program eeprom
#
#    make progfuses         = Program fuses#
#*******************************************************************************

# MCU name: "atmega8", "atmega168", "atmega168a", "atmega168p"
# may be overwritten on commandline
MCU = atmega328p

# clock freq.: 7.3728 MHz (see sysdef.h)
F_CPU = 7372800

# version number (see sysdef.h). Update this three values before a new release!!
# use "trunk", "1.5", "1.6", "1.7", "1.8", ...
VERSION   = trunk
# use "0x0000" = trunk, "0x0106" = 1.6, "0x0107" = 1.7, ...
SW_INDEX  = 0x0000
# use "0" = trunk, "16" = 1.6, "17" = 1.7, "18"=1.8, ...
SOFTWARE_VERSION = 0

# Other directories used. This assumes that the embedded loconet projects are
# checked out side by side.

# where *.[ch] file for loconet are found
LOCONET_DIR = ../../loconet
# other *c. and *.h files like timer
AVR_BASE_DIR = ../../avr-base
# BootloaderUser.[ch] and bootldr_checksum.mk
BOOTLOADER_DIR = ../BootLoader

vpath %.h $(LOCONET_DIR):$(AVR_BASE_DIR):$(BOOTLOADER_DIR):.
vpath %.c $(LOCONET_DIR):$(AVR_BASE_DIR):$(BOOTLOADER_DIR):.

# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
EXTRAINCDIRS = $(LOCONET_DIR) $(AVR_BASE_DIR) $(BOOTLOADER_DIR)

# date of compilation (see sysdef.h)
SW_YEAR   = 0x$(shell date +%y)
SW_MONTH  = 0x$(shell date +%m)
SW_DAY    = 0x$(shell date +%d)

# hardware version required by this application
# depend on MCU only on FREDI
ifeq ($(MCU),atmega8)
    HARDWARE_VERSION     = 1
else ifeq ($(MCU),atmega88)
    HARDWARE_VERSION     = 2
else ifeq ($(MCU),atmega168)
    HARDWARE_VERSION     = 3
else ifeq ($(MCU),atmega328p)
    HARDWARE_VERSION     = 4
else
    $(error MCU=$(MCU) is unknown, could not define HARDWARE_VERSION)
endif

# hardware version must match with  
#  2 - equal (force excact match)
HARDWARE_VERS_TEST   = 2

# MANUFACTURER_ID: "NMRA DIY DCC"
MANUFACTURER_ID     = 13

# DEVELOPER_ID: "FREMO"
DEVELOPER_ID        =  1

# PRODUCT_ID: 11="FREDI Throttle" (Martin Pischky)
# see: http://embeddedloconet.sourceforge.net/device_ids/fremo.en.html
PRODUCT_ID          = 11

# entry point of bootloader for triggering sw update from application
ifeq ($(MCU),atmega8)
    # start address: 0x0F00
    BOOTSTRAP_START_BYTE = 0x1E00
else ifeq ($(MCU),atmega88)
    # boot section size 256 words / start address: 0x0F00
    #BOOTSTRAP_START_BYTE = 0x1E00
    # boot section size 512 words / start address: 0x0E00
    BOOTSTRAP_START_BYTE = 0x1C00
else ifeq ($(MCU),atmega168)
    # boot section size 256 words / start address: 0x1F00
    #BOOTSTRAP_START_BYTE = 0x3E00
    # boot section size 512 words / start address: 0x1E00
    BOOTSTRAP_START_BYTE = 0x3C00
else ifeq ($(MCU),atmega328p)
    # boot section size 256 words / start address: 0x1F00
    #BOOTSTRAP_START_BYTE = 0x7E00
    # boot section size 512 words / start address: 0x3E00
    BOOTSTRAP_START_BYTE = 0x7C00
else
    $(error MCU=$(MCU) is unknown, could not define BOOTSTRAP_START_BYTE)
endif

# options for update by bootloader 
# JMRI ignores this, but allows user to set this options
#   select one of this software options ...
DO_NOT_CHECK_SOFTWARE_VERSION        = 0x00
CHECK_SOFTWARE_VERSION_LESS          = 0x04
#   ... and one of this hardware options ...
DO_NOT_CHECK_HARDWARE_VERSION        = 0x00
REQUIRE_HARDWARE_VERSION_EXACT_MATCH = 0x01
ACCEPT_LATER_HARDWARE_VERSIONS       = 0x03
#   ... to build the options:
UPDATE_OPTIONS = ( $(CHECK_SOFTWARE_VERSION_LESS) | $(REQUIRE_HARDWARE_VERSION_EXACT_MATCH) )

# Target file name (without extension).
TARGETBASE  = fredi

# Create hex (and other output) including processor and version
TARGET      = $(TARGETBASE)-$(VERSION)-$(MCU)_pgm
TARGET_FUSE = $(TARGETBASE)-$(VERSION)-$(MCU)

# parameters for bootldr_checksum.mk:
APPLICATION_NAME = FREDI $(VERSION)
APPLICATION_HEX = $(TARGET).hex
APPLICATION_WITH_CHECKSUM = $(TARGETBASE)-$(VERSION)-$(MCU)_checksum.hex
APPLICATION_WITH_BL = $(TARGETBASE)-$(VERSION)-$(MCU).hex
LOADER_FILE = $(TARGETBASE)-$(VERSION)-$(MCU).dmf
BOOTLOADER_HEX = BL/BootLoader-$(MCU).hex

# List C source files here. (C dependencies are automatically generated.)
SRC = fredi.c

# If there is more than one source file, append them above, or modify and
# uncomment the following:
SRC += systimer.c 
SRC += ln_sw_uart.c
SRC += utils.c 
SRC += ln_buf.c
SRC += ln_interface.c
SRC += fuses.c
SRC += potAdc.c
SRC += keyInp.c
SRC += BootloaderUser.c

# List Assembler source files here.
# Make them always end in a capital .S.  Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC = 

# Output format. (can be srec, ihex)
FORMAT = ihex

# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s

# Compiler flag to set the C Standard level.
CSTANDARD_C89 = c89
CSTANDARD_GNU89 = gnu89
CSTANDARD_C99 = c99
CSTANDARD_GNU99 = gnu99
CSTANDARD = -std=$(CSTANDARD_GNU99)

# Compiler flags.
#  -g:           generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -g
CFLAGS += -O$(OPT)
# CFLAGS += -funsigned-char
CFLAGS += -D SW_INDEX=$(SW_INDEX)
CFLAGS += -D SW_YEAR=$(SW_YEAR)
CFLAGS += -D SW_MONTH=$(SW_MONTH)
CFLAGS += -D SW_DAY=$(SW_DAY)
CFLAGS += -D HARDWARE_VERSION=$(HARDWARE_VERSION)
CFLAGS += -D HARDWARE_VERS_TEST=$(HARDWARE_VERS_TEST)
CFLAGS += -D SOFTWARE_VERSION=$(SOFTWARE_VERSION)
CFLAGS += -D MANUFACTURER_ID=$(MANUFACTURER_ID)
CFLAGS += -D DEVELOPER_ID=$(DEVELOPER_ID)
CFLAGS += -D PRODUCT_ID=$(PRODUCT_ID)
CFLAGS += -D BOOTSTRAP_START_BYTE=$(BOOTSTRAP_START_BYTE)
# CFLAGS += -DLOCONET_LEVEL_TEST
CFLAGS += -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -Wall -Wstrict-prototypes
CFLAGS += -Wa,-adhlns=$(<:.c=.lst)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
CFLAGS += $(CSTANDARD)

# Assembler flags.
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -ahlms:    create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
ASFLAGS = -Wa,-adhlns=$(<:.S=.lst),-gstabs 

# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref

# Programming support using avrdude. Settings and variables.

# Programming hardware: alf avr910 avrisp bascom bsd 
# dt006 pavr picoweb pony-stk200 sp12 stk200 stk500
#
# Type: avrdude -c ?
# to get a full listing.
#
AVRDUDE_PROGRAMMER = stk500v2

# com1 = serial port. Use lpt1 to connect to parallel port.
AVRDUDE_PORT = com1
#AVRDUDE_PORT = lpt1

AVRDUDE_BAUDRATE = -b 115200

AVRDUDE_WRITE_FLASH  = -U flash:w:$(APPLICATION_WITH_BL):i
AVRDUDE_WRITE_BOOT   = -U flash:w:$(BOOTLOADER_HEX):i
AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep:i

ifeq ($(MCU),atmega8)
     AVRDUDE_WRITE_FUSES = -U hfuse:w:$(TARGET_FUSE)-hfuses.hex:i \
                           -U lfuse:w:$(TARGET_FUSE)-lfuses.hex:i
else
     AVRDUDE_WRITE_FUSES = -U hfuse:w:$(TARGET_FUSE)-hfuses.hex:i \
                           -U lfuse:w:$(TARGET_FUSE)-lfuses.hex:i \
                           -U efuse:w:$(TARGET_FUSE)-efuses.hex:i
endif

# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> 
# to submit bug reports.
#AVRDUDE_VERBOSE = -v -v

AVRDUDE_FLAGS = -p $(MCU)
AVRDUDE_FLAGS += -P $(AVRDUDE_PORT)
AVRDUDE_FLAGS += $(AVRDUDE_BAUDRATE)
AVRDUDE_FLAGS += -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)

# ------------------------------------------------------------------------------

# Define programs and commands.
SHELL = sh
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
SIZEX = avr-sizex
AVRDUDE = avrdude
REMOVE = rm -f
COPY = cp

# Define Messages
MSG_COFF             = --------------- Converting to AVR COFF:
MSG_EXTENDED_COFF    = ------ Converting to AVR Extended COFF:
MSG_FLASH            = --------- Creating load file for Flash:
MSG_EEPROM           = -------- Creating load file for EEPROM:
MSG_EXTENDED_LISTING = ------------ Creating Extended Listing:
MSG_SYMBOL_TABLE     = ---------------- Creating Symbol Table:
MSG_LINKING          = ------------------------------ linking:
MSG_GENFILE          = ------------------------ creating file:
MSG_PROGRAMMING      = -------------------------- programming:
MSG_COMPILING        = ---------------------------- compiling:
MSG_ASSEMBLING       = --------------------------- assembling:
MSG_CLEANING         = --------------------- cleaning project:

# Define all object files.
OBJ = $(SRC:.c=.o) $(ASRC:.S=.o) 

# Define all listing files.
LST = $(ASRC:.S=.lst) $(SRC:.c=.lst)

# Compiler flags to generate dependency files.
GENDEPFLAGS = -Wp,-M,-MP,-MT,$(*F).o,-MF,.dep/$(@F).d

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -I. -x assembler-with-cpp $(ASFLAGS)

# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT = $(OBJCOPY) --debugging \
              --change-section-address .data-0x800000 \
              --change-section-address .bss-0x800000 \
              --change-section-address .noinit-0x800000 \
              --change-section-address .eeprom-0x810000

# ******************************************************************************

# Default target.
# For update over loconet (JMRI) the target "dmf" is needed  
# For FCalib2 the targets are "wbl" and "fuses-hex"
# "eep", "lss" and "sym" are for debugging only.
all : dmf wbl fuses-hex eep lss sym

# Traget to build for all prozessors.
allprocessors :
	make -f makefile distclean
	make -f makefile MCU=atmega88 all fuses-hex
	make -f makefile clean
	#make -f makefile MCU=atmega88a all fuses-hex
	#make -f makefile clean
	#make -f makefile MCU=atmega88p all fuses-hex
	#make -f makefile clean
	make -f makefile MCU=atmega168 all fuses-hex
	make -f makefile clean
	#make -f makefile MCU=atmega168a all fuses-hex
	#make -f makefile clean
	#make -f makefile MCU=atmega168p all fuses-hex
	#make -f makefile clean
	#make -f makefile MCU=atmega328 all fuses-hex
	#make -f makefile clean
	make -f makefile MCU=atmega328p all fuses-hex
	make -f makefile clean
	make -f makefile MCU=atmega8 all fuses-hex
	make -f makefile clean

# Target to build 
elf : $(TARGET).elf
hex : $(TARGET).hex
dmf : $(LOADER_FILE)
wbl : $(APPLICATION_WITH_BL) 
eep : $(TARGET).eep
lss : $(TARGET).lss 
sym : $(TARGET).sym
chk : $(APPLICATION_WITH_CHECKSUM)
fuses-hex : $(TARGET_FUSE)-lfuses.hex \
            $(TARGET_FUSE)-hfuses.hex \
            $(TARGET_FUSE)-efuses.hex
boot : $(BOOTLOADER_HEX)

$(TARGET_FUSE)-lfuses.hex : $(TARGET).elf
	-$(OBJCOPY) -j lfuses --change-section-address lfuses=0 \
	-O ihex $(TARGET).elf $(TARGET_FUSE)-lfuses.hex

$(TARGET_FUSE)-hfuses.hex : $(TARGET).elf
	-$(OBJCOPY) -j hfuses --change-section-address hfuses=0 \
	-O ihex $(TARGET).elf $(TARGET_FUSE)-hfuses.hex

# generates "empty" hex file for ATmega8 
$(TARGET_FUSE)-efuses.hex : $(TARGET).elf
	-$(OBJCOPY) -j efuses --change-section-address efuses=0 \
	-O ihex $(TARGET).elf $(TARGET_FUSE)-efuses.hex

# rules for $(TARGETBASE)-$(VERSION)-$(MCU).hex 
# and $(TARGETBASE)-$(VERSION)-$(MCU).dmf
include $(BOOTLOADER_DIR)/bootldr_checksum.mk

# make bootloader
$(BOOTLOADER_HEX) : BL/BootLoader.asm BL/makefile makefile
	make -C BL \
	MANUFACTURER_ID=$(MANUFACTURER_ID) \
	DEVELOPER_ID=$(DEVELOPER_ID) \
	PRODUCT_ID=$(PRODUCT_ID) \
	HARDWARE_VERS_TEST=$(HARDWARE_VERS_TEST) \
	BOOTSTRAP_START_BYTE=$(BOOTSTRAP_START_BYTE) \
	MCU=$(MCU) \
	F_CPU=$(F_CPU)

coff : $(TARGET).elf
	@echo
	@echo $(MSG_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-avr $< $(TARGET).cof

extcoff : $(TARGET).elf
	@echo
	@echo $(MSG_EXTENDED_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-ext-avr $< $(TARGET).cof

# Program the device with program, bootloader and checksum.  
program : $(APPLICATION_WITH_BL)
	@echo
	@echo $(MSG_PROGRAMMING) $?
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)

# Program the device.  
progall : $(APPLICATION_WITH_BL) $(TARGET).eep fuses-hex
	@echo
	@echo $(MSG_PROGRAMMING) $?
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) $(AVRDUDE_WRITE_FUSES)

progboot : $(BOOTLOADER_HEX)
	@echo
	@echo $(MSG_PROGRAMMING) $?
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_BOOT)

progeeprom : $(TARGET).eep
	@echo
	@echo $(MSG_PROGRAMMING) $?
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_EEPROM)

# Program the device.  
progfuses : fuses-hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FUSES)

# Create final output files (.hex, .eep) from ELF output file.
%.hex : %.elf makefile
	@echo
	@echo $(MSG_GENFILE) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom -R efuses -R hfuses -R lfuses $< $@
	
%.eep : %.elf makefile
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
%.lss : %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
%.sym : %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ) makefile
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(ALL_CFLAGS) $(OBJ) --output $@ $(LDFLAGS)

# Compile: create object files from C source files.
%.o : %.c makefile
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(ALL_CFLAGS) $< -o $@ 

# Compile: create assembler files from C source files.
%.s : %.c makefile
	$(CC) -S $(ALL_CFLAGS) $< -o $@

# Assemble: create object files from assembler source files.
%.o : %.S makefile
	@echo
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@

# Target: clean project.
clean : clean_list 

# clean project and any distribution files.
distclean : clean_list clean_list_output

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	-$(REMOVE) $(OBJ)
	-$(REMOVE) $(LST)
	-$(REMOVE) $(SRC:.c=.s)
	-$(REMOVE) $(SRC:.c=.d)
	-$(REMOVE) .dep/*
	make -C BL clean

clean_list_output :
	@echo
	@echo $(MSG_CLEANING)
	-$(REMOVE) $(TARGETBASE)*.hex
	-$(REMOVE) $(TARGETBASE)*.dmf
	-$(REMOVE) $(TARGETBASE)*.eep
	-$(REMOVE) $(TARGETBASE)*.obj
	-$(REMOVE) $(TARGETBASE)*.cof
	-$(REMOVE) $(TARGETBASE)*.elf
	-$(REMOVE) $(TARGETBASE)*.map
	-$(REMOVE) $(TARGETBASE)*.obj
	-$(REMOVE) $(TARGETBASE)*.a90
	-$(REMOVE) $(TARGETBASE)*.sym
	-$(REMOVE) $(TARGETBASE)*.lnk
	-$(REMOVE) $(TARGETBASE)*.lss
	make -C BL distclean

# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# Listing of phony targets.
.PHONY : all allprocessors elf hex dmf wbl eep lss sym chk fuses-hex \
         boot program progall progeeprom progfuses coff extcoff \
         clean distclean clean_list clean_list_output
