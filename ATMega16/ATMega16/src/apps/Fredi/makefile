# WinAVR Sample makefile written by Eric B. Weddington, Jörg Wunsch, et al.
# Released to the Public Domain
# Please read the make user manual!
#
# Additional material for this makefile was submitted by:
#  Tim Henigan
#  Peter Fleury
#  Reiner Patommel
#  Sander Pool
#  Frederik Rouleau
#  Markus Pfaff
#
# On command line:
#
# make all               = Make software (including fuses files).
#
# make MCU=atmega168 all = Make software for ATmega168.
#
# make clean             = Clean out built project files.
#
# make coff              = Convert ELF to AVR COFF (for use 
#                          with AVR Studio 3.x or VMLAB).
#
# make extcoff           = Convert ELF to AVR Extended COFF 
#                          (for use with AVR Studio 4.07 or greater).
#
# make program           = Download the hex file to the device, using avrdude.
#                          Please customize the avrdude settings below first!
#
# make progfuses         = Program fuses using avrdude.
#
# make progeeprom        = Program eeprom using avrdude.
#
# make progall           = Program flash, eeprom and fuses using avrdude.
#
# make filename.s        = Just compile filename.c into the assembler code only
#
# make allprocessors     = build output files for all processors
#
# To rebuild project do "make clean" then "make all".
#
# $Id: makefile,v 1.11 2011/09/08 10:05:27 pischky Exp $

LOCONET_DIR = ../../loconet
AVR_BASE_DIR = ../../avr-base

vpath %.h $(LOCONET_DIR):$(AVR_BASE_DIR)
vpath %.c $(LOCONET_DIR):$(AVR_BASE_DIR)

# MCU name: "atmega8", "atmega168", "atmega168a", "atmega168p"
MCU = atmega328p

# clock freq.: 7.3728 MHz (see sysdef.h)
# F_CPU = 7372800

# Output format. (can be srec, ihex, binary)
FORMAT = ihex

# version number (see sysdef.h)
# "1.5", "1.6", "trunk", ...
VERSION   = 1.7
# "0x0000" = trunk, "0x0106" = 1.6
SW_INDEX  = 0x0107

# date of compilation (see sysdef.h)
SW_YEAR   = 0x$(shell date +%y)
SW_MONTH  = 0x$(shell date +%m)
SW_DAY    = 0x$(shell date +%d)

# Target file name (without extension).
TARGETBASE = fredi

# Create hex (and other output) including processor and version
TARGET = $(TARGETBASE)-$(VERSION)-$(MCU)

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
SRC += potadc.c



# List Assembler source files here.
# Make them always end in a capital .S.  Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its command-line.
ASRC = 



# Optimization level, can be [0, 1, 2, 3, s]. 
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s


# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
EXTRAINCDIRS = $(LOCONET_DIR) $(AVR_BASE_DIR) 

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
CFLAGS += -DSW_INDEX=$(SW_INDEX) -DSW_YEAR=$(SW_YEAR) -DSW_MONTH=$(SW_MONTH) -DSW_DAY=$(SW_DAY)
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



#Additional libraries.

PRINTF_LIB_NONE = 

# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

PRINTF_LIB = $(PRINTF_LIB_NONE)

MATH_LIB = -lm



# Linker flags.
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref $(PRINTF_LIB) $(MATH_LIB)




# Programming support using avrdude. Settings and variables.

# Programming hardware: alf avr910 avrisp bascom bsd 
# dt006 pavr picoweb pony-stk200 sp12 stk200 stk500
#
# Type: avrdude -c ?
# to get a full listing.
#
AVRDUDE_PROGRAMMER = stk500

# com1 = serial port. Use lpt1 to connect to parallel port.
AVRDUDE_PORT = com1    # programmer connected to serial device
#AVRDUDE_PORT = lpt1	# programmer connected to parallel port

AVRDUDE_BAUDRATE = -b 115200

#AVRDUDE_WRITE_FUSES = -U hfuse:w:0xD1:m -U lfuse:w:0xBF:m
ifeq ($(MCU),atmega8)
     AVRDUDE_WRITE_FUSES = -U hfuse:w:$(TARGET)-hfuses.hex \
                           -U lfuse:w:$(TARGET)-lfuses.hex
else
     AVRDUDE_WRITE_FUSES = -U hfuse:w:$(TARGET)-hfuses.hex \
                           -U lfuse:w:$(TARGET)-lfuses.hex \
                           -U efuse:w:$(TARGET)-efuses.hex
endif


AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep


# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
# AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> 
# to submit bug reports.
#AVRDUDE_VERBOSE = -v -v

AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)
AVRDUDE_FLAGS += $(AVRDUDE_BAUDRATE)

# ---------------------------------------------------------------------------

# Define directories, if needed.
# DIRAVR = c:/winavr
DIRAVR = D:/WinAVR-20100110
DIRAVRBIN = $(DIRAVR)/bin
DIRAVRUTILS = $(DIRAVR)/utils/bin
DIRINC = .
DIRLIB = $(DIRAVR)/avr/lib


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
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = -------- begin --------
MSG_END = --------  end  --------
MSG_SIZE_BEFORE = Size before: 
MSG_SIZE_AFTER = Size after:
MSG_COFF = Converting to AVR COFF:
MSG_EXTENDED_COFF = Converting to AVR Extended COFF:
MSG_FLASH = Creating load file for Flash:
MSG_EEPROM = Creating load file for EEPROM:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling:
MSG_ASSEMBLING = Assembling:
MSG_CLEANING = Cleaning project:




# Define all object files.
OBJ = $(SRC:.c=.o) $(ASRC:.S=.o) 

# Define all listing files.
LST = $(ASRC:.S=.lst) $(SRC:.c=.lst)


# Compiler flags to generate dependency files.
GENDEPFLAGS = -Wp,-M,-MP,-MT,$(*F).o,-MF,.dep/$(@F).d


# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mmcu=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)


# Default target.
all: begin gccversion sizebefore build sizeafter finished end

allprocessors:
	make -f makefile clean
	make -f makefile MCU=atmega88 all fuses-hex
	make -f makefile cleanintermediate
	#make -f makefile MCU=atmega88a all fuses-hex
	#make -f makefile cleanintermediate
	#make -f makefile MCU=atmega88p all fuses-hex
	#make -f makefile cleanintermediate
	make -f makefile MCU=atmega168 all fuses-hex
	make -f makefile cleanintermediate
	#make -f makefile MCU=atmega168a all fuses-hex
	#make -f makefile cleanintermediate
	#make -f makefile MCU=atmega168p all fuses-hex
	#make -f makefile cleanintermediate
	#make -f makefile MCU=atmega328 all fuses-hex
	#make -f makefile cleanintermediate
	make -f makefile MCU=atmega328p all fuses-hex
	make -f makefile cleanintermediate
	make -f makefile MCU=atmega8 all fuses-hex
	make -f makefile cleanintermediate

build: elf hex eep fuses-hex lss sym 

elf: $(TARGET).elf
hex: $(TARGET).hex
eep: $(TARGET).eep
lss: $(TARGET).lss 
sym: $(TARGET).sym

$(TARGET)-lfuses.hex: $(TARGET).elf
	-$(OBJCOPY) -j lfuses --change-section-address lfuses=0 \
	-O ihex $(TARGET).elf $(TARGET)-lfuses.hex

$(TARGET)-hfuses.hex: $(TARGET).elf
	-$(OBJCOPY) -j hfuses --change-section-address hfuses=0 \
	-O ihex $(TARGET).elf $(TARGET)-hfuses.hex

# generates "empty" hex file for ATmega8 
$(TARGET)-efuses.hex: $(TARGET).elf
	-$(OBJCOPY) -j efuses --change-section-address efuses=0 \
	-O ihex $(TARGET).elf $(TARGET)-efuses.hex
	
fuses-hex: $(TARGET)-lfuses.hex $(TARGET)-hfuses.hex $(TARGET)-efuses.hex

# Eye candy.
# AVR Studio 3.x does not check make's exit code but relies on
# the following magic strings to be generated by the compile job.
begin:
	@echo
	@echo $(MSG_BEGIN)

finished:
	@echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)
	@echo


# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) -A $(TARGET).elf
FUNSIZE = $(SIZEX) -m $(MCU) -f 8192 -t 500 $(TARGET).elf
sizebefore:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi

functionsize:
	@if [ -f $(TARGET).elf ]; then echo; $(FUNSIZE); echo; fi

# Display compiler version information.
gccversion : 
	@$(CC) --version


# Program the device.  
program: $(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)

# Program the device.  
progall: $(TARGET).hex  $(TARGET).eep $(TARGET).eep fuses-hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) $(AVRDUDE_WRITE_FUSES)

progeeprom: $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_EEPROM)

# Program the device.  
progfuses: fuses-hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FUSES)


# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000 


coff: $(TARGET).elf
	@echo
	@echo $(MSG_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-avr $< $(TARGET).cof


extcoff: $(TARGET).elf
	@echo
	@echo $(MSG_EXTENDED_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-ext-avr $< $(TARGET).cof



# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

%.eep: %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@



# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ)
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
clean: begin clean_list clean_list_output finished end

cleanintermediate: begin clean_list finished end

clean_list :
	@echo
	@echo $(MSG_CLEANING)
	$(REMOVE) $(OBJ)
	$(REMOVE) $(LST)
	$(REMOVE) $(SRC:.c=.s)
	$(REMOVE) $(SRC:.c=.d)
	$(REMOVE) .dep/*

clean_list_output :
	$(REMOVE) $(TARGETBASE)*.hex
	$(REMOVE) $(TARGETBASE)*.eep
	$(REMOVE) $(TARGETBASE)*.obj
	$(REMOVE) $(TARGETBASE)*.cof
	$(REMOVE) $(TARGETBASE)*.elf
	$(REMOVE) $(TARGETBASE)*.map
	$(REMOVE) $(TARGETBASE)*.obj
	$(REMOVE) $(TARGETBASE)*.a90
	$(REMOVE) $(TARGETBASE)*.sym
	$(REMOVE) $(TARGETBASE)*.lnk
	$(REMOVE) $(TARGETBASE)*.lss

# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex eep lss sym coff extcoff \
clean clean_list clean_list_output cleanintermediate program fuses-hex

