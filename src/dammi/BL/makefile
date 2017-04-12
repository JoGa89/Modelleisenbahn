#*******************************************************************************
#    Copyright (C) 2011, 2012 Martin Pischky, Stefan Bormann
#*******************************************************************************
# Build Bootloader using gnu make (WINAVR)
# The assembler used is Atmels AVRASM2.EXE
#
# The target created is for use with ../Fredi
#
# $Id: makefile,v 1.4 2014/09/06 11:39:31 pischky Exp $
#
#*******************************************************************************

# this parameters can be overwritten on make commandline
# ------------------------------------------------------------------------------

# MANUFACTURER_ID: "NMRA DIY DCC"
# overwritten on command line for FREDI
MANUFACTURER_ID  = 13

# DEVELOPER_ID: "FREMO"
# overwritten on command line for FREDI
DEVELOPER_ID     =  1

# PRODUCT_ID: "FREDI Throttle" (Martin Pischky)
# see: http://embeddedloconet.sourceforge.net/device_ids/fremo.en.html
# overwritten on command line for FREDI
PRODUCT_ID       = 11

# defined also in BootLoader.asm
# overwritten on command line for FREDI
MCU              = atmega328p

# defined also in BootLoader.asm
# overwritten on command line for FREDI
F_CPU            = 7372800

# defined also in BootLoader.asm
# overwritten on command line for FREDI
BOOTSTRAP_START_BYTE = 0x7C00

# overwritten on command line for FREDI
HARDWARE_VERS_TEST   = 2

# ------------------------------------------------------------------------------

# name of target without extension (set here or overwrite on command line)
BOOTLOADER = BootLoader

# name of source (set here or overwrite on command line)
BOOTLOADER_ASM = $(BOOTLOADER).asm

# output file (set here or overwrite on command line)
BOOTLOADER_HEX = $(BOOTLOADER)-$(MCU).hex

# ------------------------------------------------------------------------------

# search *.inc from $(BOOTLOADER_INC_DIR)
BOOTLOADER_INC_DIR = ../../BootLoader/Assembler

vpath %.inc $(BOOTLOADER_INC_DIR)

# includes files $(BOOTLOADER_ASM) depends on
INCLUDES += BootLoaderAsm.inc
INCLUDES += BootLoaderDef.inc
INCLUDES += BootloaderMessageAsm.inc
INCLUDES += BurnAsm.inc
INCLUDES += LnRxOnlyHwUartAsm.inc
INCLUDES += LnRxOnlySwAsm.inc

ASMFLAGS += -D MANUFACTURER_ID=$(MANUFACTURER_ID)
ASMFLAGS += -D DEVELOPER_ID=$(DEVELOPER_ID)
ASMFLAGS += -D PRODUCT_ID=$(PRODUCT_ID)
ASMFLAGS += -D BOOTSTRAP_START_BYTE=$(BOOTSTRAP_START_BYTE)
ASMFLAGS += -D HARDWARE_VERS_TEST=$(HARDWARE_VERS_TEST)
ASMFLAGS += -D MCU_IS_$(MCU)
ASMFLAGS += -D F_CPU=$(F_CPU)

# assembler to use (AVR Studio 4 installation required/Studio 6.2 may work also)
# standard installation path is:
#    %ProgramFiles%\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe
AVRASM2 = $(shell echo $${PROGRAMFILES})\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe
# -- Atmel AVR Studio 4 --
#AVRASM2 = C:\Program Files (x86)\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe
#AVRASM2 = C:\Program Files\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe
#AVRASM2 = C:\Programme\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe
# -- Atmel Studio 6.2 (req. extra include of part definition directory) --
#AVRASM2_BASE = C:\Program Files\Atmel\Atmel Toolchain\AVR Assembler\Native\2.1.1117\avrassembler
#AVRASM2_BASE = C:\Program Files (x86)\Atmel\Atmel Toolchain\AVR Assembler\Native\2.1.1117\avrassembler
#AVRASM2 = $(AVRASM2_BASE)\avrasm2.exe
#AVRASM2_PARTDEF_INCLUDE = $(AVRASM2_BASE)\include
#ASMFLAGS += -I "$(AVRASM2_PARTDEF_INCLUDE)"

# Define programs and commands
SHELL = sh
REMOVE = rm -f

# generate listing file (uncomment if req.)
GEN_LISTING = -l $(BOOTLOADER).lst

MSG_ASSEMBLE  = --------------- assembling:

# rules ------------------------------------------------------------------------

all : avrasm2_exist $(BOOTLOADER_HEX) 

# validate that assembler has been configured properly
avrasm2_exist : 
	@if test ! -f "$(AVRASM2)" ; then\
	  echo "************************ ERROR ******************************" ;\
	  echo "\"$(AVRASM2)\" not found." ;\
	  echo "Customize makefile or add definition of complete path to make call." ;\
	  echo "For example you may use: " ;\
	  echo "     make AVRASM2=\"C:\\path to instdir\\avrasm2.exe\" all" ;\
	fi
	@test -f "$(AVRASM2)"

$(BOOTLOADER_HEX) : $(INCLUDES) $(BOOTLOADER_ASM) makefile
	@echo
	@echo $(MSG_ASSEMBLE) $(BOOTLOADER_ASM)
	@echo MANUFACTURER_ID = $(MANUFACTURER_ID)
	@echo DEVELOPER_ID = $(DEVELOPER_ID)
	@echo PRODUCT_ID = $(PRODUCT_ID)
	@echo BOOTSTRAP_START_BYTE = $(BOOTSTRAP_START_BYTE)
	@echo MCU = $(MCU)
	@echo F_CPU = $(F_CPU)
	"$(AVRASM2)" -S labels.tmp -fI -W+ie -I $(BOOTLOADER_INC_DIR) -C V2E \
	-o $(BOOTLOADER_HEX) -d $(BOOTLOADER).obj -m $(BOOTLOADER).map \
	$(GEN_LISTING) $(ASMFLAGS) $(BOOTLOADER_ASM)

clean :
	-$(REMOVE) labels.tmp *.obj *.map *.lst

distclean : clean
	-$(REMOVE) $(BOOTLOADER_HEX) *.hex

.PHONY : all clean distclean avrasm2_exist
