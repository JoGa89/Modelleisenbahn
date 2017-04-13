;****************************************************************************
;    Copyright (C) 2006..2012 Stefan Bormann / Martin Pischky
;
;    This library is free software; you can redistribute it and/or
;    modify it under the terms of the GNU Lesser General Public
;    License as published by the Free Software Foundation; either
;    version 2.1 of the License, or (at your option) any later version.
;
;    This library is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;    Lesser General Public License for more details.
;
;    You should have received a copy of the GNU Lesser General Public
;    License along with this library; if not, write to the Free Software
;    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
;
;*****************************************************************************
;
;    IMPORTANT:
;
;    Note: The sale any LocoNet device hardware (including bare PCB's) that
;    uses this or any other LocoNet software, requires testing and certification
;    by Digitrax Inc. and will be subject to a licensing agreement.
;
;    Please contact Digitrax Inc. for details.
;
;*****************************************************************************
;
; Title :   LocoNet Bootloader root assembler file for FREDI
; Author:   Stefan Bormann <stefan.bormann@gmx.de>
;           Martin Pischky <martin@pischky.de>
; Date:     5-May-2006, 5-Feb-2012, 16-Apr-2012
; Software: Atmel Assembler 2
; Target:   AtMega
;
; DESCRIPTION
;       Main module of bootstraploader.
;       Using PORT C as debug output (FREDI Hardware)
;       and Analog Comparator for Loconet.
;
; $Id: BootLoader.asm,v 1.4 2012/04/27 20:04:46 pischky Exp $
;
;*****************************************************************************

.listmac

; "#if MCU == atmega8" does not work so we use this little cheat
#if defined(MCU_IS_atmega8)
    #include <m8def.inc>
#elif defined(MCU_IS_atmega88)
    #include <m88def.inc>
#elif defined(MCU_IS_atmega168)
    #include <m168def.inc>
#elif defined(MCU_IS_atmega328p)
    #include <m328Pdef.inc>
#else
    #error "unknown MCU (expected something like '-D MCU_IS_atmega8' as command line parameter)"
#endif

#define DEBUG

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Board definitions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


#define LN_RX_PORT           ACSR //FREDI uses analog Comparator
#define LN_RX_BIT            ACO

#ifndef F_CPU
    #define F_CPU            7372800   ; system clock frequency, needed for
                                       ; LocoNet bit timing
#endif

    .def tmp2 = r25     ; register used by InitHardware (see usage in 
                        ; BootloaderAsm.inc)
    .def tmp1 = r24     ; register used by InitLocoNetHardware

    .macro InitHardware

        #if defined(__ATmega88__)   \
          | defined(__ATmega88A__) | defined(__ATmega88P__)  \
          | defined(__ATmega168__) | defined(__ATmega168A__) \
          | defined(__ATmega168P__)| defined(__ATmega328__)  \
          | defined(__ATmega328P__)

            // PortB0: Key S8 "SHIFT"  --> input, enable pullup to avoid floating
            // PortB1: Key S7 "F4/F8"  --> input, enable pullup to avoid floating
            // PortB2: Key S6 "F3/F7"  --> input, enable pullup to avoid floating
            // PortB3: Key S5 "F2/F6"  --> input, enable pullup to avoid floating
            // PortB4: Key S4 "F1/F5"  --> input, enable pullup to avoid floating
            // PortB5: Key S3 "F0"     --> input, enable pullup to avoid floating
            ldi   tmp2, (0<<DDB5) | (0<<DDB4) | (0<<DDB3) | (0<<DDB2) | (0<<DDB1) | (0<<DDB0)
            out   DDRB, tmp2
            ldi   tmp2, (1<<PORTB5) | (1<<PORTB4) | (1<<PORTB3) | (1<<PORTB2) | (1<<PORTB1) | (1<<PORTB0)
            out   PORTB, tmp2

            // PortD0: Key S10 "SHIFT2" --> input, enable pullup to avoid floating
            // PortD1: n.c.             --> input, enable pullup to avoid floating
            // PortD2: encoder          --> input, enable pullup to avoid floating
            // PortD3: encoder          --> input, enable pullup to avoid floating
            // PortD4: loconet tx       --> output, set to low
            // PortD5: S9 "ESTOP"       --> input, enable pullup to avoid floating
            // PortD[6,7] are Analog Comparator, set to input without pullups
            ldi   tmp2, (0<<DDD7) | (0<<DDD6) | (0<<DDD5) | (1<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0)
            out   DDRD, tmp2
            ldi   tmp2, (0<<PORTD7) | (0<<PORTD6) | (1<<PORTD5) | (0<<PORTD4) | (1<<PORTD3) | (1<<PORTD2) | (1<<PORTD1) | (1<<PORTD0)
            out   PORTD, tmp2

            // ADC off
            // DIDR0 = (1<<ADC0D)
            ldi   tmp2, (1<<ADC0D)        // disable digital input buffer to
            StoreRegister   DIDR0, tmp2   // reduce power consumption (PORTC0)
            // PRR |= (1<<PRADC)
            LoadRegister    tmp2, PRR     // shut down ADC in power reduction
            ori   tmp2, (1<<PRADC)        // register
            StoreRegister   PRR, tmp2

        #elif defined(__ATmega8__)
            #warning "no space on Atmega8 to do InitHardware"
        #else
            #error "MCU unknown"
        #endif

    .endmacro

    .macro InitLocoNetHardware

        // Enable Analog Comparator, disable Interrupt and Input Capture
        // Select Interrupt Mode to Rising (as in main FREDI code to avoid
        // interrupts on later initialization)
        ldi   tmp1, (0<<ACD) | (0<<ACBG) | (0<<ACI) | (0<<ACIE)  | (0<<ACIC) | (1<<ACIS1) | (1<<ACIS0)
        out   ACSR, tmp1
        #if defined(__ATmega88__)   \
          | defined(__ATmega88A__) | defined(__ATmega88P__)  \
          | defined(__ATmega168__) | defined(__ATmega168A__) \
          | defined(__ATmega168P__)| defined(__ATmega328__)  \
          | defined(__ATmega328P__)
            //AIN1, AIN0 digital input disable:
            // disable digital input buffer on the pins to reduce power consumption
            ldi   tmp1, (1<<AIN1D) | (1<<AIN0D)
            sts   DIDR1, tmp1
        #elif defined(__ATmega8__)
            //empty
        #else
            #error "MCU unknown"
        #endif

    .endmacro

;       BOOTSTRAP_START_BYTE 0x1E00    ; reset vector = entry point of bootloader
;       HARDWARE_VERSION               ; the bootloader must know the version of
                                       ; the hardware it resides. On FREDI this
                                       ; is the mcu used.
#if defined(__ATmega8__)
    #define HARDWARE_VERSION     1
    #ifndef BOOTSTRAP_START_BYTE
        #define BOOTSTRAP_START_BYTE 0x1E00
    #endif
#elif defined(__ATmega88__)
    #define HARDWARE_VERSION     2
    #ifndef BOOTSTRAP_START_BYTE
        ; boot section size 256 words / start address: 0x0F00
        ;#define BOOTSTRAP_START_BYTE 0x1E00
        ; boot section size 512 words / start address: 0x0E00
        #define BOOTSTRAP_START_BYTE 0x1C00
    #endif
    #define RESET_WATCHDOG_ON_STARTUP
#elif defined(__ATmega168__)
    #define HARDWARE_VERSION     3
    #ifndef BOOTSTRAP_START_BYTE
        ; boot section size 256 words / start address: 0x1F00
        ;#define BOOTSTRAP_START_BYTE 0x3E00
        ; boot section size 512 words / start address: 0x1E00
        #define BOOTSTRAP_START_BYTE 0x3C00
    #endif
    #define RESET_WATCHDOG_ON_STARTUP
#elif defined(__ATmega328P__)
    #define HARDWARE_VERSION     4
    #ifndef BOOTSTRAP_START_BYTE
        ; boot section size 256 words / start address: 0x3F00
        ;#define BOOTSTRAP_START_BYTE 0x7E00
        ; boot section size 512 words / start address: 0x3E00
        #define BOOTSTRAP_START_BYTE 0x7C00
    #endif
    #define RESET_WATCHDOG_ON_STARTUP
#else
    #error "MCU unknown"
#endif

.include "BootloaderDef.inc"

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Debug Code ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


; The debug code outputs the boot information and the page when burning
; to a digital port. This is the code for FREDI Hardware

    .equ DEBUG_PULL_UP = (1<<PORTC1)  ; DIRSWITCH
    .equ LED_GREEN_R   = PORTC2
    .equ LED_GREEN_L   = PORTC3
    .equ LED_RED       = PORTC4

    // macro called once on init
    .macro DebugInit

        ; pins with LEDs as output: DDRC = 0x1c
        ; LED_GREEN_R, LED_GREEN_L, LED_RED
        ldi   r24, (1<<DDC2) | (1<<DDC3) | (1<<DDC4)
        out   DDRC, r24

        ; enable pullup on direction switch to avoid floating, LEDs off
        ldi   r24, (0<<LED_GREEN_R) | (0<<LED_GREEN_L) | (0<<LED_RED) | DEBUG_PULL_UP
        out   PORTC, r24

    .endmacro

    // output("booting to app") ; starting application
    .macro DebugOutputToApp

        ldi   r24, (0<<LED_GREEN_R) | (0<<LED_GREEN_L) | (0<<LED_RED) | DEBUG_PULL_UP
        out   PORTC, r24

    .endmacro

    // output("app called bootloader") ; entering bootloader from application
    .macro DebugOutputFromApp

        ldi   r24,  (1<<LED_GREEN_R) | (1<<LED_GREEN_L) | (0<<LED_RED) | DEBUG_PULL_UP
        out   PORTC, r24

    .endmacro

    // output("invalid app") ; invalid application checksum found after reset
    .macro DebugOutputInvalid

        ; all LEDs on
        ldi   r24,  (1<<LED_GREEN_R) | (1<<LED_GREEN_L) | (1<<LED_RED) | DEBUG_PULL_UP
        out   PORTC, r24

    .endmacro

    // output(address>>6 & 0xff) ; show that we burn the flash memory
    .macro DebugOutputBurn   ;ADDRESS_LOW:ADDRESS_HIGH just passed a page boundary

        //FIXME
        movw  r24, ADDRESS_LOW    ;if("pagesize 32 words")
        lsl   r24                 ;    DEBUG_PORT = usPage>>6;
        rol   r25                 ;else if("pagesize 64 words")
                                  ;    DEBUG_PORT = usPage>>7;
        .if PAGESIZE < 64 ; can't use #if here because PAGESIZE is defined with .EQU
            lsl   r24
            rol   r25
        .endif

        lsl   r25                 ; leds are on pin 2..4
        lsl   r25
        com   r25                 ; start with all leds on
        andi  r25, (1<<LED_GREEN_R) | (1<<LED_GREEN_L) | (1<<LED_RED)
        ori   r25, DEBUG_PULL_UP
        out   PORTC, r25

    .endmacro

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; Bootloader Code ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


.include "BootloaderMessageAsm.inc"
.include "BurnAsm.inc"
.include "LnRxOnlySwAsm.inc"    ; using bit banged LocoNet
;.include "LnRxOnlyHwUartAsm.inc" ; option: hardware UART for LocoNet
.include "BootLoaderAsm.inc"    ; device independant code of bootloader

