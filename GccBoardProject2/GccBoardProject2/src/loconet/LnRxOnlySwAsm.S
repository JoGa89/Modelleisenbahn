/****************************************************************************
    Copyright (C) 2006 Stefan Bormann

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

*****************************************************************************

    IMPORTANT:

    Note: The sale any LocoNet device hardware (including bare PCB's) that
    uses this or any other LocoNet software, requires testing and certification
    by Digitrax Inc. and will be subject to a licensing agreement.

    Please contact Digitrax Inc. for details.

*****************************************************************************

 Title :   LocoNet Bootstraploader
 Author:   Stefan Bormann <stefan.bormann@gmx.de>
 Date:     17-April-2006
 Software: AVR-GCC
 Target:   AtMega8

 DESCRIPTION
       Implementation of LocoNet receive only slim protocol modules.
       This module is an intermediate step for the translation to an
       assembler only bootloader.

*****************************************************************************/

#include <avr/io.h>

.global LnRxOnlyData
.global LnRxOnlySevereError
.global LnRxOnlyOpcode

.global LnRxOnlyChecksum
.global lnCompareTarget


.section .bss
lnCompareTarget:    .skip 2  // calculated bit sample time
ucData:             .skip 1  // one received message byte
ucLnRxOnlyChecksum: .skip 1  // non static for access from inline function


.section .text


.func LnRxOnlySevereError
LnRxOnlySevereError:
    ldi	r24, 0x18
    ldi	r18, 0x08
    out	WDTCR-0x20, r24
    out	WDTCR-0x20, r18
LnRxOnlySevereErrorLoop:
    rjmp	LnRxOnlySevereErrorLoop
.endfunc


.func LnRxOnlyChecksum
LnRxOnlyChecksum:

;void LnRxOnlyChecksum(void)
;{
;LnRxOnlyData();
    rcall	LnRxOnlyData

;if (ucLnRxOnlyChecksum!=0xff)
    lds  r24, ucLnRxOnlyChecksum
	ldi  r25, 0xff
    cpse r24, r25
;LnRxOnlySevereError();
    rcall	LnRxOnlySevereError
    ret
;}
.endfunc



.func LnRxOnlyOpcode
LnRxOnlyOpcode:
;unsigned char LnRxOnlyOpcode(void)
;{
;do
LnRxOnlyOpcodeWaitOpc:
;{
;ucLnRxOnlyChecksum = 0;
    sts	ucLnRxOnlyChecksum, r1
;loop_until_bit_is_set  (LN_RX_PORT, LN_RX_BIT);    // wait for IDLE=1 state
LnRxOnlyOpcodeWaitIdle:
    sbis	PINB-0x20, 0	; 22
    rjmp	LnRxOnlyOpcodeWaitIdle
;LnRxOnlyData();
    rcall	LnRxOnlyData    	; 0x1d34
;}
    and	r24, r24
    brge	LnRxOnlyOpcodeWaitOpc
;while (bit_is_clear(ucData, 7));  // loop if not opcode or byte framing error

;return ucData;
;}
    eor	r25, r25
    ret
.endfunc



#define COMPARE_TARGET_LO r18
#define COMPARE_TARGET_HI r19
#define BIT_LOOP_COUNTER  r25
#define TEMP_REG          r0
#define TEMP_REG_HIGH     r26
#define UCDATA_MIRROR     r24  // return value


.func LnRxOnlyData
LnRxOnlyData:
		push TEMP_REG_HIGH
;unsigned char LnRxOnlyData(void)
;{
.L2:
;unsigned char ucLoop;
;
;//--- wait for start bit
;loop_until_bit_is_clear(LN_RX_PORT, LN_RX_BIT);
LnRxOnlyDataWaitStart:
		sbic PINB-0x20,0
		rjmp LnRxOnlyDataWaitStart
;
;//--- Get the Current Timer1 Count and Add the offset for the Compare target to find the middle of 
;lnCompareTarget = TCNT1 + LN_TIMER_RX_START_PERIOD;
		in COMPARE_TARGET_LO,TCNT1-0x20
		in COMPARE_TARGET_HI,(TCNT1)+1-0x20
		subi COMPARE_TARGET_LO,lo8(-(720))
		sbci COMPARE_TARGET_HI,hi8(-(720))
;OCR1A = lnCompareTarget ;
		out (OCR1A)+1-0x20,COMPARE_TARGET_HI
		out OCR1A-0x20,COMPARE_TARGET_LO

		ldi BIT_LOOP_COUNTER,8
LnRxOnlyDataBitLoop:
;
;//--- Loop over data bits
;for (ucLoop=0; ucLoop<8; ucLoop++)
;{
;//--- Waiting for data bit
;ucData >>= 1;   // Loconet is LSB first, shifting data in from left
		lsr UCDATA_MIRROR
;sbi(TIFR, OCF1A);
		in TEMP_REG_HIGH,TIFR-0x20
		ori TEMP_REG_HIGH,lo8(16)
		out TIFR-0x20,TEMP_REG_HIGH
LnRxOnlyDataWaitDataMiddle:
;loop_until_bit_is_set(TIFR, OCF1A);  // wait for middle of data bit
		in TEMP_REG_HIGH,TIFR-0x20
		sbrs TEMP_REG_HIGH,4
		rjmp LnRxOnlyDataWaitDataMiddle
;
;//--- Copy data bit from input to MSB of data buffer
;if (bit_is_set(LN_RX_PORT, LN_RX_BIT))
		sbic PINB-0x20,0
;{
;sbi(ucData, 7);
		ori UCDATA_MIRROR,lo8(0x80)
;}
;
;//--- Calculate timer target for next data (or stop) bit
;lnCompareTarget += LN_TIMER_RX_RELOAD_PERIOD;
		subi COMPARE_TARGET_LO,lo8(-(480))
		sbci COMPARE_TARGET_HI,hi8(-(480))

;OCR1A = lnCompareTarget;
		out (OCR1A)+1-0x20,COMPARE_TARGET_HI
		out OCR1A-0x20,COMPARE_TARGET_LO

		subi BIT_LOOP_COUNTER,1
		brne LnRxOnlyDataBitLoop
;
;}
;//--- Check stop bit
;	sbi(TIFR, OCF1A);
		in TEMP_REG_HIGH,TIFR-0x20
		ori TEMP_REG_HIGH,lo8(16)
		out TIFR-0x20,TEMP_REG_HIGH
LnRxOnlyDataWaitStopMiddle:
;loop_until_bit_is_set(TIFR, OCF1A);  // wait for middle of stop bit
		in TEMP_REG_HIGH,TIFR-0x20
		sbrs TEMP_REG_HIGH,4
		rjmp LnRxOnlyDataWaitStopMiddle
;if (bit_is_clear(LN_RX_PORT, LN_RX_BIT))
		sbis PINB-0x20,0
;{
;LnRxOnlySevereError();
		rcall LnRxOnlySevereError
.L16:
;}
;ucLnRxOnlyChecksum ^= ucData;
		lds TEMP_REG_HIGH,ucLnRxOnlyChecksum
		eor TEMP_REG_HIGH,UCDATA_MIRROR
		sts ucLnRxOnlyChecksum,TEMP_REG_HIGH
;return ucData;
;}
		clr r25
		sts ucData, UCDATA_MIRROR
		pop TEMP_REG_HIGH
		ret
