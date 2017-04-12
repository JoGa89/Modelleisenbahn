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

 Title :   RX only Loconet Receiver
 Author:   Stefan Bormann <stefan.bormann@gmx.de>
 Date:     17-Mar-2006
 Software: AVR-GCC
 Target:   AtMega8

 DESCRIPTION
       Source of receive only slim LocoNet receiver.

*****************************************************************************/
#include <avr/io.h>
#include <avr/wdt.h>
#include "sysdef.h"    // F_CPU
#include "LnRxOnly.h"


#define LN_RX_PORT            PINB
#define LN_RX_BIT             PB0

#define LN_BIT_PERIOD       (F_CPU / 16666)
          // The Start Bit period is a full bit period + half of the next bit period
          // so that the bit is sampled in middle of the bit
#define LN_TIMER_RX_START_PERIOD    LN_BIT_PERIOD + (LN_BIT_PERIOD / 2)
#define LN_TIMER_RX_RELOAD_PERIOD   LN_BIT_PERIOD 


unsigned short lnCompareTarget; // calculated bit sample time
unsigned char ucData;           // one received message byte

unsigned char ucLnRxOnlyChecksum;      // non static for access from inline function



void LnRxOnlySevereError(void)
{
	wdt_enable(WDTO_15MS);  // prepare for reset
	while (1) {}            // stop
}


unsigned char LnRxOnlyOpcode(void)
{
	do
	{
		ucLnRxOnlyChecksum = 0;
		loop_until_bit_is_set  (LN_RX_PORT, LN_RX_BIT);    // wait for IDLE=1 state
		LnRxOnlyData();
	}
	while (bit_is_clear(ucData, 7));  // loop if not opcode or byte framing error

	return ucData;
}



unsigned char LnRxOnlyData(void)
{
	unsigned char ucLoop;

//--- wait for start bit
	loop_until_bit_is_clear(LN_RX_PORT, LN_RX_BIT);

//--- Get the Current Timer1 Count and Add the offset for the Compare target to find the middle of the first data bit
	lnCompareTarget = TCNT1 + LN_TIMER_RX_START_PERIOD;
	OCR1A = lnCompareTarget ;

//--- Loop over data bits
	for (ucLoop=0; ucLoop<8; ucLoop++)
	{
//--- Waiting for data bit
		ucData >>= 1;   // Loconet is LSB first, shifting data in from left
		sbi(TIFR, OCF1A);
		loop_until_bit_is_set(TIFR, OCF1A);  // wait for middle of data bit

//--- Copy data bit from input to MSB of data buffer
		if (bit_is_set(LN_RX_PORT, LN_RX_BIT))
		{
			sbi(ucData, 7);
		}

//--- Calculate timer target for next data (or stop) bit
		lnCompareTarget += LN_TIMER_RX_RELOAD_PERIOD;
		OCR1A = lnCompareTarget;

	}
//--- Check stop bit
	sbi(TIFR, OCF1A);
	loop_until_bit_is_set(TIFR, OCF1A);  // wait for middle of stop bit
	if (bit_is_clear(LN_RX_PORT, LN_RX_BIT))
	{
		LnRxOnlySevereError();
	}
	ucLnRxOnlyChecksum ^= ucData;
	return ucData;
}

void LnRxOnlyChecksum(void)
{
	LnRxOnlyData();

	if (ucLnRxOnlyChecksum!=0xff)
		LnRxOnlySevereError();
}

