/****************************************************************************
    Copyright (C) 2004 Alex Shepherd

    Portions Copyright (C) Digitrax Inc.

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

    Some of the message formats used in this code are Copyright Digitrax, Inc.
    and are used with permission as part of the EmbeddedLocoNet project. That
    permission does not extend to uses in other software products. If you wish
    to use this code, algorithm or these message formats outside of
    EmbeddedLocoNet, please contact Digitrax Inc, for specific permission.

    Note: The sale any LocoNet device hardware (including bare PCB's) that
    uses this or any other LocoNet software, requires testing and certification
    by Digitrax Inc. and will be subject to a licensing agreement.

    Please contact Digitrax Inc. for details.

*****************************************************************************

 Title :   LocoNet Fast Clock source code file
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     27-Mar-2004
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module provides a LocoNet slave Fast Clock that synchronises to the
       Master Fast Clock in the Command Station

*****************************************************************************/

#include <string.h>

#include "fastclock.h"
#include "systimer.h"
#include "ln_sw_uart.h"
#ifndef __BORLANDC__
#include "lcd.h"
#endif

/*
	The LocoNet fast clock in the Command Station is driven from a 65.535 ms
  time base. A normal minute takes approximately 915 x 65.535 ms ticks.

	The LocoNet fast clock values are stored in a special slot in the Command
	Station called the fast clock slot which is slot number 0x7B or 123
	
	Each of the fields in the are supposed to count up until the most significant bit
	is 0x80 and then rollover the appropriate values and reset however this behaviour
	does not seem to hold for all fields and so some corrction factors are needed

	An important part of syncing to the Fast Clock master is to interpret the current
	FRAC_MINS fields so that a Fast Clock Slave can sync to the part minute and then
	rollover it's accumulators in sync with the master. The FRAC_MINS counter is a
	14 bit counter that is stored in the two 7 bit FRAC_MINSL & FRAC_MINSH fields.
	It counts up the FRAC_MINSL field until it rolls over to 0x80 and then increments
	the FRAC_MINSH high field until it rolls over to 0x80 and then increments the minute,
	hour and day fields as appropriate and then resets the FRAC_MINS fields to 0x4000 - 915
	which is stored in each of the 7 bit fields.
 
	HOWEVER when the DCS100 resets FRAC_MINS fields to 0x4000 - 915, it then immediately
	rolls over a 128 count and so the minute is short by 915 - 128 65.535 ms ticks, so it
	runs too fast. To correct this problem the fast clock slot can be overwritten with
	corrected FRAC_MINS field values that the DCS100 will then increment correctly.

	This implementation of a LocoNet Fast Clock Slave has two features to correct these
	short commings:
	
	A) It has the option to reduce the FRAC_MINS count by 128 so that it keeps in step with
	the DCS100 Fast Clock which normally runs too fast. This is enabled by passing in the
	FC_FLAG_DCS100_COMPATIBLE_SPEED flag bit to the initFastClock() function. 
	
	B) It has the option to overwrite the LocoNet Fast Clock Master slot values with corrected
	FRAC_MINS fields imediately after it rolls-over the fast minute, to make the DCS100 not run
	too fast as it normally does.	
	
	There also seems to be problems with the hours field not rolling over correctly from 23
	back to 0 and so there is extra processing to work out the hours when it has rolled over
	to 0x80 or 0x00 by the time the bit 7 is cleared. This seems to cause the DT400 throttle
	problems as well and so when running in FC_FLAG_MINUTE_ROLLOVER_SYNC mode, this should
	be corrected. 

	The DT400 throttle display seems to decode the minutes incorrectly by 1 count and so we
	have to make the same interpretation here which is why there is a 127 and not a 128
  roll-over for the minutes. 
*/

#define FC_FRAC_MIN_BASE   0x3FFF
#define FC_FRAC_RESET_HIGH	 0x78
#define FC_FRAC_RESET_LOW 	 0x6D
#define FC_TIMER_TICKS         65        // 65ms ticks
#define FC_TIMER_TICKS_REQ	  250        // 250ms waiting for Response to FC Req

#define sendLocoNet4BytePacket(OpCode, Data1, Data2 )

typedef enum
{
  FC_ST_IDLE,
  FC_ST_REQ_TIME,
  FC_ST_READY,
  FC_ST_DISABLED,
} FC_STATE ;

static FC_STATE fcState ;					// State of the Fast Clock Slave 

static byte			fcFlags ;					// Storage of the option flags passed into initFastClock()

static fastClockMsg fcSlotData ;		// Primary storage for the Fast Clock slot data 

static byte fcLastPeriod ;					// Period of last tick so we can alternate between
																		// 65 and 66 ms ticks to approximate a 65.5 ms tick 

static TimerAction fcTimer ;				// Timer context

	// function Prototype for the system timer callback handler  
byte fcTimerAction( void *UserPointer ) ;  

void initFastClock(byte Flags)
{
  fcState = FC_ST_IDLE ;
	fcFlags = Flags ;
	
  addTimerAction(&fcTimer, FC_TIMER_TICKS, fcTimerAction, 0, 1 ) ;
}


void pollFastClock(void)
{
  //TO-DO-TEAM
  //sendLocoNet4BytePacket( OPC_RQ_SL_DATA, FC_SLOT, 0 ) ;
}

	// This function decodes the various fast clock slot message into meaningful clock values
	// and calls the user supplied notifyFastClock() function. The Sync bit tells the user
	// function that the change notification is due to hearing a fast clock slot update on the 
	// LocoNet
	
static void fcDoNotify( byte Sync )
{
	notifyFastClock(
		fcSlotData.clk_rate,
		fcSlotData.days,
		(fcSlotData.hours_24 >= (128-24)) ? fcSlotData.hours_24 - (128-24) : fcSlotData.hours_24 % 24 ,
		fcSlotData.mins_60 - (127-60 ),
		Sync ) ;
}

byte fcTimerAction( void *UserPointer )
{
		// If we are all initialised and ready then increment accumulators
  if( fcState == FC_ST_READY )
  {
    fcSlotData.frac_minsl +=  fcSlotData.clk_rate ;
    if( fcSlotData.frac_minsl & 0x80 )
    {
      fcSlotData.frac_minsl &= ~0x80 ;

      fcSlotData.frac_minsh++ ;
      if( fcSlotData.frac_minsh & 0x80 )
      {
					// For the next cycle prime the fraction of a minute accumulators
        fcSlotData.frac_minsl = FC_FRAC_RESET_LOW ;
				
					// If we are in FC_FLAG_DCS100_COMPATIBLE_SPEED mode we need to run faster
					// by reducong the FRAC_MINS duration count by 128
        fcSlotData.frac_minsh = FC_FRAC_RESET_HIGH + (fcFlags & FC_FLAG_DCS100_COMPATIBLE_SPEED) ;

        fcSlotData.mins_60++;
        if( fcSlotData.mins_60 >= 0x7F )
        {
          fcSlotData.mins_60 = 127 - 60 ;

          fcSlotData.hours_24++ ;
          if( fcSlotData.hours_24 & 0x80 )
          {
            fcSlotData.hours_24 = 128 - 24 ;

            fcSlotData.days++;
          }
        }
				
					// We either send a message out onto the LocoNet to change the time,
					// which we will also see and act on or just notify our user
					// function that our internal time has changed.
				if( fcFlags & FC_FLAG_MINUTE_ROLLOVER_SYNC )
				{
					fcSlotData.command = OPC_WR_SL_DATA ;
					//TO-DO-TEAM
					//sendLocoNetPacket((lnMsg*)&fcSlotData) ;
				}
				else
					fcDoNotify( 0 ) ;
      }
    }

		if( fcFlags & FC_FLAG_NOTIFY_FRAC_MINS_TICK )
			notifyFastClockFracMins( FC_FRAC_MIN_BASE - ( ( fcSlotData.frac_minsh << 7 ) + fcSlotData.frac_minsl ) ) ;

			// We need a delay of about 65.535 ms so lets just alternate between 65 & 66 ms
		fcLastPeriod = (fcLastPeriod == FC_TIMER_TICKS) ? FC_TIMER_TICKS + 1 : FC_TIMER_TICKS ;
	  return fcLastPeriod ;
  }

	if( fcState == FC_ST_IDLE )
  {
	//TO-DO-TEAM
    sendLocoNet4BytePacket( OPC_RQ_SL_DATA, FC_SLOT, 0 ) ;
    fcState = FC_ST_REQ_TIME ;
  }

  return FC_TIMER_TICKS_REQ ;
}


void processFastClockMessage( lnMsg *LnPacket )
{
  if( ( LnPacket->fc.slot == FC_SLOT ) &&
			( ( LnPacket->fc.command == OPC_WR_SL_DATA ) ||
				( LnPacket->fc.command == OPC_SL_RD_DATA ) ) )
  {
    if( LnPacket->fc.clk_cntrl & 0x40 )
    {
      if( fcState >= FC_ST_REQ_TIME )
      {
				memcpy( &fcSlotData, &LnPacket->fc, sizeof( fastClockMsg ) ) ; 
				
        fcDoNotify( 1 ) ;

				if( fcFlags & FC_FLAG_NOTIFY_FRAC_MINS_TICK )
					notifyFastClockFracMins( FC_FRAC_MIN_BASE - ( ( fcSlotData.frac_minsh << 7 ) + fcSlotData.frac_minsl ) );

        fcState = FC_ST_READY ;
      }
    }
    else
      fcState = FC_ST_DISABLED ;
  }
}


