#ifndef THROTTLE_INCLUDED
#define THROTTLE_INCLUDED

/****************************************************************************
    Copyright (C) 2002 Alex Shepherd

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

 Title :   LocoNet Throttle module header file
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     16-Mar-2003
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module provides all the throttle message decoding and provides
       functions to handle the common throttle operations

*****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "loconet.h"
#include "ln_sw_uart.h"
#include "common_defs.h"
#include "systimer.h"

typedef enum
{
  TH_ST_FREE   = 0,
  TH_ST_ACQUIRE,
  TH_ST_SELECT,
  TH_ST_DISPATCH,
  TH_ST_SLOT_MOVE,
  TH_ST_SLOT_FREE,
  TH_ST_SLOT_RESUME,
  TH_ST_IN_USE
} TH_STATE ;

typedef enum
{
  TH_ER_OK = 0,
  TH_ER_SLOT_IN_USE,
  TH_ER_BUSY,
  TH_ER_NOT_SELECTED,
  TH_ER_NO_LOCO,
  TH_ER_NO_SLOTS
} TH_ERROR ;

#define TH_OP_DEFERRED_SPEED 0x01

typedef struct throttle_data_t
{
  TH_STATE    State ;         // State of throttle
  word        TicksSinceLastAction ;
	word				ThrottleId ;		// Id of throttle
  byte        Slot ;          // Master Slot index
  word        Address ;       // Decoder Address
  byte        Speed ;         // Loco Speed
  byte        DeferredSpeed ; // Deferred Loco Speed setting
  byte        Status1 ;       // Stat1
  byte        DirFunc0to4 ;   // Direction
  byte        Func5to8 ;       // Direction
  byte        UserData ;
	byte				Options ;
  TimerAction ThrottleTimer ;
} THROTTLE_DATA_T;

void initThrottle( THROTTLE_DATA_T *ThrottleRec, byte UserData, byte Options, word ThrottleId ) ;

void processThrottleMessage( THROTTLE_DATA_T *ThrottleRec, lnMsg *LnPacket ) ;

word getThrottleAddress( THROTTLE_DATA_T *ThrottleRec ) ;
TH_ERROR setThrottleAddress( THROTTLE_DATA_T *ThrottleRec, word Address ) ;
TH_ERROR resumeThrottleAddress( THROTTLE_DATA_T *ThrottleRec, word Address, byte LastSlot ) ;
TH_ERROR dispatchThrottleAddress( THROTTLE_DATA_T *ThrottleRec, word Address ) ;
TH_ERROR acquireThrottleAddress( THROTTLE_DATA_T *ThrottleRec ) ;
void releaseThrottleAddress( THROTTLE_DATA_T *ThrottleRec ) ;
TH_ERROR freeThrottleAddress( THROTTLE_DATA_T *ThrottleRec, word Address ) ;

byte getThrottleSpeed( THROTTLE_DATA_T *ThrottleRec ) ;
TH_ERROR setThrottleSpeed( THROTTLE_DATA_T *ThrottleRec, byte Speed ) ;

byte getThrottleDirection( THROTTLE_DATA_T *ThrottleRec ) ;
TH_ERROR setThrottleDirection( THROTTLE_DATA_T *ThrottleRec, byte Direction ) ;

byte getThrottleFunction( THROTTLE_DATA_T *ThrottleRec, byte Function ) ;
TH_ERROR setThrottleFunction( THROTTLE_DATA_T *ThrottleRec, byte Function, byte Value ) ;
TH_ERROR setThrottleDirFunc0to4Direct( THROTTLE_DATA_T *ThrottleRec, byte Value ) ;
TH_ERROR setThrottleFunc5to8Direct( THROTTLE_DATA_T *ThrottleRec, byte Value ) ;

TH_STATE getThrottleState( THROTTLE_DATA_T *ThrottleRec ) ;

void notifyThrottleAddress( byte UserData, TH_STATE State, word Address, byte Slot ) ;
void notifyThrottleSpeed( byte UserData, TH_STATE State, byte Speed ) ;
void notifyThrottleDirection( byte UserData, TH_STATE State, byte Direction ) ;
void notifyThrottleFunction( byte UserData, byte Function, byte Value ) ;
void notifyThrottleSlotStatus( byte UserData, byte Status ) ;
void notifyThrottleError( byte UserData, TH_ERROR Error ) ;
void notifyThrottleState( byte UserData, TH_STATE PrevState, TH_STATE State ) ;

char *getStateStr( TH_STATE State ) ;
char *getErrorStr( TH_ERROR Error ) ;

#ifdef __cplusplus
}
#endif

#endif
