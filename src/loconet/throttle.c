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

 Title :   LocoNet Throttle module
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     16-Mar-2003
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module provides all the throttle message decoding and provides
       functions to handle the common throttle operations

*****************************************************************************/

#include "throttle.h"

#ifdef __BORLANDC__
#include <malloc.h>
#include <string.h>
#endif

#define SLOT_REFRESH_TICKS   600   // 600 * 100ms = 60 seconds between speed refresh
#define THROTTLE_TIMER_TICKS   1   // 1 * 100ms = 100ms ticks

  // To make it easier to handle the Speed steps 0 = Stop, 1 = Em Stop and 2 -127
  // normal speed steps we will swap speed steps 0 and 1 so that the normal
  // range for speed steps is Stop = 1 2-127 is normal as before and now 0 = EmStop
static byte SwapSpeedZeroAndEmStop( byte Speed )
{
  if( Speed == 0 )
    return 1 ;

  if( Speed == 1 )
    return 0 ;

  return Speed ;
}

static void updateAddress( THROTTLE_DATA_T *ThrottleRec, word Address, byte ForceNotify )
{
  if( ForceNotify || ThrottleRec->Address != Address )
  {
    ThrottleRec->Address = Address ;
    notifyThrottleAddress( ThrottleRec->UserData, ThrottleRec->State, Address, ThrottleRec->Slot ) ;
  }
}

static void updateSpeed( THROTTLE_DATA_T *ThrottleRec, byte Speed, byte ForceNotify )
{
  if( ForceNotify || ThrottleRec->Speed != Speed )
  {
    ThrottleRec->Speed = Speed ;
    notifyThrottleSpeed( ThrottleRec->UserData, ThrottleRec->State, SwapSpeedZeroAndEmStop( Speed ) ) ;
  }
}

static void updateState( THROTTLE_DATA_T *ThrottleRec, TH_STATE State, byte ForceNotify )
{
  byte PrevState ;

  if( ForceNotify || ThrottleRec->State != State )
  {
    PrevState = ThrottleRec->State ;
    ThrottleRec->State = State ;
    notifyThrottleState( ThrottleRec->UserData, PrevState, State ) ;
  }
}

static void updateStatus1( THROTTLE_DATA_T *ThrottleRec, byte Status, byte ForceNotify )
{
	register byte Mask ;	// Temporary Byte Variable for bitwise AND to force
												// the compiler to only do 8 bit operations not 16

  if( ForceNotify || ThrottleRec->Status1 != Status )
  {
    ThrottleRec->Status1 = Status ;
    notifyThrottleSlotStatus( ThrottleRec->UserData, Status ) ;

		Mask = LOCO_IN_USE ;
    updateState( ThrottleRec, ( ( Status & Mask ) == Mask ) ? TH_ST_IN_USE : TH_ST_FREE, ForceNotify ) ;
  }
}

static void updateDirectionAndFunctions( THROTTLE_DATA_T *ThrottleRec, byte DirFunc0to4, byte ForceNotify )
{
  byte Diffs ;
  byte Function ;
  byte Mask ;

  if( ForceNotify || ThrottleRec->DirFunc0to4 != DirFunc0to4 )
  {
    Diffs = ThrottleRec->DirFunc0to4 ^ DirFunc0to4 ;
    ThrottleRec->DirFunc0to4 = DirFunc0to4 ;

      // Check Functions 1-4
    for( Function = 1, Mask = 1; Function <= 4; Function++ )
    {
      if( ForceNotify || Diffs & Mask )
        notifyThrottleFunction( ThrottleRec->UserData, Function, DirFunc0to4 & Mask ) ;

      Mask <<= 1 ;
    }

      // Check Functions 0
    if( ForceNotify || Diffs & DIRF_F0 )
      notifyThrottleFunction( ThrottleRec->UserData, 0, DirFunc0to4 & (byte)DIRF_F0 ) ;

      // Check Direction
    if( ForceNotify || Diffs & DIRF_DIR )
      notifyThrottleDirection( ThrottleRec->UserData, ThrottleRec->State, DirFunc0to4 & (byte)DIRF_DIR ) ;
  }
}

static void updateFunctions5to8( THROTTLE_DATA_T *ThrottleRec, byte Func5to8, byte ForceNotify )
{
  byte Diffs ;
  byte Function ;
  byte Mask ;

  if( ForceNotify || ThrottleRec->Func5to8 != Func5to8 )
  {
    Diffs = ThrottleRec->Func5to8 ^ Func5to8 ;
    ThrottleRec->Func5to8 = Func5to8 ;

      // Check Functions 5-8
    for( Function = 5, Mask = 1; Function <= 8; Function++ )
    {
      if( ForceNotify || Diffs & Mask )
        notifyThrottleFunction( ThrottleRec->UserData, Function, Func5to8 & Mask ) ;

      Mask <<= 1 ;
    }
  }
}

byte ThrottleTimerAction( void *UserPointer )
{
  THROTTLE_DATA_T *ThrottleRec = (THROTTLE_DATA_T*) UserPointer ;

  if( ThrottleRec->State == TH_ST_IN_USE )
  {
    ThrottleRec->TicksSinceLastAction++ ;

    if( ( ThrottleRec->DeferredSpeed ) || ( ThrottleRec->TicksSinceLastAction > SLOT_REFRESH_TICKS ) )
    {
      sendLocoNet4BytePacket( OPC_LOCO_SPD, ThrottleRec->Slot,
        ( ThrottleRec->DeferredSpeed ) ? ThrottleRec->DeferredSpeed : ThrottleRec->Speed ) ;

      if( ThrottleRec->DeferredSpeed )
        ThrottleRec->DeferredSpeed = 0 ;

      ThrottleRec->TicksSinceLastAction = 0 ;
    }
  }

  return THROTTLE_TIMER_TICKS ;
}

void initThrottle( THROTTLE_DATA_T *ThrottleRec, byte UserData, byte Options, word ThrottleId )
{
  ThrottleRec->State = TH_ST_FREE ;
	ThrottleRec->ThrottleId = ThrottleId ;
  ThrottleRec->DeferredSpeed = 0 ;
  ThrottleRec->UserData = UserData ;
	ThrottleRec->Options = Options ;
  addTimerAction(&ThrottleRec->ThrottleTimer, THROTTLE_TIMER_TICKS, ThrottleTimerAction, ThrottleRec, 0 ) ;
}

void processThrottleMessage( THROTTLE_DATA_T *ThrottleRec, lnMsg *LnPacket )
{
  byte  Data2 ;
	word  SlotAddress ;

    // Update our copy of slot information if applicable
  if( LnPacket->sd.command == OPC_SL_RD_DATA )
  {
		SlotAddress = (word) (( LnPacket->sd.adr2 << 7 ) + LnPacket->sd.adr ) ;
		
    if( ThrottleRec->Slot == LnPacket->sd.slot )
    {
				// Make sure that the slot address matches even though we have the right slot number
				// as it is possible that another throttle got in before us and took our slot.
			if( ThrottleRec->Address == SlotAddress )
			{
				if(	( ThrottleRec->State == TH_ST_SLOT_RESUME ) &&
						( ThrottleRec->ThrottleId != ( ( LnPacket->sd.id2 << 7 ) + LnPacket->sd.id1 ) ) )
				{
					updateState( ThrottleRec, TH_ST_FREE, 1 ) ;
	        notifyThrottleError( ThrottleRec->UserData, TH_ER_NO_LOCO ) ;
				}  
				
				updateState( ThrottleRec, TH_ST_IN_USE, 1 ) ;
				updateAddress( ThrottleRec, SlotAddress, 1 ) ;
				updateSpeed( ThrottleRec, LnPacket->sd.spd, 1 ) ;
				updateDirectionAndFunctions( ThrottleRec, LnPacket->sd.dirf, 1 ) ;
				updateFunctions5to8( ThrottleRec, LnPacket->sd.snd, 1 ) ;
				updateStatus1( ThrottleRec, LnPacket->sd.stat, 1 ) ;
	
					// We need to force a State update to cause a display refresh once all data is known
				updateState( ThrottleRec, TH_ST_IN_USE, 1 ) ;
				
					// Now Write our own Throttle Id to the slot and write it back to the command station
				LnPacket->sd.command = OPC_WR_SL_DATA ;
				LnPacket->sd.id1 = (byte) ( ThrottleRec->ThrottleId & 0x7F ) ;
				LnPacket->sd.id2 = (byte) ( ThrottleRec->ThrottleId >> 7 );
				sendLocoNetPacket( LnPacket ) ; 
			}
				// Ok another throttle did a NULL MOVE with the same slot before we did
				// so we have to try again
			else if( ThrottleRec->State == TH_ST_SLOT_MOVE )
			{
				updateState( ThrottleRec, TH_ST_SELECT, 1 ) ;
				sendLocoNet4BytePacket( OPC_LOCO_ADR, (byte) ( ThrottleRec->Address >> 7 ), (byte) ( ThrottleRec->Address & 0x7F ) ) ;
			}
    }
      // Slot data is not for one of our slots so check if we have requested a new addres
    else
    {
      if( ThrottleRec->Address == SlotAddress )
      {
        if( ( ThrottleRec->State == TH_ST_SELECT ) || ( ThrottleRec->State == TH_ST_DISPATCH ) )
        {
          if( ( LnPacket->sd.stat & STAT1_SL_CONUP ) == 0 &&
               ( LnPacket->sd.stat & LOCO_IN_USE ) != LOCO_IN_USE )
          {
            if( ThrottleRec->State == TH_ST_SELECT )
            {
              updateState( ThrottleRec, TH_ST_SLOT_MOVE, 1 ) ;
              ThrottleRec->Slot = LnPacket->sd.slot ;
              Data2 = LnPacket->sd.slot ;
            }
            else
            {
              updateState( ThrottleRec, TH_ST_FREE, 1 ) ;
              Data2 = 0 ;
            }

            sendLocoNet4BytePacket( OPC_MOVE_SLOTS, LnPacket->sd.slot, Data2 ) ;
          }
          else
          {
            notifyThrottleError( ThrottleRec->UserData, TH_ER_SLOT_IN_USE ) ;
            updateState( ThrottleRec, TH_ST_FREE, 1 ) ;
          }
        }
        else
        {
          if( ThrottleRec->State == TH_ST_SLOT_FREE )
          {
            sendLocoNet4BytePacket( OPC_SLOT_STAT1, LnPacket->sd.slot, (byte) ( ThrottleRec->Status1 & ~STAT1_SL_BUSY ) ) ;
            ThrottleRec->Slot = 0xFF ;
            updateState( ThrottleRec, TH_ST_FREE, 1 ) ;
          }
        }
      }

      if( ThrottleRec->State == TH_ST_ACQUIRE )
      {
        ThrottleRec->Slot = LnPacket->sd.slot ;
        updateState( ThrottleRec, TH_ST_IN_USE, 1 ) ;

        updateAddress( ThrottleRec, SlotAddress, 1 ) ;
        updateSpeed( ThrottleRec, LnPacket->sd.spd, 1 ) ;
        updateDirectionAndFunctions( ThrottleRec, LnPacket->sd.dirf, 1 ) ;
        updateStatus1( ThrottleRec, LnPacket->sd.stat, 1 ) ;
      }
    }
  }

  else if( ( ( LnPacket->sd.command >= OPC_LOCO_SPD ) && ( LnPacket->sd.command <= OPC_LOCO_SND ) ) ||
             ( LnPacket->sd.command == OPC_SLOT_STAT1 ) )
  {
    if( ThrottleRec->Slot == LnPacket->ld.slot )
    {
      if( LnPacket->ld.command == OPC_LOCO_SPD )
        updateSpeed( ThrottleRec, LnPacket->ld.data, 0 ) ;

      else if( LnPacket->ld.command == OPC_LOCO_DIRF )
        updateDirectionAndFunctions( ThrottleRec, LnPacket->ld.data, 0 ) ;

      else if( LnPacket->ld.command == OPC_LOCO_SND )
        updateFunctions5to8( ThrottleRec, LnPacket->ld.data, 0 ) ;

      else if( LnPacket->ld.command == OPC_SLOT_STAT1 )
        updateStatus1( ThrottleRec, LnPacket->ld.data, 0 ) ;
    }
  }
  else if( LnPacket->lack.command == OPC_LONG_ACK )
  {
    if( ( ThrottleRec->State >= TH_ST_ACQUIRE ) && ( ThrottleRec->State <= TH_ST_SLOT_MOVE ) )
    {
      if( LnPacket->lack.opcode == ( OPC_MOVE_SLOTS & 0x7F ) )
        notifyThrottleError( ThrottleRec->UserData, TH_ER_NO_LOCO ) ;

      if( LnPacket->lack.opcode == ( OPC_LOCO_ADR & 0x7F ) )
        notifyThrottleError( ThrottleRec->UserData, TH_ER_NO_SLOTS ) ;

      updateState( ThrottleRec, TH_ST_FREE, 1 ) ;
    }
  }
}

word getThrottleAddress( THROTTLE_DATA_T *ThrottleRec )
{
  return ThrottleRec->Address ;
}

TH_ERROR setThrottleAddress( THROTTLE_DATA_T *ThrottleRec, word Address )
{
  if( ThrottleRec->State == TH_ST_FREE )
  {
    updateAddress( ThrottleRec, Address, 1 ) ;
    updateState( ThrottleRec, TH_ST_SELECT, 1 ) ;

    sendLocoNet4BytePacket( OPC_LOCO_ADR, (byte) ( Address >> 7 ), (byte) ( Address & 0x7F ) ) ;
    return TH_ER_OK ;
  }

  notifyThrottleError( ThrottleRec->UserData, TH_ER_BUSY ) ;
  return TH_ER_BUSY ;
}

TH_ERROR resumeThrottleAddress( THROTTLE_DATA_T *ThrottleRec, word Address, byte LastSlot )
{
  if( ThrottleRec->State == TH_ST_FREE )
  {
		ThrottleRec->Slot = LastSlot ;
    updateAddress( ThrottleRec, Address, 1 ) ;
    updateState( ThrottleRec, TH_ST_SLOT_RESUME, 1 ) ;

    sendLocoNet4BytePacket( OPC_RQ_SL_DATA, LastSlot, 0 ) ;
    return TH_ER_OK ;
  }

  notifyThrottleError( ThrottleRec->UserData, TH_ER_BUSY ) ;
  return TH_ER_BUSY ;
}

TH_ERROR freeThrottleAddress( THROTTLE_DATA_T *ThrottleRec, word Address )
{
  if( ThrottleRec->State == TH_ST_FREE )
  {
    updateAddress( ThrottleRec, Address, 1 ) ;
    updateState( ThrottleRec, TH_ST_SLOT_FREE, 1 ) ;

    sendLocoNet4BytePacket( OPC_LOCO_ADR, (byte) ( Address >> 7 ), (byte) ( Address & 0x7F ) ) ;
    return TH_ER_OK ;
  }

  notifyThrottleError( ThrottleRec->UserData, TH_ER_BUSY ) ;
  return TH_ER_BUSY ;
}


TH_ERROR dispatchThrottleAddress( THROTTLE_DATA_T *ThrottleRec, word Address )
{
  if( ThrottleRec->State == TH_ST_FREE)
  {
    updateAddress( ThrottleRec, Address, 1 ) ;
    updateState( ThrottleRec, TH_ST_DISPATCH, 1 ) ;

    sendLocoNet4BytePacket( OPC_LOCO_ADR, (byte) ( Address >> 7 ), (byte) ( Address & 0x7F ) ) ;
    return TH_ER_OK ;
  }

  notifyThrottleError( ThrottleRec->UserData, TH_ER_BUSY ) ;
  return TH_ER_BUSY ;
}

TH_ERROR acquireThrottleAddress( THROTTLE_DATA_T *ThrottleRec )
{
  if( ThrottleRec->State == TH_ST_FREE )
  {
    updateState( ThrottleRec, TH_ST_ACQUIRE, 1 ) ;

    sendLocoNet4BytePacket( OPC_MOVE_SLOTS, 0, 0 ) ; 
    return TH_ER_OK ;
  }

  notifyThrottleError( ThrottleRec->UserData, TH_ER_BUSY ) ;
  return TH_ER_BUSY ;
}

void releaseThrottleAddress( THROTTLE_DATA_T *ThrottleRec )
{
  if( ThrottleRec->State == TH_ST_IN_USE )
  {
    sendLocoNet4BytePacket( OPC_SLOT_STAT1, ThrottleRec->Slot, (byte) ( ThrottleRec->Status1 & ~STAT1_SL_BUSY ) ) ;
  }

  ThrottleRec->Slot = 0xFF ;
  updateState( ThrottleRec, TH_ST_FREE, 1 ) ;
}

byte getThrottleSpeed( THROTTLE_DATA_T *ThrottleRec )
{
  return SwapSpeedZeroAndEmStop( ThrottleRec->Speed ) ;
}

TH_ERROR setThrottleSpeed( THROTTLE_DATA_T *ThrottleRec, byte Speed )
{
  if( ThrottleRec->State == TH_ST_IN_USE )
  {
    Speed = SwapSpeedZeroAndEmStop( Speed ) ;

    if( ThrottleRec->Speed != Speed )
    {
        // Always defer any speed other than stop or em stop
      if( (ThrottleRec->Options & TH_OP_DEFERRED_SPEED) &&
					( ( Speed > 1 ) || (ThrottleRec->TicksSinceLastAction == 0 ) ) )
        ThrottleRec->DeferredSpeed = Speed ;
      else
      {
        sendLocoNet4BytePacket( OPC_LOCO_SPD, ThrottleRec->Slot, Speed ) ;
        ThrottleRec->TicksSinceLastAction = 0 ;
        ThrottleRec->DeferredSpeed = 0 ;
      }
    }
    return TH_ER_OK ;
  }

  notifyThrottleError( ThrottleRec->UserData, TH_ER_NOT_SELECTED ) ;
  return TH_ER_NOT_SELECTED ;
}

byte getThrottleDirection( THROTTLE_DATA_T *ThrottleRec )
{
  return ThrottleRec->DirFunc0to4 & (byte)DIRF_DIR ;
}

TH_ERROR setThrottleDirection( THROTTLE_DATA_T *ThrottleRec, byte Direction )
{
  if( ThrottleRec->State == TH_ST_IN_USE )
  {
    sendLocoNet4BytePacket( OPC_LOCO_DIRF, ThrottleRec->Slot,
    ( Direction ) ? (byte) ( ThrottleRec->DirFunc0to4 | DIRF_DIR ) : (byte) ( ThrottleRec->DirFunc0to4 & ~DIRF_DIR ) ) ;

    ThrottleRec->TicksSinceLastAction = 0 ;
    return TH_ER_OK ;
  }

  notifyThrottleError( ThrottleRec->UserData, TH_ER_NOT_SELECTED ) ;
  return TH_ER_NOT_SELECTED ;
}

byte getThrottleFunction( THROTTLE_DATA_T *ThrottleRec, byte Function )
{
  byte Mask ;

  if( Function <= 4 )
  {
    Mask = (byte) (1 << ((Function) ? Function - 1 : 4 )) ;
    return ThrottleRec->DirFunc0to4 & Mask ;
  }

  Mask = (byte) (1 << (Function - 5)) ;
  return ThrottleRec->Func5to8 & Mask ;
}

TH_ERROR setThrottleFunction( THROTTLE_DATA_T *ThrottleRec, byte Function, byte Value )
{
  byte Mask ;
  byte OpCode ;
  byte Data ;

  if( ThrottleRec->State == TH_ST_IN_USE )
  {
    if( Function <= 4 )
    {
      OpCode = OPC_LOCO_DIRF ;
      Data = ThrottleRec->DirFunc0to4 ;
      Mask = (byte)(1 << ((Function) ? Function - 1 : 4 )) ;
    }
    else
    {
      OpCode = OPC_LOCO_SND ;
      Data = ThrottleRec->Func5to8 ;
      Mask = (byte)(1 << (Function - 5)) ;
    }

    if( Value )
      Data |= Mask ;
    else
      Data &= (byte)~Mask ;

    sendLocoNet4BytePacket( OpCode, ThrottleRec->Slot, Data ) ;

    ThrottleRec->TicksSinceLastAction = 0 ;
    return TH_ER_OK ;
  }

  notifyThrottleError( ThrottleRec->UserData, TH_ER_NOT_SELECTED ) ;
  return TH_ER_NOT_SELECTED ;
}

TH_ERROR setThrottleDirFunc0to4Direct( THROTTLE_DATA_T *ThrottleRec, byte Value )
{
  if( ThrottleRec->State == TH_ST_IN_USE )
	{
    sendLocoNet4BytePacket( OPC_LOCO_DIRF, ThrottleRec->Slot, Value & 0x7F ) ;
    return TH_ER_OK ;
	}

  notifyThrottleError( ThrottleRec->UserData, TH_ER_NOT_SELECTED ) ;
  return TH_ER_NOT_SELECTED ;
}

TH_ERROR setThrottleFunc5to8Direct( THROTTLE_DATA_T *ThrottleRec, byte Value )
{
  if( ThrottleRec->State == TH_ST_IN_USE )
	{
    sendLocoNet4BytePacket( OPC_LOCO_SND, ThrottleRec->Slot, Value & 0x7F ) ;
    return TH_ER_OK ;
	}

  notifyThrottleError( ThrottleRec->UserData, TH_ER_NOT_SELECTED ) ;
  return TH_ER_NOT_SELECTED ;
}

TH_STATE getThrottleState( THROTTLE_DATA_T *ThrottleRec )
{
  return ThrottleRec->State ;
}


char *getStateStr( TH_STATE State )
{
  switch( State )
  {
    case TH_ST_FREE:
      return "Free" ;

    case TH_ST_ACQUIRE:
      return "Acquire" ;

    case TH_ST_SELECT:
      return "Select" ;

    case TH_ST_DISPATCH:
      return "Dispatch" ;

    case TH_ST_SLOT_MOVE:
      return "Slot Move" ;

    case TH_ST_SLOT_FREE:
      return "Slot Free" ;

    case TH_ST_IN_USE:
      return "In Use" ;

    default:
      return "Unknown" ;
  }
}

char *getErrorStr( TH_ERROR Error )
{
  switch( Error )
  {
    case TH_ER_OK:
      return "Ok" ;

    case TH_ER_SLOT_IN_USE:
      return "In Use" ;

    case TH_ER_BUSY:
      return "Busy" ;

    case TH_ER_NOT_SELECTED:
      return "Not Sel" ;

    case TH_ER_NO_LOCO:
      return "No Loco" ;

    case TH_ER_NO_SLOTS:
      return "No Free Slots" ;

    default:
      return "Unknown" ;
  }
}


