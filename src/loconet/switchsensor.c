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

 Title :   LocoNet Switch & Sensor library file
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     13-Jun-2004
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module provides a set of function to handle LocoNet Switch and Sensor messages

*****************************************************************************/
#include "switchsensor.h"
#include "ln_interface.h"

static word LastSwitchAddress = 0xFFFF ;

byte processSwitchSensorMessage( lnMsg *LnPacket )
{
	word	Address ;
	byte 	Direction ;
	byte  Output ;
	byte  ConsumedFlag = 1 ;

	Address = (LnPacket->srq.sw1 | ( ( LnPacket->srq.sw2 & 0x0F ) << 7 )) ;
	if( LnPacket->sr.command != OPC_INPUT_REP )
		Address++;
		
	switch( LnPacket->sr.command )
	{
	case OPC_INPUT_REP:
		Address <<= 1 ;
		Address += ( LnPacket->ir.in2 & OPC_INPUT_REP_SW ) ? 2 : 1 ;
			
		notifySensor( Address, LnPacket->ir.in2 & OPC_INPUT_REP_HI ) ;
		break ;

	case OPC_SW_REQ:
		notifySwitchRequest( Address, LnPacket->srq.sw2 & OPC_SW_REQ_OUT, LnPacket->srq.sw2 & OPC_SW_REQ_DIR ) ;
		break ;

	case OPC_SW_REP:
		notifySwitchReport( Address, LnPacket->srp.sn2 & OPC_SW_REP_HI, LnPacket->srp.sn2 & OPC_SW_REP_SW ) ;
		break ;

	case OPC_SW_STATE:
		Direction = LnPacket->srq.sw2 & OPC_SW_REQ_DIR ;
		Output = LnPacket->srq.sw2 & OPC_SW_REQ_OUT ;

		notifySwitchState( Address, Output, Direction ) ;
		break;
		
	case OPC_SW_ACK:
		break ;

  case OPC_LONG_ACK:
    if( LnPacket->lack.opcode == (OPC_SW_STATE & 0x7F ) )
    {
      Direction = LnPacket->lack.ack1 & 0x01 ;
    }
		else
			ConsumedFlag = 0 ;
		break;
		
	default:
		ConsumedFlag = 0 ;
	}
	
	return ConsumedFlag ;
}

void requestSwitch( word Address, byte Output, byte Direction )
{
	byte AddrH = (--Address >> 7) & 0x0F ;
	byte AddrL = Address & 0x7F ;
	
	if( Output )
		AddrH |= OPC_SW_REQ_OUT ;
		
	if( Direction )
		AddrH |= OPC_SW_REQ_DIR ;
		
  sendLocoNet4BytePacket( OPC_SW_REQ, AddrL, AddrH ) ;
}

void reportSwitchState( word Address )
{
    // Remove offset of 1
  LastSwitchAddress = --Address ;

  sendLocoNet4BytePacket( OPC_SW_STATE, (Address & 0x7F), ((Address >> 7) & 0x0F) ) ;
}
