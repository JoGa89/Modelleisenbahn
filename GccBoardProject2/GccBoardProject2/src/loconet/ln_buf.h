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

	Title :   LocoNet Buffer header file
	Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
	Date:     13-Feb-2004
	Software:  AVR-GCC
	Target:    AtMega8
	
	DESCRIPTION
	This module provides functions that manage the sending and receiving of LocoNet packets.
	
	As bytes are received from the LocoNet, they are stored in a circular
	buffer and after a valid packet has been received it can be read out.
	
	When packets are sent successfully, they are also appended to the Receive
	circular buffer so they can be handled like they had been received from
	another device.

	Statistics are maintained for both the send and receiving of packets.

	Any invalid packets that are received are discarded and the stats are
	updated approproately.

*****************************************************************************/

#ifndef _LN_BUF_INCLUDED

#define _LN_BUF_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "common_defs.h"
#include "loconet.h"

#ifndef LN_BUF_SIZE
#define LN_BUF_SIZE 128
#endif

typedef struct
{
	word RxPackets ;
	byte RxErrors ;
	word TxPackets ;
	byte TxError ;
	byte  Collisions ;
} LnBufStats ;

typedef struct
{
	byte     Buf[ LN_BUF_SIZE ] ;
	byte     WriteIndex ;
	byte     ReadIndex ;
	byte     ReadPacketIndex ;
	byte     CheckSum ;
	byte     ReadExpLen ;
	LnBufStats  Stats ;
} LnBuf ;

void initLnBuf( LnBuf *Buffer ) ;
lnMsg *recvLnMsg( LnBuf *Buffer ) ;
LnBufStats *getLnBufStats( LnBuf *Buffer ) ;
byte getLnMsgSize( volatile lnMsg * newMsg ) ;

#ifdef __BORLANDC__
void addByteLnBuf( LnBuf *Buffer, byte newByte );
void addMsgLnBuf( LnBuf *Buffer, volatile lnMsg * newMsg );
#else
static inline void addByteLnBuf( LnBuf *Buffer, byte newByte )
{
	Buffer->Buf[ Buffer->WriteIndex++ ] = newByte ;
	if( Buffer->WriteIndex >= LN_BUF_SIZE )
		Buffer->WriteIndex = 0 ;
}

static inline void addMsgLnBuf( LnBuf *Buffer, volatile lnMsg * newMsg )
{
	byte	Index ;
	byte 	Length ;

	Length = getLnMsgSize( newMsg ) ;
	for( Index = 0; Index < Length; Index++ )
		addByteLnBuf(Buffer, newMsg->data[ Index ] ) ;
}
#endif

#ifdef __cplusplus
}
#endif

#endif

