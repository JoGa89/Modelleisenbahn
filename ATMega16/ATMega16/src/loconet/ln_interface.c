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

 Title :   LocoNet Access library
 Author:   Stefan Bormann <stefan.bormann@gmx.de>
 Date:     17-Aug-2006
 Software:  AVR-GCC
 Target:    megaAVR

 DESCRIPTION
  Hardware independant part of interface to the LocoNet.
       
 USAGE
  See the C include ln_interface.h file for a description of each function.
       
*****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#include "ln_interface.h"



static LnBuf *pstLnRxBuffer;  // this queue eats received LN messages


void initLocoNet(LnBuf *RxBuffer)
{
	pstLnRxBuffer = RxBuffer;
	initLocoNetHardware(RxBuffer);
}


lnMsg * recvLocoNetPacket( void )
{
	return recvLnMsg(pstLnRxBuffer);
}


#define   LN_TX_RETRIES_MAX  25
// this function should be moved to a hardware independant module
LN_STATUS sendLocoNetPacket( lnMsg *punTxData )
{
  unsigned char ucTry;
  unsigned char ucPrioDelay = LN_BACKOFF_INITIAL;
  LN_STATUS enReturn;
  unsigned char ucWaitForEnterBackoff;

  for (ucTry = 0; ucTry < LN_TX_RETRIES_MAX; ucTry++)
  {

    // wait previous traffic and than prio delay and than try tx
    ucWaitForEnterBackoff = 1;  // don't want to abort do/while loop before
    do                          // we did not see the backoff state once
    {
      enReturn = sendLocoNetPacketTry(punTxData, ucPrioDelay);

      if (enReturn == LN_DONE)  // success?
        return LN_DONE;

      if (enReturn == LN_PRIO_BACKOFF)
        ucWaitForEnterBackoff = 0; // now entered backoff -> next state != LN_BACKOFF is worth incrementing the try counter
    }
    while ((enReturn == LN_CD_BACKOFF) ||                             // waiting CD backoff
           (enReturn == LN_PRIO_BACKOFF) ||                           // waiting master+prio backoff
           ((enReturn == LN_NETWORK_BUSY) && ucWaitForEnterBackoff)); // or within any traffic unfinished
    // failed -> next try going to higher prio = smaller prio delay
    if (ucPrioDelay > LN_BACKOFF_MIN)
      ucPrioDelay--;
  }
  pstLnRxBuffer->Stats.TxError++ ;
  return LN_RETRY_ERROR;
}


LN_STATUS sendLocoNet4BytePacket( byte OpCode, byte Data1, byte Data2 )
{
  lnMsg SendPacket ;

  SendPacket.data[ 0 ] = OpCode ;
  SendPacket.data[ 1 ] = Data1 ;
  SendPacket.data[ 2 ] = Data2 ;

  return sendLocoNetPacket( &SendPacket ) ;
}

LN_STATUS sendLocoNet4BytePacketTry( byte OpCode, byte Data1, byte Data2, byte PrioDelay )
{
  lnMsg SendPacket ;

  SendPacket.data[ 0 ] = OpCode ;
  SendPacket.data[ 1 ] = Data1 ;
  SendPacket.data[ 2 ] = Data2 ;

  return sendLocoNetPacketTry( &SendPacket, PrioDelay ) ;
}
