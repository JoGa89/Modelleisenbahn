/****************************************************************************
    Copyright (C) 2003,2004 Alex Shepherd, Stefan Bormann

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

 Title :   LocoNet SV Configuration module
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
           Stefan Bormann <sbormann71@sourceforge.net>
 Date:     15-Oct-2003
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module provides a LocoNet interface for the SV Configuration
       messages

*****************************************************************************/

#ifdef __BORLANDC__
#include <malloc.h>
#include <string.h>
#endif

#include <stdint.h>     // typedef uint8_t, typedef uint16_t
#include <avr/wdt.h>    // wdt_enable() for reboot
#include <avr/io.h>
#include "ln_interface.h"
#include "sv.h"
#include "utils.h"
#include "sysdef.h"
#include "SVStorage.h"    // access to persistant SV storage
#include "IdStorage.h"    // read/write address of device


#ifndef MANUFACTURER_ID
#error "you must define 'MANUFACTURER_ID' in sysdef.h or makefile!"
#endif

#ifndef DEVELOPER_ID
#error "you must define 'DEVELOPER_ID' in sysdef.h or makefile!"
#endif

#ifndef PRODUCT_ID
#error "you must define 'PRODUCT_ID' in sysdef.h!"
#endif

#ifdef SV_VENDOR_ID
#error "you must replace SV_VENDOR_ID by MANUFACTURER_ID and DEVELOPER_ID"
#undef SV_VENDOR_ID
#endif

#ifdef SV_DEVICE_ID
#error "you must replyce SV_DEVICE_ID by PRODUCT_ID"
#undef SV_DEVICE_ID
#endif


#define SV_VENDOR_ID (MANUFACTURER_ID | (DEVELOPER_ID << 8))
#define SV_DEVICE_ID PRODUCT_ID



typedef union
{
uint16_t                  w;
struct { uint8_t lo,hi; } b;
} U16_t;

typedef union
{
struct
{
  U16_t unDestinationId;
  U16_t unVendorIdOrSvAddress;
  U16_t unDeviceId;
  U16_t unSerialNumber;
}    stDecoded;
uint8_t abPlain[8];
} SV_Addr_t;


uint8_t DeferredSrcAddr ;
uint8_t DeferredProcessingRequired ;


SV_STATUS doSVDeferredProcessing( void )
{
  if( DeferredProcessingRequired )
  {
    lnMsg msg ;
    SV_Addr_t unData ;
    
    msg.sv.command = (uint8_t) OPC_PEER_XFER ;
    msg.sv.mesg_size = (uint8_t) 0x10 ;
    msg.sv.src = DeferredSrcAddr ;
    msg.sv.sv_cmd = SV_DISCOVER | (uint8_t) 0x40 ;
    msg.sv.sv_type = (uint8_t) 0x02 ;
    msg.sv.svx1 = (uint8_t) 0x10 ;
    msg.sv.svx2 = (uint8_t) 0x10 ;
    
    unData.stDecoded.unDestinationId.w       = readSVDestinationId();
    unData.stDecoded.unVendorIdOrSvAddress.w = SV_VENDOR_ID;
    unData.stDecoded.unDeviceId.w            = SV_DEVICE_ID;
    unData.stDecoded.unSerialNumber.b.lo     = readSVStorage(SV_ADDR_SERIAL_NUMBER_L);
    unData.stDecoded.unSerialNumber.b.hi     = readSVStorage(SV_ADDR_SERIAL_NUMBER_H);
    
    encodePeerData( &msg.px, unData.abPlain );
    
    if( sendLocoNetPacketTry( &msg, LN_BACKOFF_INITIAL + ( unData.stDecoded.unSerialNumber.b.lo % (uint8_t) 10 ) ) != LN_DONE )
      return SV_DEFERRED_PROCESSING_NEEDED ;

    DeferredProcessingRequired = 0 ;
  }

  return SV_OK ;
}


static LN_STATUS SendLAck(unsigned char ucCode)
{
 return sendLocoNet4BytePacket(OPC_LONG_ACK, OPC_PEER_XFER-0x80, ucCode);
}


static bool CheckAddressRange(unsigned short usStartAddress, unsigned char ucCount)
{
 while (ucCount!=0)
 {
  if (!isValidSVStorage(usStartAddress))
  {
   SendLAck(42); // report invalid SV address error
   return FALSE;
  }
  usStartAddress++;
  ucCount--;
 }
 return TRUE; // all valid
}


SV_STATUS processSVMessage( lnMsg *LnPacket )
{
  SV_Addr_t unData ;
  
  if( ( LnPacket->sv.mesg_size != (uint8_t) 0x10 ) ||
      ( LnPacket->sv.command != (uint8_t) OPC_PEER_XFER ) ||
      ( LnPacket->sv.sv_type != (uint8_t) 0x02 ) ||
      ( LnPacket->sv.sv_cmd & (uint8_t) 0x40 ) ||
      ( ( LnPacket->sv.svx1 & (uint8_t) 0xF0 ) != (uint8_t) 0x10 ) ||
      ( ( LnPacket->sv.svx2 & (uint8_t) 0xF0 ) != (uint8_t) 0x10 ) )
    return SV_OK ;
 
  decodePeerData( &LnPacket->px, unData.abPlain ) ;

  if ((LnPacket->sv.sv_cmd != SV_DISCOVER) && 
      (LnPacket->sv.sv_cmd != SV_CHANGE_ADDRESS) && 
      (unData.stDecoded.unDestinationId.w != readSVDestinationId()))
  {
    return SV_OK;
  }

  switch( LnPacket->sv.sv_cmd )
  {
    case SV_WRITE_SINGLE:
        if (!CheckAddressRange(unData.stDecoded.unVendorIdOrSvAddress.w, 1)) return SV_ERROR;
        writeSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w, unData.abPlain[4]);
        // fall through inteded!
    case SV_READ_SINGLE:
        if (!CheckAddressRange(unData.stDecoded.unVendorIdOrSvAddress.w, 1)) return SV_ERROR;
        unData.abPlain[4] = readSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w);
        break;

    case SV_WRITE_MASKED:
        if (!CheckAddressRange(unData.stDecoded.unVendorIdOrSvAddress.w, 1)) return SV_ERROR;
        // new scope for temporary local variables only
        {
         unsigned char ucOld = readSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w) & (~unData.abPlain[5]);
         unsigned char ucNew = unData.abPlain[4] & unData.abPlain[5];
         writeSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w, ucOld | ucNew);
        }
        unData.abPlain[4] = readSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w);
        break;

    case SV_WRITE_QUAD:
        if (!CheckAddressRange(unData.stDecoded.unVendorIdOrSvAddress.w, 4)) return SV_ERROR;
        writeSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w+0,  unData.abPlain[4]);
        writeSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w+1,  unData.abPlain[5]);
        writeSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w+2,  unData.abPlain[6]);
        writeSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w+3, unData.abPlain[7]);
        // fall through intended!
    case SV_READ_QUAD:
        if (!CheckAddressRange(unData.stDecoded.unVendorIdOrSvAddress.w, 4)) return SV_ERROR;
        unData.abPlain[4] = readSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w+0);
        unData.abPlain[5] = readSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w+1);
        unData.abPlain[6] = readSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w+2);
        unData.abPlain[7] = readSVStorage(unData.stDecoded.unVendorIdOrSvAddress.w+3);
        break;

    case SV_DISCOVER:
        DeferredSrcAddr = LnPacket->sv.src ;
        DeferredProcessingRequired = 1 ;
        return SV_DEFERRED_PROCESSING_NEEDED ;
        break;
    
    case SV_IDENTIFY:
        unData.stDecoded.unDestinationId.w       = readSVDestinationId();
        unData.stDecoded.unVendorIdOrSvAddress.w = SV_VENDOR_ID;
        unData.stDecoded.unDeviceId.w            = SV_DEVICE_ID;
        unData.stDecoded.unSerialNumber.b.lo     = readSVStorage(SV_ADDR_SERIAL_NUMBER_L);
        unData.stDecoded.unSerialNumber.b.hi     = readSVStorage(SV_ADDR_SERIAL_NUMBER_H);
        break;

    case SV_CHANGE_ADDRESS:
        if(SV_VENDOR_ID != unData.stDecoded.unVendorIdOrSvAddress.w)
          return SV_OK; // not addressed
        if(SV_DEVICE_ID != unData.stDecoded.unDeviceId.w)
          return SV_OK; // not addressed
        if(readSVStorage(SV_ADDR_SERIAL_NUMBER_L) != unData.stDecoded.unSerialNumber.b.lo)
          return SV_OK; // not addressed
        if(readSVStorage(SV_ADDR_SERIAL_NUMBER_H) != unData.stDecoded.unSerialNumber.b.hi)
          return SV_OK; // not addressed
          
        if (writeSVDestinationId(unData.stDecoded.unDestinationId.w) != 0)
        {
          SendLAck(44);  // failed to change address (not implemented or failed to write)
          return SV_OK ; // the LN reception was ok, we processed the message
        }
        break;

    case SV_RECONFIGURE:
        break;  // actual handling is done after sending out the reply

    default:
        SendLAck(43); // not yet implemented
        return SV_ERROR;
  }

  encodePeerData( &LnPacket->px, unData.abPlain ); // recycling the received packet

  LnPacket->sv.sv_cmd |= 0x40;    // flag the message as reply

  sendLocoNetPacket(LnPacket);   // send successful reply

  if (LnPacket->sv.sv_cmd == (SV_RECONFIGURE | 0x40))
  {
    wdt_enable(WDTO_15MS);  // prepare for reset
    while (1) {}            // stop and wait for watchdog to knock us out
  }

  return SV_OK ;
}

