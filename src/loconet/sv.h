/****************************************************************************
    Copyright (C) 2002 Alex Shepherd

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

 Title :   LocoNet SV Configuration header file
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
           Stefan Bormann <sbormann71@sourceforge.net>
 Date:     15-Oct-2003
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module provides a LocoNet interface for the SV Configuration
       messages

*****************************************************************************/


#ifndef _SV_H_
#define _SV_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "loconet.h"


typedef enum
{
  SV_EE_SZ_256 = 0,
  SV_EE_SZ_512 = 1,
  SV_EE_SZ_1024 = 2,
  SV_EE_SZ_2048 = 3,
  SV_EE_SZ_4096 = 4
} SV_EE_SIZE ;

typedef enum
{
  SV_WRITE_SINGLE = 0x01,
  SV_READ_SINGLE = 0x02,
  SV_WRITE_MASKED = 0x03,
  SV_WRITE_QUAD = 0x05,
  SV_READ_QUAD = 0x06,
  SV_DISCOVER = 0x07,
  SV_IDENTIFY = 0x08,
  SV_CHANGE_ADDRESS = 0x09,
  SV_RECONFIGURE = 0x0F
} SV_CMD ;

typedef enum
{
  SV_ADDR_EEPROM_SIZE = 1,
  SV_ADDR_SW_VERSION = 2,
  SV_ADDR_SERIAL_NUMBER_L = 3,
  SV_ADDR_SERIAL_NUMBER_H = 4,
  SV_ADDR_USER_BASE = 5,
} SV_ADDR ;

typedef enum
{
  SV_OK = 0,
  SV_ERROR = 1,
  SV_DEFERRED_PROCESSING_NEEDED = 2
} SV_STATUS ;

SV_STATUS processSVMessage( lnMsg *LnPacket );
SV_STATUS doSVDeferredProcessing( void );

#ifdef __cplusplus
}
#endif


#endif
