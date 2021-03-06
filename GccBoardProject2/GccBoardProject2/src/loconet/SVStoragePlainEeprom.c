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
 Date:     2-Jan-2004
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module implements the API to the EEPROM for SV storage
       for access by sv.c or possibly local application

*****************************************************************************/


#include <stdint.h>             // typedef uint8_t, typedef uint16_t
#include <avr/eeprom.h>         // eeprom_read_byte(), eeprom_write_byte()
#include "sysdef.h"             // SOFTWARE_VERSION, SERIAL_NUMBER_*
#include "sv.h"                 // SV_EE_SZ_*


#ifndef SOFTWARE_VERSION
#error "you must define 'SOFTWARE_VERSION' in sysdef.h!"
#endif



uint8_t readSVStorage( uint16_t Offset )
{
	switch (Offset)
	{
		case SV_ADDR_EEPROM_SIZE:
#if (E2END==0x0FF)	/* E2END is defined in processor include */
								return SV_EE_SZ_256;
#elif (E2END==0x1FF)
								return SV_EE_SZ_512;
#elif (E2END==0x3FF)
								return SV_EE_SZ_1024;
#elif (E2END==0x7FF)
								return SV_EE_SZ_2048;
#elif (E2END==0xFFF)
								return SV_EE_SZ_4096;
#else
								return 0xFF;
#endif

		case SV_ADDR_SW_VERSION:
								return SOFTWARE_VERSION;

		default:
								return eeprom_read_byte((uint8_t*)Offset);
	}
}


uint8_t writeSVStorage( uint16_t Offset, uint8_t Value )
{
  eeprom_write_byte((uint8_t*)Offset, Value);

#ifdef DO_USER_ACTION_AFTER_WRITING_EEPROM
	doUserActionAfterWriteSVStorage(Offset, Value);
#endif

  return 0;
}


uint8_t isValidSVStorage(uint16_t Offset)
{
	return Offset <= E2END;
}
