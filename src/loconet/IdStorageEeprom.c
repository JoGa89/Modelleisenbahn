/****************************************************************************
    Copyright (C) 2005 Stefan Bormann

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

 Title :   Implementation of Read / Write access to device address "Destination 
           ID" in EEPROM
 Author:   Stefan Bormann <sbormann71@sourceforge.net>
 Date:     18-Dez-2005
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module provides a method to send a message asynchronously
       with a defined priority delay. Primary application is the discover
       reply of the SV programming module.

*****************************************************************************/


#include "avr/eeprom.h"
#include "IdStorage.h"
#include "sysdef.h"

#ifndef EEPROM_DSTL
	#define EEPROM_DSTL   1   // low byte of destination ID in SV location of eeprom size
#endif
#ifndef EEPROM_DSTH
	#define EEPROM_DSTH   2   // high byte of destination ID in SV location of version number
#endif


word readSVDestinationId(void)
{
	union
	{
		unsigned short                  w;
		struct { unsigned char lo,hi; } b;
	} un;

	un.b.lo = eeprom_read_byte((uint8_t *)EEPROM_DSTL);
	un.b.hi = eeprom_read_byte((uint8_t *)EEPROM_DSTH);

	return un.w;
}


word writeSVDestinationId(word usId)
{
	union
	{
		unsigned short                  w;
		struct { unsigned char lo,hi; } b;
	} un;

	un.w = usId;

	eeprom_write_byte((uint8_t *)EEPROM_DSTL, un.b.lo);
	eeprom_write_byte((uint8_t *)EEPROM_DSTH, un.b.hi);
	return 0;
}
