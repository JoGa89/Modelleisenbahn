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

 Title :   RX only Loconet Receiver
 Author:   Stefan Bormann <stefan.bormann@gmx.de>
 Date:     17-Mar-2006
 Software: AVR-GCC
 Target:   AtMega8

 DESCRIPTION
       Header of receive only slim LocoNet receiver.
       Some functions are inline, because they are expected to be called
       only once.

*****************************************************************************/
#ifndef _LN_RX_ONLY_H_
#define _LN_RX_ONLY_H_


#include <avr/io.h>




static void inline LnRxOnlyInit(void)
{
	TCCR1B = 0x01;     // no prescaler, normal mode
}



unsigned char LnRxOnlyOpcode(void);
unsigned char LnRxOnlyData(void);
void LnRxOnlySevereError(void);
void LnRxOnlyChecksum(void);





#endif
