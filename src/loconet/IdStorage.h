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

 Title :   API of Read / Write access to device address "Destination ID"
 Author:   Stefan Bormann <sbormann71@sourceforge.net>
 Date:     18-Dez-2005
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This is just the API, different implementations exist.

*****************************************************************************/


#ifndef _ID_STORAGE_H_
#define _ID_STORAGE_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "avr-base/common_defs.h"   // word

word readSVDestinationId(void);
word writeSVDestinationId(word Destination);


#ifdef __cplusplus
}
#endif


#endif

