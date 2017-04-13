#ifndef TURNOUT_INCLUDED
#define TURNOUT_INCLUDED

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

 Title :   LocoNet Layout Switch & Sensor header file
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     13-Dec-2004
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module provides a set of function to handle LocoNet Switch and Sensor messages

*****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#include "loconet.h"

byte processSwitchSensorMessage( lnMsg *LnPacket ) ;

void notifySensor( word Address, byte State ) ;

void notifySwitchRequest( word Address, byte Output, byte Direction ) ;
void notifySwitchReport( word Address, byte Output, byte Direction ) ;
void notifySwitchState( word Address, byte Output, byte Direction ) ;

void requestSwitch( word Address, byte Output, byte Direction ) ;
void reportSwitch( word Address ) ;

#ifdef __cplusplus
}
#endif

#endif
