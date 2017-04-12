/****************************************************************************
    Copyright (C) 2002 Alex Shepherd

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

 Title :    LocoNet cccess Software UART library
 Author:    Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:      13-Aug-2002
 Software:  AVR-GCC with AVR-AS
 Target:    any AVR device

DESCRIPTION

  Basic routines for interfacing to the LocoNet via any output pin and
  either the Analog Comparator pins or the Input Capture pin

  The receiver uses the Timer1 Input Capture Register and Interrupt to detect
  the Start Bit and then the Compare A Register for timing the subsequest
  bit times.

  The Transmitter uses just the Compare A Register for timing all bit times

****************************************************************************/

#ifndef _LN_SW_UART_INCLUDED
#define _LN_SW_UART_INCLUDED


// All hardware independent generic interfaces are in loconet/ln_interface.h
// If this module needs implementation specific headers, place them here!


#endif
