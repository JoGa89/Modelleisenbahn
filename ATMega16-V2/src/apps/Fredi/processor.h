/****************************************************************************
    Copyright (C) 2006, 2011 Olaf Funke, Martin Pischky

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

    $Id: processor.h,v 1.3 2011/07/25 18:31:16 pischky Exp $
******************************************************************************/

#ifndef _PROCESSOR_H_
#define _PROCESSOR_H_

/*
 *  Define some functions that use different names on ATmega8 and ATmega168
 */ 

#if defined(__AVR_ATmega16__)

  #define RESET_RESET_SOURCE() MCUCSR = 0;

#elif defined(__AVR_ATmega48__)  | defined(__AVR_ATmega48A__)  \
    | defined(__AVR_ATmega48P__) | defined(__AVR_ATmega88__)   \
    | defined(__AVR_ATmega88A__) | defined(__AVR_ATmega88P__)  \
    | defined(__AVR_ATmega168__) | defined(__AVR_ATmega168A__) \
    | defined(__AVR_ATmega168P__)| defined(__AVR_ATmega328__) \
    | defined(__AVR_ATmega328P__)

  #define RESET_RESET_SOURCE() MCUSR = 0;

#else

    #error unsupported MCU value (for now)

#endif

#endif // _PROCESSOR_H_
