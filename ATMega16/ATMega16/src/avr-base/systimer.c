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

 Title :   System Timer
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     16-Mar-2003
 Software:  AVR-GCC
 Target:    AtMega8

 DESCRIPTION
       This module provides system timer actions

*****************************************************************************/

#include <avr/interrupt.h>

#include "systimer.h"

#define TIMER_PRESCALER_COUNT 256L    
#define TIMER_PRESCALER_CODE  4  

#define TICK_RELOAD           (256L - (F_CPU/(TIMER_TICK_FREQUENCY * TIMER_PRESCALER_COUNT)))

static TimerAction *SlowTimerActionList = 0 ; // 100 ms ticks
static TimerAction *FastTimerActionList = 0 ; // 1 ms ticks

byte SlowTimerAccumulator = 0 ;
volatile byte SlowTimerTicks = 0 ;
volatile byte FastTimerTicks = 0 ;
volatile word DelayTimerTicks = 0 ;

ISR(TIMER0_OVF_vect)
{
  TCNT0 = (byte) TICK_RELOAD ;

  FastTimerTicks++;

  if( ++SlowTimerAccumulator >= 100 )
  {
    SlowTimerAccumulator = 0 ;

    SlowTimerTicks++;
  }
	
	if( DelayTimerTicks )
		DelayTimerTicks--;
}

void initTimer()
{
	#if defined (__AVR_ATmega16__)
	
	    // Get the Current Timer1 Count and Add the offset for the Compare target
	  TCNT0 = (byte) TICK_RELOAD ;
	
	    // Clear the Overflow interrupt status bit and enable the overflow interrupt
	  sbi(TIFR, TOV0) ;
	  sbi(TIMSK, TOIE0) ; 
	
	    // Set Timer Clock Source 
	  TCCR0 = (TCCR0 & 0xF8) | TIMER_PRESCALER_CODE ;

	#elif defined (__AVR_ATmega88__)   || defined (__AVR_ATmega88A__)  \
	   || defined (__AVR_ATmega88P__)  || defined (__AVR_ATmega168__)  \
	   || defined (__AVR_ATmega168A__) || defined (__AVR_ATmega168P__) \
	   || defined (__AVR_ATmega328__)  || defined (__AVR_ATmega328P__)

	    // Get the Current Timer1 Count and Add the offset for the Compare target
	  TCNT0 = (byte) TICK_RELOAD ;
	
	    // Clear the Overflow interrupt status bit and enable the overflow interrupt
	  sbi(TIFR0, TOV0) ;
	  sbi(TIMSK0, TOIE0) ; 
	
	    // Set Timer Clock Source 
	  TCCR0B = (TCCR0B & 0xF8) | TIMER_PRESCALER_CODE ;

	#else
		#error "don't know how to initialize for this processor"
	#endif	  
	  
}

void addTimerAction( TimerAction *pAction, byte Ticks, byte (*TickAction) (void *UserPointer), void *UserPointer, byte Fast )
{
  byte  StatusReg ;

    // This needs to be done with Interrupts off, save Status reg,
    // disable interrupts and then restore the previous Status reg
    // to enable interrupts again if they were off 
  StatusReg = SREG ;
  cli() ;

  if(Fast)
  {
    pAction->Next = FastTimerActionList ;
    FastTimerActionList = pAction ;
  }
  else
  {
    pAction->Next = SlowTimerActionList ;
    SlowTimerActionList = pAction ;
  }

  pAction->Ticks = Ticks ;
  pAction->TickAction = TickAction ;
  pAction->UserPointer = UserPointer ;
  
    // Enable Interrupts if they were on to start with
  SREG = StatusReg ;
}



void resetTimerAction( TimerAction *pAction, byte Ticks )
{
  pAction->Ticks = Ticks ;
}

void delayTimer( word delayTicks )
{
	DelayTimerTicks = delayTicks ;
	
	while( DelayTimerTicks )
		; // Sit and wait for the interrupt handler to decrement the ticks to 0
}

void processTimerActions(void)
{
  TimerAction *pAction ;
  byte StatusReg ;
  byte FastTicks ;
  byte SlowTicks ;
  
    // This needs to be done with Interrupts off, save Status reg,
    // disable interrupts and then restore the previous Status reg
    // to enable interrupts again if they were off 
  StatusReg = SREG ;
  cli() ;

  FastTicks = FastTimerTicks ;   
  FastTimerTicks = 0 ;

  SlowTicks = SlowTimerTicks ;   
  SlowTimerTicks = 0 ;

    // Enable Interrupts if they were on to start with
  SREG = StatusReg ;

  if( FastTicks && FastTimerActionList )
  {
    while( FastTicks-- )
    {
      pAction = FastTimerActionList ;

      while( pAction )
      {
        if( pAction->Ticks )
        {
           pAction->Ticks--;
          // If we have gone from > 0 to 0 then set done and possibly reload
          if( !pAction->Ticks )
            pAction->Ticks = (*pAction->TickAction) ( pAction->UserPointer ) ;
        }

        pAction = pAction->Next ;
      }
    }
  }

  if( SlowTicks && SlowTimerActionList )
  {
    while( SlowTicks-- )
    {
      pAction = SlowTimerActionList ;

      while( pAction )
      {
        if( pAction->Ticks )
        {
           pAction->Ticks--;
          // If we have gone from > 0 to 0 then set done and possibly reload
          if( !pAction->Ticks )
            pAction->Ticks = (*pAction->TickAction) ( pAction->UserPointer ) ;
        }

        pAction = pAction->Next ;
      }
    }
  }
}                                                                                                                 
