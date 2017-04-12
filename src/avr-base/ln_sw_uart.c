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

 Title :   LocoNet Software UART Access library
 Author:   Alex Shepherd <kiwi64ajs@sourceforge.net>
 Date:     13-Aug-2002
 Software:  AVR-GCC with AVR-AS
 Target:    any AVR device

 DESCRIPTION
  Basic routines for interfacing to the LocoNet via any output pin and
  either the Analog Comparator pins or the Input Capture pin

  The receiver uses the Timer1 Input Capture Register and Interrupt to detect
  the Start Bit and then the Compare A Register for timing the subsequest
  bit times.

  The Transmitter uses just the Compare A Register for timing all bit times
       
 USAGE
  See the C include ln_interface.h file for a description of each function
       
*****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#include "ln_sw_uart.h"    // prototypes of this module
#include "ln_interface.h"  // hardware independent prototypes and definitions
#include "sysdef.h"        // board definition

#if !defined(cbi) | !defined(sbi) // some sysdef.h define these two macros
  #include <compat/deprecated.h>  // #define cbi(), #define sbi()
#endif

/*** [BA040321] added ifdef for different PCB. Value defined in sysdef.h ****/


// Enable this definition for having a hardware support for measuring the
// startbit offset "LN_TIMER_TX_RELOAD_ADJUST".
// To get the needed value for LN_TIMER_TX_RELOAD_ADJUST enable the
// following definition, edit TX_START_MEASUREMENT_* defines for an unused
// pin and connect as scope to this pin.
// Adjust LN_TIMER_TX_RELOAD_ADJUST until length of impuls on this pin is 60
// microseconds. Do not forget to disable definition of TX_START_MEASUREMENT
// afterwards.
// See definition of LN_TIMER_TX_RELOAD_ADJUST in this file.
//#define TX_START_MEASUREMENT
#ifdef TX_START_MEASUREMENT
	#define TX_START_MEASUREMENT_PORT PORTB
	#define TX_START_MEASUREMENT_DDR  DDRB
	#define TX_START_MEASUREMENT_PIN  PB3
#endif


#if defined wBOARD_DAMMI 

  #ifndef BOARD_DEFINED_IN_SYSDEF
    #error definitions of loconet resources moved to sysdef.h for FREDI
  #endif

#elif defined BOARD_LOCO_DEV //EmbeddedLocoNet

#if defined (__AVR_ATmega16__)
	#define LN_SB_INT_ENABLE_REG  TIMSK
	#define LN_SB_INT_ENABLE_BIT  TICIE1
	#define LN_SB_INT_STATUS_REG  TIFR
	#define LN_TMR_INT_ENABLE_REG TIMSK
	#define LN_TMR_INT_STATUS_REG TIFR
#elif defined (__AVR_ATmega88__) || defined (__AVR_ATmega168__)
	#define LN_SB_INT_ENABLE_REG  TIMSK1
	#define LN_SB_INT_ENABLE_BIT  ICIE1
	#define LN_SB_INT_STATUS_REG  TIFR1
	#define LN_TMR_INT_ENABLE_REG TIMSK1
	#define LN_TMR_INT_STATUS_REG TIFR1
#endif

#define LN_SB_SIGNAL          TIMER1_CAPT_vect
#define LN_SB_INT_STATUS_BIT  ICF1

#define LN_TMR_SIGNAL         TIMER1_COMPA_vect
#define LN_TMR_INT_ENABLE_BIT OCIE1A
#define LN_TMR_INT_STATUS_BIT OCF1A

#define LN_TMR_INP_CAPT_REG   ICR1     // [BA040319] added defines for:
#define LN_TMR_OUTP_CAPT_REG  OCR1A     // ICR1, OCR1A, TCNT1, TCCR1B
#define LN_TMR_COUNT_REG      TCNT1     // and replaced their occurence in
#define LN_TMR_CONTROL_REG    TCCR1B    // the code.

#define LN_TMR_PRESCALER      1

#define LN_RX_PORT            PINB
#define LN_RX_BIT             PB0

#define LN_TX_PORT            PORTD
#define LN_TX_DDR             DDRD
#define LN_TX_BIT             PD6


#elif defined BOARD_PROTO_128 //ProtoBoardMega128
#define LN_RX_PORT            PINE
#define LN_RX_BIT             PORTE7

#define LN_SB_SIGNAL          TIMER3_CAPT_vect
#define LN_SB_INT_ENABLE_REG  ETIMSK
#define LN_SB_INT_ENABLE_BIT  TICIE3
#define LN_SB_INT_STATUS_REG  ETIFR
#define LN_SB_INT_STATUS_BIT  ICF3

#define LN_TMR_SIGNAL         TIMER3_COMPA_vect
#define LN_TMR_INT_ENABLE_REG ETIMSK
#define LN_TMR_INT_ENABLE_BIT OCIE3A
#define LN_TMR_INT_STATUS_REG ETIFR
#define LN_TMR_INT_STATUS_BIT OCF3A
#define LN_TMR_INP_CAPT_REG   ICR3     // [BA040319] added defines for:
#define LN_TMR_OUTP_CAPT_REG  OCR3A     // ICR1, OCR1A, TCNT1, TCCR1B
#define LN_TMR_COUNT_REG      TCNT3     // and replaced their occurence in
#define LN_TMR_CONTROL_REG    TCCR3B    // the code.

#define LN_TMR_PRESCALER      1

#define LN_TX_PORT            PORTE
#define LN_TX_DDR             DDRE
#define LN_TX_BIT             PORTE6

/*** [DH040609] added OmniPort Board definitions for OmniLinx Project */
#elif defined BOARD_OMNIPORT_OMNILINX // OmniPort OmniLinx Project
#define LN_RX_PORT            PINC	// OmniLinx
#define LN_RX_BIT             PC1	// OmniLinx

#define LN_SB_SIGNAL          TIMER1_CAPT_vect
#define LN_SB_INT_ENABLE_REG  TIMSK
#define LN_SB_INT_ENABLE_BIT  TICIE1
#define LN_SB_INT_STATUS_REG  TIFR
#define LN_SB_INT_STATUS_BIT  ICF1

#define LN_TMR_SIGNAL         TIMER1_COMPA_vect
#define LN_TMR_INT_ENABLE_REG TIMSK
#define LN_TMR_INT_ENABLE_BIT OCIE1A
#define LN_TMR_INT_STATUS_REG TIFR
#define LN_TMR_INT_STATUS_BIT OCF1A
#define LN_TMR_INP_CAPT_REG   ICR1     // [BA040319] added defines for:
#define LN_TMR_OUTP_CAPT_REG  OCR1A     // ICR1, OCR1A, TCNT1, TCCR1B
#define LN_TMR_COUNT_REG      TCNT1     // and replaced their occurence in
#define LN_TMR_CONTROL_REG    TCCR1B    // the code.

#define LN_TMR_PRESCALER      1

#define LN_TX_PORT            PORTC	// OmniLinx
#define LN_TX_DDR             DDRC	// OmniLinx
#define LN_TX_BIT             PC0	// OmniLinx


#elif defined BOARD_DT006_MEGA16 //ProtoBoardMega128
#define LN_RX_PORT            PINB
#define LN_RX_BIT             PB0

#define LN_SB_SIGNAL          TIMER1_CAPT_vect
#define LN_SB_INT_ENABLE_REG  TIMSK
#define LN_SB_INT_ENABLE_BIT  TICIE1
#define LN_SB_INT_STATUS_REG  TIFR
#define LN_SB_INT_STATUS_BIT  ICF1

#define LN_TMR_SIGNAL         TIMER1_COMPA_vect
#define LN_TMR_INT_ENABLE_REG TIMSK
#define LN_TMR_INT_ENABLE_BIT OCIE1A
#define LN_TMR_INT_STATUS_REG TIFR
#define LN_TMR_INT_STATUS_BIT OCF1A
#define LN_TMR_INP_CAPT_REG   ICR1     // [BA040319] added defines for:
#define LN_TMR_OUTP_CAPT_REG  OCR1A     // ICR1, OCR1A, TCNT1, TCCR1B
#define LN_TMR_COUNT_REG      TCNT1     // and replaced there occurence in
#define LN_TMR_CONTROL_REG    TCCR1B    // the code.

#define LN_TMR_PRESCALER      1

#define LN_TX_PORT            PORTD
#define LN_TX_DDR             DDRD
#define LN_TX_BIT             PD6

#elif defined BOARD_DEFINED_IN_SYSDEF

// no warning and no defnition

#else //No Board defined (Error)
#    warning "Board not defined"
#endif //Boardtype


#ifndef LN_SW_UART_SET_TX_LOW                               // putting a 1 to the pin to switch on NPN transistor
#define LN_SW_UART_SET_TX_LOW  sbi(LN_TX_PORT, LN_TX_BIT);  // to pull down LN line to drive low level
#endif

#ifndef LN_SW_UART_SET_TX_HIGH                              // putting a 0 to the pin to switch off NPN transistor
#define LN_SW_UART_SET_TX_HIGH cbi(LN_TX_PORT, LN_TX_BIT);  // master pull up will take care of high LN level
#endif


#define LN_ST_IDLE            0   // net is free for anyone to start transmission
#define LN_ST_CD_BACKOFF      1   // timer interrupt is counting backoff bits
#define LN_ST_TX_COLLISION    2   // just sending break after creating a collision
#define LN_ST_TX              3   // transmitting a packet
#define LN_ST_RX              4   // receiving bytes


#define   LN_COLLISION_TICKS 15

#define   LN_TX_RETRIES_MAX  25

          // The Start Bit period is a full bit period + half of the next bit period
          // so that the bit is sampled in middle of the bit
#define LN_TIMER_RX_START_PERIOD    LN_BIT_PERIOD + (LN_BIT_PERIOD / 2)
#define LN_TIMER_RX_RELOAD_PERIOD   LN_BIT_PERIOD 
#define LN_TIMER_TX_RELOAD_PERIOD   LN_BIT_PERIOD 

// ATTENTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// LN_TIMER_TX_RELOAD_ADJUST is a value for an error correction. This is needed for 
// every start of a byte. The first bit is to long. Therefore we need to reduce the 
// reload value of the bittimer.
// The following value depences highly on used compiler, optimizationlevel and hardware.
// Define the value in sysdef.h. This is very project specific.
// For the FREDI hard- and software it is nearly a quarter of a LN_BIT_PERIOD.
// Olaf Funke, 19th October 2007
// See definition of TX_START_MEASUREMENT in this file.
// ATTENTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#ifndef LN_TIMER_TX_RELOAD_ADJUST
	#define LN_TIMER_TX_RELOAD_ADJUST   0
	#error detect value by oszilloscope
#endif

// ATTENTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// signal to show when the bits are sampled
// The first falling edge is on start of stop bit.
// The following edges show the time when the bits 0..7
// and the stop bit are sampled.
//#define RX_MONITOR
#ifdef RX_MONITOR
  #define RX_MONITOR_PORT PORTB
  #define RX_MONITOR_DDR  DDRB
  #define RX_MONITOR_BIT  PB2
#endif

//#define COLLISION_MONITOR
#ifdef COLLISION_MONITOR
#define COLLISION_MONITOR_PORT PORTB
#define COLLISION_MONITOR_DDR DDRB
#define COLLISION_MONITOR_BIT PB4
#endif

//#define STARTBIT_MONITOR  //FIXME: not used
#ifdef STARTBIT_MONITOR
#define STARTBIT_MONITOR_PORT PORTB
#define STARTBIT_MONITOR_DDR DDRB
#define STARTBIT_MONITOR_BIT PB4
#endif

volatile byte    		lnState ;
volatile byte				lnBitCount ;
volatile byte				lnCurrentByte ;
volatile word       lnCompareTarget ; //TODO: validate that only used locally and remove (at least the volatile)

LnBuf               * lnRxBuffer ;
volatile lnMsg      * volatile lnTxData ;
volatile byte				lnTxIndex ;
volatile byte				lnTxLength ;
volatile byte       lnTxSuccess ;   // this boolean flag as a message from timer interrupt to send function


/**************************************************************************
*
* Start Bit Interrupt Routine
*
* DESCRIPTION
* This routine is executed when a falling edge on the incoming serial
* signal is detected. It disables further interrupts and enables
* timer interrupts (bit-timer) because the UART must now receive the
* incoming data.
*
**************************************************************************/

ISR(LN_SB_SIGNAL)
{
    // Disable the Input Comparator Interrupt
  cbi( LN_SB_INT_ENABLE_REG, LN_SB_INT_ENABLE_BIT );     
#ifdef RX_MONITOR
  cbi(RX_MONITOR_PORT, RX_MONITOR_BIT);
#endif

    // Get the Current Timer1 Count and Add the offset for the Compare target
  lnCompareTarget = LN_TMR_INP_CAPT_REG + LN_TIMER_RX_START_PERIOD ;
  LN_TMR_OUTP_CAPT_REG = lnCompareTarget ;

    // Clear the current Compare interrupt status bit and enable the Compare interrupt
  sbi(LN_TMR_INT_STATUS_REG, LN_TMR_INT_STATUS_BIT) ;
  sbi(LN_TMR_INT_ENABLE_REG, LN_TMR_INT_ENABLE_BIT) ; 

    // Set the State to indicate that we have begun to Receive
  lnState = LN_ST_RX ;

    // Reset the bit counter so that on first increment it is on 0
  lnBitCount = 0;
}

/**************************************************************************
*
* Timer Tick Interrupt
*
* DESCRIPTION
* This routine coordinates the transmition and reception of bits. This
* routine is automatically executed at a rate equal to the baud-rate. When
* transmitting, this routine shifts the bits and sends it. When receiving,
* it samples the bit and shifts it into the buffer.
*
**************************************************************************/

ISR(LN_TMR_SIGNAL)     /* signal handler for timer0 overflow */
{
    // Advance the Compare Target
  lnCompareTarget += LN_TIMER_RX_RELOAD_PERIOD ;
  LN_TMR_OUTP_CAPT_REG = lnCompareTarget ;

  lnBitCount++;                         //Increment bit_counter

  // Are we in the RX State
  if( lnState == LN_ST_RX )                // Are we in RX mode
  {
    #ifdef RX_MONITOR
      //toggle monitor output
      if( lnBitCount&1 ) sbi(RX_MONITOR_PORT, RX_MONITOR_BIT);
      else               cbi(RX_MONITOR_PORT, RX_MONITOR_BIT);
    #endif
    if( lnBitCount < 9)               // Are we in the Stop Bits phase
    {
      lnCurrentByte >>= 1;
      #ifdef LN_SW_UART_RX_INVERTED
        if( bit_is_clear(LN_RX_PORT, LN_RX_BIT) ) lnCurrentByte |= 0x80;
      #else
        if( bit_is_set(LN_RX_PORT, LN_RX_BIT) ) lnCurrentByte |= 0x80;
      #endif
      return ;
    }

    // Clear the Start Bit Interrupt Status Flag and Enable ready to
    // detect the next Start Bit
    sbi( LN_SB_INT_STATUS_REG, LN_SB_INT_STATUS_BIT ) ;
    sbi( LN_SB_INT_ENABLE_REG, LN_SB_INT_ENABLE_BIT ) ;

    #ifdef RX_MONITOR
      sbi(RX_MONITOR_PORT, RX_MONITOR_BIT);
    #endif

      // If the Stop bit is not Set then we have a Framing Error
#ifdef LN_SW_UART_RX_INVERTED
    if( bit_is_set(LN_RX_PORT,LN_RX_BIT) )
#else
    if( bit_is_clear(LN_RX_PORT,LN_RX_BIT) )
#endif
      lnRxBuffer->Stats.RxErrors++ ;

    else
        // Put the received byte in the buffer
      addByteLnBuf( lnRxBuffer, lnCurrentByte ) ;

    lnBitCount = 0 ;
    lnState = LN_ST_CD_BACKOFF ;
  }


    // Are we in the TX State
  if( lnState == LN_ST_TX )
  {
      // To get to this point we have already begun the TX cycle so we need to 
      // first check for a Collision. For now we will simply check that TX and RX
      // ARE NOT THE SAME as our circuit requires the TX signal to be INVERTED
      // If they are THE SAME then we have a Collision

#ifdef LN_SW_UART_TX_NON_INVERTED  // if you want to use the UART TX pin:
  #ifdef LN_SW_UART_RX_INVERTED
    if( ( ( LN_TX_PORT >> LN_TX_BIT ) & 0x01 ) == ( ( LN_RX_PORT >> LN_RX_BIT ) & 0x01 ) )
  #else
    if( ( ( LN_TX_PORT >> LN_TX_BIT ) & 0x01 ) != ( ( LN_RX_PORT >> LN_RX_BIT ) & 0x01 ) )
  #endif
#else  // inverted is the normal case, just a NPN between TX pin and LN:
  #ifdef LN_SW_UART_RX_INVERTED
    if( ( ( LN_TX_PORT >> LN_TX_BIT ) & 0x01 ) != ( ( LN_RX_PORT >> LN_RX_BIT ) & 0x01 ) )
  #else
    if( ( ( LN_TX_PORT >> LN_TX_BIT ) & 0x01 ) == ( ( LN_RX_PORT >> LN_RX_BIT ) & 0x01 ) )
  #endif
#endif
    {
      lnBitCount = 0 ;
      lnState = LN_ST_TX_COLLISION ;
    }
      // Send each Bit
    else if( lnBitCount < 9)
    {
#ifdef TX_START_MEASUREMENT
			cbi(TX_START_MEASUREMENT_PORT, TX_START_MEASUREMENT_PIN);
#endif
			
      if( lnCurrentByte & 0x01 )
        LN_SW_UART_SET_TX_HIGH

      else
        LN_SW_UART_SET_TX_LOW

      lnCurrentByte >>= 1;
    }
      // When the Data Bits are done, generate stop-bit
    else if( lnBitCount ==  9)
      LN_SW_UART_SET_TX_HIGH

      // Any more bytes in buffer
    else if( ++lnTxIndex < lnTxLength )
    {
        // Setup for the next byte
      lnBitCount = 0 ;
      lnCurrentByte = lnTxData->data[ lnTxIndex ] ;

        // Begin the Start Bit
      LN_SW_UART_SET_TX_LOW
#ifdef TX_START_MEASUREMENT
			sbi(TX_START_MEASUREMENT_PORT, TX_START_MEASUREMENT_PIN);
#endif

        // Get the Current Timer1 Count and Add the offset for the Compare target
        // added adjustment value for bugfix (Olaf Funke)
      lnCompareTarget = LN_TMR_COUNT_REG + LN_TIMER_TX_RELOAD_PERIOD - LN_TIMER_TX_RELOAD_ADJUST; 
      LN_TMR_OUTP_CAPT_REG = lnCompareTarget ;
    }
    else
    {
        // Successfully Sent all bytes in the buffer
        // so set the Packet Status to Done
      lnTxSuccess = 1 ;

        // Now copy the TX Packet into the RX Buffer
      addMsgLnBuf( lnRxBuffer, lnTxData );

        // Begin CD Backoff state
      lnBitCount = 0 ;
      lnState = LN_ST_CD_BACKOFF ;      
    }
  }

    // Note we may have got here from a failed TX cycle, if so BitCount will be 0
  if( lnState == LN_ST_TX_COLLISION )
  {
    if( lnBitCount == 0 )
    {
        // Pull the TX Line low to indicate Collision
      LN_SW_UART_SET_TX_LOW
#ifdef COLLISION_MONITOR
      cbi( COLLISION_MONITOR_PORT,COLLISION_MONITOR_BIT ) ;
#endif
    }
    else if( lnBitCount >= LN_COLLISION_TICKS )
    {
        // Release the TX Line
      LN_SW_UART_SET_TX_HIGH
#ifdef COLLISION_MONITOR
      sbi( COLLISION_MONITOR_PORT,COLLISION_MONITOR_BIT ) ;
#endif

      lnBitCount = 0 ;
      lnState = LN_ST_CD_BACKOFF ;

      lnRxBuffer->Stats.Collisions++ ;
    }
  }

  if( lnState == LN_ST_CD_BACKOFF )
  {
    if( lnBitCount == 0 )
    {
        // Even though we are waiting, other nodes may try and transmit early
        // so Clear the Start Bit Interrupt Status Flag and Enable ready to 
        // detect the next Start Bit
      sbi( LN_SB_INT_STATUS_REG, LN_SB_INT_STATUS_BIT ) ;
      sbi( LN_SB_INT_ENABLE_REG, LN_SB_INT_ENABLE_BIT ) ;
    }
    else if( lnBitCount >= LN_BACKOFF_MAX )
    { // declare network to free after maximum backoff delay
      lnState = LN_ST_IDLE ;
      cbi( LN_TMR_INT_ENABLE_REG, LN_TMR_INT_ENABLE_BIT ) ;
    }
  }
}


void initLocoNetHardware( LnBuf *RxBuffer )
{
#ifdef COLLISION_MONITOR
  sbi( COLLISION_MONITOR_DDR,COLLISION_MONITOR_BIT ) ; 
  sbi( COLLISION_MONITOR_PORT,COLLISION_MONITOR_BIT ) ; 
#endif
#ifdef STARTBIT_MONITOR
  sbi( STARTBIT_MONITOR_DDR,  STARTBIT_MONITOR_BIT ) ; 
  sbi( STARTBIT_MONITOR_PORT, STARTBIT_MONITOR_BIT ) ; 
#endif

#ifdef RX_MONITOR
  sbi(RX_MONITOR_DDR, RX_MONITOR_BIT);
  sbi(RX_MONITOR_PORT, RX_MONITOR_BIT);
#endif

#ifdef TX_START_MEASUREMENT
  sbi(TX_START_MEASUREMENT_DDR, TX_START_MEASUREMENT_PIN);
  cbi(TX_START_MEASUREMENT_PORT, TX_START_MEASUREMENT_PIN);
#endif

	
	lnRxBuffer = RxBuffer ;

    // Set the TX line to Inactive
  LN_SW_UART_SET_TX_HIGH
  sbi( LN_TX_DDR, LN_TX_BIT ) ;

/*** [DH040609] added ifdef for different PCB. Value defined in sysdef.h ****/
/*** [DH040609] added OmniPort Board definitions for OmniLinx Project */
#if defined BOARD_OMNIPORT_OMNILINX // OmniPort OmniLinx Project
	// Setup comparitors, etc
	SFIOR |= (1<<ACME);					// enable the Analog Comparator Muliplexor
	ADCSRA &= ~(1<<ADEN);					// turn off ADC
	ACSR |= (1<<ACBG)|(1<<ACO)|(1<<ACIC);	// turn on comparitor, bandgap, output, input capture
	ADMUX |= (1<<REFS0)|(1<<REFS1)|1;		//choose internal 2.56V source and Chl 1
	TCCR1B = (TCCR1B & 0xF8) | (1<<ICES1) | LN_TMR_PRESCALER ;    		// Set Rising Edge and Timer Clock Source 
#elif (defined wBOARD_FREDI || defined LN_SW_USE_AC_AS_IN_FREDI)
	// Enable Analog Comparator to Trigger the Input Capture unit
	ACSR = (1<<ACI) | (1<<ACIS1) | (1<<ACIS0) | (1<<ACIC) ;
  #if defined(__AVR_ATmega48__)  | defined(__AVR_ATmega48A__)  \
    | defined(__AVR_ATmega48P__) | defined(__AVR_ATmega88__)   \
    | defined(__AVR_ATmega88A__) | defined(__AVR_ATmega88P__)  \
    | defined(__AVR_ATmega168__) | defined(__AVR_ATmega168A__) \
    | defined(__AVR_ATmega168P__)| defined(__AVR_ATmega328__)  \
    | defined(__AVR_ATmega328P__)
    //AIN1, AIN0 digital input disable:
    // disable digital input buffer on the pins to reduce power consumption
    DIDR1 |= (1<<AIN1D) | (1<<AIN0D);
  #endif
#else
  #ifdef LN_SW_UART_USE_AC
    // First Enable the Analog Comparitor Power, 
    // Set the mode to Falling Edge
    // Enable Analog Comparator to Trigger the Input Capture unit
	  ACSR = (1<<ACI) | (1<<ACIS1) | (1<<ACIC) | (1<<ACBG);
    TCCR1B |= (1<<ICES1);
  #else
    // Turn off the Analog Comparator
    ACSR = 1<<ACD ;
  #endif
//	TCCR1B |= (1<<ICNC1) ;    		// Enable Noise Canceller 
#endif

  lnState = LN_ST_IDLE ;

    //Clear StartBit Interrupt flag
  sbi( LN_SB_INT_STATUS_REG, LN_SB_INT_STATUS_BIT ) ;

    //Enable StartBit Interrupt
  sbi( LN_SB_INT_ENABLE_REG, LN_SB_INT_ENABLE_BIT ) ;

    // Set Timer Clock Source 
  LN_TMR_CONTROL_REG = (LN_TMR_CONTROL_REG & 0xF8) | LN_TMR_PRESCALER ;
}


LN_STATUS sendLocoNetPacketTry(lnMsg *TxData, unsigned char ucPrioDelay)
{
  byte  CheckSum ;
  byte  CheckLength ;

  lnTxLength = getLnMsgSize( TxData ) ;

  // First calculate the checksum as it may not have been done
  CheckLength = lnTxLength - 1 ;
  CheckSum = 0xFF ;

  for( lnTxIndex = 0; lnTxIndex < CheckLength; lnTxIndex++ )
      CheckSum ^= TxData->data[ lnTxIndex ] ;
  
  TxData->data[ CheckLength ] = CheckSum ; 

  // clip maximum prio delay
  if (ucPrioDelay > LN_BACKOFF_MAX)
    ucPrioDelay = LN_BACKOFF_MAX;

  // if priority delay was waited now, declare net as free for this try
  cli();  // disabling interrupt to avoid confusion by ISR changing lnState while we want to do it
  if (lnState == LN_ST_CD_BACKOFF)
  {
    if (lnBitCount >= ucPrioDelay)  // Likely we don't want to wait as long as
    {                               // the timer ISR waits its maximum delay.
      lnState = LN_ST_IDLE ;
      cbi( LN_TMR_INT_ENABLE_REG, LN_TMR_INT_ENABLE_BIT ) ;
    }
  }
  sei();  // a delayed start bit interrupt will happen now,
          // a delayed timer interrupt was stalled

  // If the Network is not Idle, don't start the packet
  if (lnState == LN_ST_CD_BACKOFF)
  {
    if (lnBitCount < LN_CARRIER_TICKS)   // in carrier detect timer?
      return LN_CD_BACKOFF;
    else
      return LN_PRIO_BACKOFF;
  }

  if( lnState != LN_ST_IDLE )
    return LN_NETWORK_BUSY;  // neither idle nor backoff -> busy

  // We need to do this with interrupts off.
  // The last time we check for free net until sending our start bit
  // must be as short as possible, not interrupted.
  cli() ;
  // Before we do anything else - Disable StartBit Interrupt
  cbi( LN_SB_INT_ENABLE_REG, LN_SB_INT_ENABLE_BIT ) ;
  if (bit_is_set(LN_SB_INT_STATUS_REG, LN_SB_INT_STATUS_BIT))
  {
    // first we disabled it, than before sending the start bit, we found out
    // that somebody was faster by examining the start bit interrupt request flag
    sbi( LN_SB_INT_ENABLE_REG, LN_SB_INT_ENABLE_BIT ) ;
    sei() ;  // receive now what our rival is sending
    return LN_NETWORK_BUSY;
  }

  LN_SW_UART_SET_TX_LOW        // Begin the Start Bit

  // Get the Current Timer1 Count and Add the offset for the Compare target
  // added adjustment value for bugfix (Olaf Funke)
#ifdef TX_START_MEASUREMENT
	sbi(TX_START_MEASUREMENT_PORT, TX_START_MEASUREMENT_PIN);
#endif
	
  lnCompareTarget = LN_TMR_COUNT_REG + LN_TIMER_TX_RELOAD_PERIOD - LN_TIMER_TX_RELOAD_ADJUST;
  LN_TMR_OUTP_CAPT_REG = lnCompareTarget ;

  sei() ;

  lnTxData = TxData ;
  lnTxIndex = 0 ;
  lnTxSuccess = 0 ;

    // Load the first Byte
  lnCurrentByte = TxData->data[ 0 ] ;

    // Set the State to Transmit
  lnState = LN_ST_TX ;                      

    // Reset the bit counter
  lnBitCount = 0 ;                          

    // Clear the current Compare interrupt status bit and enable the Compare interrupt
  sbi(LN_TMR_INT_STATUS_REG, LN_TMR_INT_STATUS_BIT) ;
  sbi(LN_TMR_INT_ENABLE_REG, LN_TMR_INT_ENABLE_BIT) ; 

  // now busy waiting until the interrupts did the rest
  while (lnState == LN_ST_TX) {}

  if (lnTxSuccess)
  {
    lnRxBuffer->Stats.TxPackets++ ;
    return LN_DONE;
  }

  if (lnState == LN_ST_TX_COLLISION)
  {
    return LN_COLLISION;
  }

  return LN_UNKNOWN_ERROR; // everything else is an error
}

