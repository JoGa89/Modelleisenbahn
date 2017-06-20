/****************************************************************************
    Copyright (C) 2006 Olaf Funke
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
    $Id: sysdef.h,v 1.16 2011/08/04 12:41:35 pischky Exp $
******************************************************************************/
//#define TX_START_MEASUREMENT

#ifndef _SYSDEF_H_
#define _SYSDEF_H_

#include <avr/io.h>		// [we need all register and port definitions]

#define wBOARD_FREDI

#ifndef SW_INDEX // see makefile
  #define SW_INDEX      0x0106  // software for release (06.04.10)
                                // send speed 1 only if speed was above 1 otherwise send 0
#endif

#ifndef SW_DAY // see makefile
  #define SW_DAY          0x06
#endif

#ifndef SW_MONTH // see makefile
  #define SW_MONTH        0x04
#endif

#ifndef SW_YEAR // see makefile
  #define SW_YEAR         0x10
#endif


/******************************************************************************/
// timing
/******************************************************************************/

#ifndef F_CPU
  #define F_CPU 16000000                                       //CPU-Speed auf 16Mhz gesetzt
#endif

#if defined(__AVR_ATmega16__)
  #define LN_TIMER_TX_RELOAD_ADJUST    85// 13.3 us delay for FREDI an ATmega8
                                          // avr-gcc (WinAVR 20100110) 4.3.3213
#else
  #error "unknown mcu"
#endif


#define TIMER_TICK_FREQUENCY        1000L // 1000kHz = 1ms tick size
#define TIMER_TICK_PERIOD_MS        (1000/TIMER_TICK_FREQUENCY) // Tick Period in ms

#if (TIMER_TICK_FREQUENCY==1000L)     // 1ms

  // fast timer                       // base 1ms
  #define INCREMENT_TIME          5   // 5ms
  #define KEY_POLL_TIME          10   // 10ms
  
  // slow timer                       // base 100ms
  #define LED_BLINK_TIME          1   // 100ms
  #define LED_SELFTEST_TIME       4   // 400ms
  #define LED_SELFTEST_DONE_TIME  1   // 100ms
  #define MESSAGE_TIME            3   // 300ms
  #define RESPONSE_TIME          20   // 2s
  #define SPEED_TIME            250   // 25s
  #define RELEASE_STOP_TIME       5   // 500ms

#else
  #error wrong timer tick frequency
#endif

#define LED_ON                      0 // 

/******************************************************************************/
// loconet resources
/******************************************************************************/
// moved definitions from ln_sw_uart.c to here

#define BOARD_DEFINED_IN_SYSDEF


#define LN_RX_PORT            ACSR // Analog Comperator Control and Status Register
#define LN_RX_BIT             ACO  // Analog Comperator Output

#if defined(__AVR_ATmega16__)

  #define LN_SB_SIGNAL          TIMER1_CAPT_vect
  #define LN_SB_INT_ENABLE_REG  TIMSK   // Timer/Counter Interrupt Mask Register
  #define LN_SB_INT_ENABLE_BIT  TICIE1  // Timer/Counter1, Input Capture Interrupt Enable
  #define LN_SB_INT_STATUS_REG  TIFR    // Timer/Counter Interrupt Flag Register
  #define LN_SB_INT_STATUS_BIT  ICF1

  #define LN_TMR_SIGNAL         TIMER1_COMPA_vect
  #define LN_TMR_INT_ENABLE_REG TIMSK   // Timer/Counter Interrupt Mask Register
  #define LN_TMR_INT_ENABLE_BIT OCIE1A
  #define LN_TMR_INT_STATUS_REG TIFR    // Timer/Counter Interrupt Flag Register
  #define LN_TMR_INT_STATUS_BIT OCF1A   // Timer/Counter1, Output Compare A Match Flag
  #define LN_TMR_INP_CAPT_REG   ICR1
  #define LN_TMR_OUTP_CAPT_REG  OCR1A
  #define LN_TMR_COUNT_REG      TCNT1
  #define LN_TMR_CONTROL_REG    TCCR1B
  #define LN_TMR_PRESCALER      1

#else
  #error unsupported MCU value (for now)
#endif

#define LN_TX_PORT            PORTB
#define LN_TX_DDR             DDRB

#define LN_TX_BIT             PB1

/******************************************************************************/
// Hardware mapping
/******************************************************************************/

// PortA
#define LED1			PA0
#define LED2			PA1
#define FUNK_L_4		PA2 
#define RICHTG_2		PA3 
#define REGLER_1		PA6
#define REGLER_2		PA7
#define REGLER_3		PA5
#define REGLER_4		PA4

#define PORT_A			PORTA
#define DDR_A			DDRA
#define PIN_A			PINA

//PortB (PB1, PB2, PB3, PB4 : undefined)
#define FUNK_L_2		PB0

#define PORT_B			PORTB
#define DDR_B			DDRB
#define PIN_B			PINB

//PortC (PC4 : undefined)

#define RICHTG_1		PC0
#define RICHTG_4		PC1
#define RICHTG_3		PC2
#define LED3			PC3
#define LED4			PC4
#define FUNK_L_3		PC5
#define FUNK_L_1		PC7

#define PORT_C			PORTC
#define DDR_C			DDRC
#define PIN_C			PINC

//PortD (PD0, PD1, PD2, PD7 : undefined)
#define FUNK_1			PD3
#define FUNK_2			PD4
#define FUNK_3			PD5
#define FUNK_4			PD6

#define PORT_D			PORTD
#define DDR_D			DDRD
#define PIN_D			PIND

// defines for key mapping
// Extra Funk Tasten
#define KEYPIN_FUNK_PORT_D    ( _BV(FUNK_1) |\
								_BV(FUNK_2) |\
								_BV(FUNK_3) |\
								_BV(FUNK_4) )

#define Key_FUNK_1				_BV(FUNK_1)
#define Key_FUNK_2				_BV(FUNK_2)
#define Key_FUNK_3				_BV(FUNK_3)
#define Key_FUNK_4				_BV(FUNK_4)

// Funk Tasten
#define Key_FUNK_L_1			_BV(FUNK_L_1)
#define Key_FUNK_L_2			_BV(FUNK_L_2)
#define Key_FUNK_L_3			_BV(FUNK_L_3)
#define Key_FUNK_L_4			_BV(FUNK_L_4)

// Direction Keys
#define Key_Dir_1				_BV(RICHTG_1)
#define Key_Dir_2				_BV(RICHTG_2)
#define Key_Dir_3				_BV(RICHTG_3)
#define Key_Dir_4				_BV(RICHTG_4)

/******************************************************************************/
// other defines
/******************************************************************************/

#define EVENT_1  0x01
#define EVENT_2  0x02
#define EVENT_3  0x04
#define EVENT_4  0x08
#define EVENT_5  0x10
#define EVENT_6  0x20
#define EVENT_7  0x40
#define EVENT_8  0x80

#define EVENT_KEY       EVENT_1
#define EVENT_LOCONET   EVENT_2
#define EV_MONITOR_RX   EVENT_3


/******************************************************************************/
// eeprom 
/******************************************************************************/
enum EEPROM_ADR
{
  EEPROM_ID2,                     // changed by Olaf 09.12.2007
  EEPROM_ID1,
  EEPROM_ADR_LOCO_HB,
  EEPROM_ADR_LOCO_LB,
  EEPROM_DECODER_TYPE,
  EEPROM_IMAGE,
  EEPROM_VERSION,
  EEPROM_SW_INDEX_HB,
  EEPROM_SW_INDEX_LB,
  EEPROM_SW_DAY,
  EEPROM_SW_MONTH,
  EEPROM_SW_YEAR,
  
  EEPROM_ADR_R1_LOCO_HB,
  EEPROM_ADR_R1_LOCO_LB,
  EEPROM_R1_STAT,
 
  EEPROM_ADR_R2_LOCO_HB,
  EEPROM_ADR_R2_LOCO_LB,
  EEPROM_R2_STAT,  
  
  EEPROM_ADR_LAST,
};

#define  EEPROM_ID1_DEFAULT             0x00
#define  EEPROM_ID2_DEFAULT             0x00

#define  EEPROM_DECODER_TYPE_DEFAULT    0x00  // 28 Step decoder

#define  EEPROM_IMAGE_DEFAULT           0x55



/******************************************************************************/
// other
/******************************************************************************/

// this defines are required by ln_sw_uart.c, systimer.c
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define LN_MAX_BUFFER_SIZE  240

#endif // _SYSDEF_H_