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
#ifndef _SYSDEF_H_
#define _SYSDEF_H_

#include <avr/io.h>		// [we need all register and port definitions]

#define wBOARD_FREDI

//#define SW_INDEX        0x0001  // Software aus Wehmingen 05
//#define SW_INDEX        0x0002  // Software aus Hotteln 05
//#define SW_INDEX        0x0003  // Software nach Braunlage 06 (22.04.06)
                                  // Bugfix: reconnect (blinking forever)
                                  //         poti version works like Fred
                                  //         increment doesn't stop anymore
//#define SW_INDEX        0x0004  // Software für Cloppenburg 06 (14.05.06)
                                  // Bugfix: ignore of speedmessage after change
                                  //         dir by dirswitch

//#define SW_INDEX        0x0005  // Software nach Cloppenburg 06 (21.05.06)
                                  //          state cleaned up.
                                  
//#define SW_INDEX        0x0006  // Software Test mit Chief (03.06.06)
                                  // Bugfix: - state THR_STATE_RECONNECT_GET_SLOT 
                                  //           was missing in error handling
                                  //         - state corrected in case of not
                                  //           sending message successfully

//#define SW_INDEX          0x0007  // Software with decoder type encoding (22.08.06)

//#define SW_INDEX          0x0008  // Software with bugfix analog functions (26.08.06)

//#define SW_INDEX            0x0100  // software for release (18.09.06)

//#define SW_INDEX            0x0101  // T.Kurz: Software for release with invertion of dir
                                      // and differend speed tables
                                      // Red led is on, when loco is stoped (25.11.06)

//#define SW_INDEX            0x0102  // software for release (04.05.07)
                                     // Bugfix: no increment steps while stop is pressed
                                     // Bugfix: red led is running on selftest again

//#define SW_INDEX            0x0103  // software for release (19.08.07)
                                     // Removed the extension of Thomas, cause of problems on 
                                     // programming FREDIs with older SW versions

//#define SW_INDEX            0x0104  // software for release (19.10.07)
                                     // adjust of LN TX routine added
                                     // fixed problems on large arrangements

//#define SW_INDEX            0x0105  // software for release (09.12.07)
                                     // changed high and low byte for ID

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
  #define F_CPU                     7372800    // Fredi
#endif

#if defined(__AVR_ATmega48__)  | defined(__AVR_ATmega48A__)  \
  | defined(__AVR_ATmega48P__) | defined(__AVR_ATmega88__)   \
  | defined(__AVR_ATmega88A__) | defined(__AVR_ATmega88P__)  \
  | defined(__AVR_ATmega168__) | defined(__AVR_ATmega168A__) \
  | defined(__AVR_ATmega168P__)| defined(__AVR_ATmega328__)  \
  | defined(__AVR_ATmega328P__)
  #define LN_TIMER_TX_RELOAD_ADJUST   102 // 13.8 us delay for FREDI
                                          // on ATmega328P or ATmega168
                                          // avr-gcc (WinAVR 20100110) 4.3.3
#elif defined(__AVR_ATmega16__)
  #define LN_TIMER_TX_RELOAD_ADJUST    98 // 13.3 us delay for FREDI an ATmega8
                                          // avr-gcc (WinAVR 20100110) 4.3.3
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

#elif defined(__AVR_ATmega48__)  | defined(__AVR_ATmega48A__)  \
    | defined(__AVR_ATmega48P__) | defined(__AVR_ATmega88__)   \
    | defined(__AVR_ATmega88A__) | defined(__AVR_ATmega88P__)  \
    | defined(__AVR_ATmega168__) | defined(__AVR_ATmega168A__) \
    | defined(__AVR_ATmega168P__)| defined(__AVR_ATmega328__) \
    | defined(__AVR_ATmega328P__)

  #define LN_SB_SIGNAL          TIMER1_CAPT_vect
  #define LN_SB_INT_ENABLE_REG  TIMSK1  // Timer/Counter1 Interrupt Mask Register
  #define LN_SB_INT_ENABLE_BIT  ICIE1   // Timer/Counter1, Input Capture Interrupt Enable
  #define LN_SB_INT_STATUS_REG  TIFR1   // Timer/Counter1 Interrupt Flag Register
  #define LN_SB_INT_STATUS_BIT  ICF1

  #define LN_TMR_SIGNAL         TIMER1_COMPA_vect
  #define LN_TMR_INT_ENABLE_REG TIMSK1  // Timer/Counter1 Interrupt Mask Register
  #define LN_TMR_INT_ENABLE_BIT OCIE1A
  #define LN_TMR_INT_STATUS_REG TIFR1   // Timer/Counter1 Interrupt Flag Register
  #define LN_TMR_INT_STATUS_BIT OCF1A   // Timer/Counter1, output compare A match flag
  #define LN_TMR_INP_CAPT_REG   ICR1
  #define LN_TMR_OUTP_CAPT_REG  OCR1A
  #define LN_TMR_COUNT_REG      TCNT1
  #define LN_TMR_CONTROL_REG    TCCR1B
  #define LN_TMR_PRESCALER      1

#else
  #error unsupported MCU value (for now)
#endif

#define LN_TX_PORT            PORTD
#define LN_TX_DDR             DDRD

#define LN_TX_BIT             PD4



/******************************************************************************/
// ?????
/******************************************************************************/


// use monitor in and output for monitoring
//#define USE_UART


/******************************************************************************/
// Hardware mapping
/******************************************************************************/

// PortB
#define KEYPIN_SHIFT    PB0
#define KEYPIN_F4       PB1
#define KEYPIN_F3       PB2
#define KEYPIN_F2       PB3
#define KEYPIN_F1       PB4
#define KEYPIN_F0       PB5

#define KEYPIN_PORT     PORTB
#define KEYPIN_DDR      DDRB
#define KEYPIN_PIN      PINB

// PortC
#define ANA_SPD         PC0
#define DIRSWITCH       PC1
#define LED_GREEN_R     PC2
#define LED_GREEN_L     PC3
#define LED_RED         PC4
#define ANA_VERSION     PC5

#define LED_PORT        PORTC
#define LED_DDR         DDRC

#define DIRSWITCH_PORT  PORTC
#define DIRSWITCH_DDR   DDRC
#define DIRSWITCH_PIN   PINC

// PortD
#define ENC_BIT_0       PD2
#define ENC_BIT_1       PD3
#define ENC_SWITCH      PD5

#define ENC_PORT        PORTD
#define ENC_DDR         DDRD
#define ENC_PIN         PIND

//External Input Mapping

//Regler 1 (von Links nach Rechts wenn LED unter den Schiebereglern sind)
//insert schieberegler hier
#define LED1		PA0 //PINA
#define DIRKEY1		PC0 //PINC
#define FUNKEY1		PC7	//PINC

//Regler 2
//insert Schieberegler hier
#define LED2		PA1 //PINA
#define DIRKEY2		PA3 //PINA
#define FUNKEY2		PB0 //PINB

//Regler 3
//insert Schieberegler hier
#define LED3		PC3 //PINC
#define DIRKEY3		PC1 //PINC, Funktioniert nach mehreren Tastendrücken nichtmehr (Funktionswiederherstellung tum Teil durch Drücken anderer Taste)
#define FUNKEY3		PB4 // PINB

//Regler 4
//insert Schieberegler hier
#define LED4		PC4 //PINC
#define DIRKEY4		PC2 //PINC, Scheinbar Kurzschluss zu LED2, Wenn im Code benutzt leuchtet LED1 dauerhaft??
#define FUNKEY4		PA2 //PINA

//Erweiterte Funktionstasten (Nummeriert von oben nach unten)
#define ERW_FUNKEY1		PD3 //PIND
//#define ERW_FUNKEY2		PD4 //PINDgibt dauerhaft 1 aus
#define ERW_FUNKEY3		PD5 //PIND
#define ERW_FUNKEY4		PD6 //PIND
// encoder interrupt
#if defined(__AVR_ATmega16__)

  #define ENC_ISC_REG   MCUCR   // MCU Control Register
  #define ENC_ISC_BIT0  ISC10   // Interrupt Sense Control 1 Bit 0
  #define ENC_ISC_BIT1  ISC11   // Interrupt Sense Control 1 Bit 1
  #define ENC_EIRE_REG  GICR    // General Interrupt Control Register
  #define ENC_EIRE_BIT  INT1    // External Interrupt Request 1 Enable
  #define ENC_EIRF_REG  GIFR    // General Interrupt Flag Register
  #define ENC_EIRF_BIT  INTF1   // External Interrupt Flag 1

#elif defined(__AVR_ATmega48__)  | defined(__AVR_ATmega48A__)  \
    | defined(__AVR_ATmega48P__) | defined(__AVR_ATmega88__)   \
    | defined(__AVR_ATmega88A__) | defined(__AVR_ATmega88P__)  \
    | defined(__AVR_ATmega168__) | defined(__AVR_ATmega168A__) \
    | defined(__AVR_ATmega168P__)| defined(__AVR_ATmega328__) \
    | defined(__AVR_ATmega328P__)

  #define ENC_ISC_REG   EICRA   // External interrupt control register A
  #define ENC_ISC_BIT0  ISC10   // Interrupt Sense Control 1 Bit 0
  #define ENC_ISC_BIT1  ISC11   // Interrupt Sense Control 1 Bit 1
  #define ENC_EIRE_REG  EIMSK   // External interrupt mask register
  #define ENC_EIRE_BIT  INT1    // External interrupt request 1 enable
  #define ENC_EIRF_REG  EIFR    // External Interrupt Flag Register
  #define ENC_EIRF_BIT  INTF1   // External Interrupt Flag 1

#else
  #error unsupported MCU value (for now)
#endif

#define ENC_INT_vect	INT1_vect	// interrupt 1

// defines for key mapping

#define KEYPIN_ALL    ( _BV(KEYPIN_F0) |\
                        _BV(KEYPIN_F1) |\
                        _BV(KEYPIN_F2) |\
                        _BV(KEYPIN_F3) |\
                        _BV(KEYPIN_F4) |\
                        _BV(KEYPIN_SHIFT) )

#define Key_F0          _BV(KEYPIN_F0)
#define Key_SHIFT       _BV(KEYPIN_SHIFT)
#define Key_F1          _BV(KEYPIN_F1)
#define Key_F2          _BV(KEYPIN_F2)
#define Key_F3          _BV(KEYPIN_F3)
#define Key_F4          _BV(KEYPIN_F4)
#define Key_F5          (Key_F1 | Key_SHIFT)
#define Key_F6          (Key_F2 | Key_SHIFT)
#define Key_F7          (Key_F3 | Key_SHIFT)
#define Key_F8          (Key_F4 | Key_SHIFT)
#define Key_Stop        0x40
#define Key_Dir         0x80

#define Key_Enc_L       0x0100
#define Key_Enc_R       0x0200
#define Key_Poti_L      0x0400
#define Key_Poti_R      0x0800

#define Key_Fredi_Inkrement         (KEYPIN_ALL | Key_Stop | Key_Enc_L  | Key_Enc_R )
#define Key_Fredi_Inkrement_Switch  (KEYPIN_ALL | Key_Stop | Key_Enc_L  | Key_Enc_R  | Key_Dir)
#define Key_Fredi_Poti              (KEYPIN_ALL | Key_Stop | Key_Poti_L | Key_Poti_R | Key_Dir)

#define ENC_BITS      ( _BV(ENC_BIT_0) | _BV(ENC_BIT_1) )


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
  EEPROM_ADR_LAST,
};

#define  EEPROM_ID1_DEFAULT             0x00
#define  EEPROM_ID2_DEFAULT             0x00

#define  EEPROM_DECODER_TYPE_DEFAULT    0x00  // 28 Step decoder

#define  EEPROM_IMAGE_DEFAULT           0x55



/******************************************************************************/
// other
/******************************************************************************/

#define OPC_SELFTEST        0xaf
#define OPC_FRED_ADC        0xAF
#define OPC_FRED_BUTTON     0xA8

// this defines are required by ln_sw_uart.c, systimer.c
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define LN_MAX_BUFFER_SIZE  240



#endif // _SYSDEF_H_
