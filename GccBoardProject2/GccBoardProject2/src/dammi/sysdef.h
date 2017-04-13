/*
 * CFile1.c
 *
 * Created: 10.04.2017 14:43:57
 *  Author: Nils
 */ 

#include <avr/io.h>		// [we need all register and port definitions]

#define wBOARD_DAMMI
#define MANUFACTURER_ID 13
#define DEVELOPER_ID 1
#define PRODUCT_ID 1
#define SOFTWARE_VERSION 1
//eine "random" SV_MAX_NUMBER gesetzt
#define SV_MAX_NUMBER 4

#if defined(__AVR_ATmega16__)  
#define HARDWARE_VERSION  1
#else
#error "could not define HARDWARE_VERSION)"
#endif

/******************************************************************************/
// timing
/******************************************************************************/

#ifndef F_CPU
#define F_CPU                     7372800    // Fredi
#endif


#if defined(__AVR_ATmega16__)  
#define LN_TIMER_TX_RELOAD_ADJUST   102 // 13.8 us delay for FREDI
// on ATmega328P or ATmega168
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

#else
#error unsupported MCU value (for now)
#endif

#define LN_TX_PORT            PORTD
#define LN_TX_DDR             DDRD

#define LN_TX_BIT             PD4

// cbi sbi defines
// this defines are required by ln_sw_uart.c, systimer.c
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))