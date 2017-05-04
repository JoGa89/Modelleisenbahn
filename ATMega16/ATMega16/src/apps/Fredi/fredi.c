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

    $Id: fredi.c,v 1.13 2011/07/31 15:54:12 pischky Exp $
******************************************************************************/


/******************************************************************************/
// Fredi
// The new FRED for Fremo!
/******************************************************************************/


/******************************************************************************/
// State by LED
/******************************************************************************/
//  - red on, green left off, green right off
//    Fredi ready for dispatching
//
//  - red and green blinking alternately
//    Fredi is connecting
//
//  - red, and green running around slowly
//    Fredi is in selftest, press all buttons and move knob around
//
//  - red, and green running around fast
//    Fredi has finished selftest successfully, remove cable and plug in again
//
//  - red off, green left on, green right off
//    normal running, direction backward
//
//  - red off, green left off, green right on
//    normal running, direction forward
//    
//  - red off, green left blinking, green right off
//    emergency stop, so move knob to the left (0), direction backward
//    
//  - red off, green left blinking, green right on
//    emergency stop, so move knob to the left (0), direction forward
//    
//  - all on
//    Fredi has no ID, you need a programmer
//    
//  - all off 
//    Plug in cable, ;-)
/******************************************************************************/


/******************************************************************************/
// overview
/******************************************************************************/
//         
//            ----------
//          /    ---     \                                                     .
//  red    | 0 /     \    |
//         |  |       |   | knob speed (and stop)
//         |   \     /    |
//         |     ---      |
// green l | 0         0  | green r
//         |              |
// stop    | O   (o)o     | direction switch
//         |              |
//         |      O       | F0
//         |              |
//         |      O       | F1 / F5
//         |              |
//         |      O       | F2 / F6
//         |              |
//         |      O       | F3 / F7
//         |              |
//         |      O       | F4 / F8
//         |              |
//         |      O       | Shift
//          \            /
//            ----------
//
// - dispatching
//   press stop and F0 together
// - F5..F8 = F1..F4 + Shift
/******************************************************************************/


#include <stdint.h>         // typedef int8_t, typedef uint8_t,
                            // typedef int16_t, typedef uint16_t
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>     // #define EEMEM

#include "sysdef.h"
#include "common_defs.h"
#include "ln_sw_uart.h"
#include "ln_interface.h"
//#include "throttle.h"
#include "systimer.h"

#include "processor.h"
#include "potadc.h"         // potAdcSpeedValue, potAdcRawValue
                            // potAdcPowerOff(), potAdcInit(),
                            // potAdcTimerAction()

/******************************************************************************/
// functions
/******************************************************************************/
void vSetState( byte bState );
void vProcessRxLoconetMessage(rwSlotDataMsg *currentSlot);
void vProcessRxMonitorMessage(void);
void vProcessKey(rwSlotDataMsg *currentSlot);
void vProcessEncoder(rwSlotDataMsg *currentSlot);
void vProcessPoti(rwSlotDataMsg *currentSlot);
void vCheckSelfTestEnd(void);

void vCopySlotFromRxPacket(void);
void vSetUnconnected(void);

void sendLocoNetSpd(rwSlotDataMsg *pSlot);
void sendLocoNetDirf(rwSlotDataMsg *pSlot);
void sendLocoNetSnd(rwSlotDataMsg *pSlot);
void sendLocoNetWriteSlotData(rwSlotDataMsg *pSlot);
void sendLocoNetMove(byte bSrc, byte bDest);
void sendLocoNetAdr(rwSlotDataMsg *pSlot);
//void sendLocoNetSelfTest(byte bTestCase, byte bValue);
void sendLocoNetFredAdc( uint16_t raw );
void sendLocoNetFredCd( uint8_t cdTime );
void sendLocoNetFredButton( uint8_t button );

/******************************************************************************/
// main defines & variables
/******************************************************************************/

/******************************************************************************/
// eeprom 
/******************************************************************************/

byte abEEPROM[EEPROM_ADR_LAST] EEMEM;

enum FREDI_VERSION
{
  FREDI_VERSION_UNDEF             = 0,
  FREDI_VERSION_INCREMENT         = 1,
  FREDI_VERSION_INCREMENT_SWITCH  = 2,
  FREDI_VERSION_ANALOG            = 3,
  FREDI_VERSION_LAST
};


byte bFrediVersion = FREDI_VERSION_UNDEF;


enum THR_STATE
{
  THR_STATE_INIT                    = 0,
  THR_STATE_CONNECTED               = 1,

  THR_STATE_ACQUIRE_LOCO_GET        = 10,
  THR_STATE_ACQUIRE_LOCO_WRITE      = 11, 

  THR_STATE_RECONNECT_GET_SLOT      = 20,
  THR_STATE_RECONNECT_NULL_MOVE     = 22, 
  THR_STATE_RECONNECT_WRITE         = 24,

  THR_STATE_UNCONNECTED_WRITE       = 30,
  THR_STATE_UNCONNECTED             = 32,

  THR_STATE_SELFTEST                = 100, // must be higher than other values
  THR_STATE_SELFTEST_DONE           = 101,
  THR_STATE_LAST
};


byte bNewState = THR_STATE_INIT;
byte bThrState = THR_STATE_SELFTEST;
volatile byte  bEvent     = 0;


/******************************************************************************/
// speed 
/******************************************************************************/

static volatile byte bSpdCnt = 0;

#define MAX_SPEED  33

const byte abSpd[MAX_SPEED+4] =
{
  0,  3,  5,  7, 11, 15, 19, 23, 27, 31,
  35, 39, 43, 47, 51, 55, 59, 63, 67, 71,
  75, 79, 83, 87, 91, 95, 99,103,107,111,
  115,119,123,127,127,127,127
};

#define GET_SPDCNT_BY_SLOTSPD bSpdCnt = MAX_SPEED;                                        \
                              for (i = 0; i < MAX_SPEED; i++)                             \
                              {                                                           \
                                if ((abSpd[i] <= rSlot.spd) && (abSpd[i+1] > rSlot.spd))  \
                                {                                                         \
                                  bSpdCnt = i;                                            \
                                }                                                         \
                              }




/******************************************************************************/
// loconet
/******************************************************************************/
LnBuf RxBuffer;

lnMsg *RxPacket;

//LN_STATUS RxStatus ;
//word      RxMsgCount ;
lnMsg TxPacket;

static int numberOfSlots = 1; //number of slots to be managed by the device
rwSlotDataMsg rSlot;

/******************************************************************************/
// timer
/******************************************************************************/
TimerAction  IncrementTimer;
TimerAction  KeyTimer;
TimerAction  LEDTimer;
TimerAction  MessageTimer;
TimerAction  ReleaseStopTimer;


byte bLEDReload = LED_BLINK_TIME;

/******************************************************************************/
// switches and decoder
/******************************************************************************/

static volatile int8_t    sEncDir       = 0;
static volatile byte    bCurrentKey   = 0;
static          uint16_t  wSelfTest     = 0;
static volatile byte    fSetSpeed     = TRUE;
static volatile byte    bStopPressed  = FALSE;

/******************************************************************************/

/******************************************************FunctionHeaderBegin******
 * FUNCTION    : IncrementTimerAction
 * CREATED     : 2005-01-21
 * AUTHOR      : Olaf Funke
 * DESCRIPTION : For debouncing of increment switch a timer is started. At 
 *               expiry of the timer the interrupt for the next increment is set
 *               in dependencie of the current level.
 *******************************************************************************
 * ARGUMENTS   : void *UserPointer
 * RETURN VALUE: byte, is set to 0 because this timer is has only 
 *               "monoflop"-functionality
 *******************************************************FunctionHeaderEnd******/
byte IncrementTimerAction( void *UserPointer)
{
  // detect actual state of pin to set next interrupt edge
  if ( bit_is_set(ENC_PIN, ENC_BIT_1))
  { // set falling edge
    ENC_ISC_REG &= ~_BV(ENC_ISC_BIT0);
  }
  else
  { // set rising edge
    ENC_ISC_REG |= _BV(ENC_ISC_BIT0);
  }

  // clear pending interrupt
  ENC_EIRF_REG |= _BV(ENC_EIRF_BIT);
  // set interrupt activ
  ENC_EIRE_REG |= _BV(ENC_EIRE_BIT);

  return 0;
}


/******************************************************FunctionHeaderBegin******
 * FUNCTION    : LEDTimerAction
 * CREATED     : 20051-01-08
 * AUTHOR      : Olaf Funke
 * DESCRIPTION : This function is responsible for a blinking of the LEDs. There 
 *               are different reasons for blinking:
 *               - An analog Fredi is in connected, but the position of the poti
 *                 mismatches to the current speed of loco. So the blinking green 
 *                 LED shows shows the direction you have to turn the speed 
 *                 button.
 *               - Selftest is active. A running light shows an active selftest.
 *                 If all keys were pressed and the speed button has been turned 
 *                 to the left and to the right, the frequency of blinking is 
 *                 changed. For leaving selftest you have to disconnect the 
 *                 Fredi from loconet and reconnect again.
 *               - While throttle is dispatching an alterning blinking of red 
 *                 and green LEDs shows this state
 *******************************************************************************
 * ARGUMENTS   : void *UserPointer
 * RETURN VALUE: byte, depence on blinkfrequency
 *******************************************************FunctionHeaderEnd******/
byte LEDTimerAction( void *UserPointer)
{
/******************************************************************************/
// mismatching speed
/******************************************************************************/
  if (bThrState == THR_STATE_CONNECTED)
  {
    if (  (bFrediVersion == FREDI_VERSION_ANALOG)
          && (!fSetSpeed))                         // if analog value does not correspond, show blinking
    {
			if (rSlot.spd > potAdcSpeedValue)         // speed is higher than position of poti
			{
				// -> speed up (turn right)
				if (bit_is_clear(LED_PORT, LED_GREEN_R))
				{
					LED_PORT |= _BV(LED_GREEN_R); 
				}
				else
				{
					LED_PORT &= ~_BV(LED_GREEN_R); 
				}
			}
			else                                      // speed is lower than position of poti
			{
				// -> speed down (turn left)             
				if (bit_is_clear(LED_PORT, LED_GREEN_L))
				{
					LED_PORT |= _BV(LED_GREEN_L); 
				}
				else
				{
					LED_PORT &= ~_BV(LED_GREEN_L); 
				}
			}
    }
    else                                        // no use for blinking anymore, so stop timer
    {
   		bLEDReload = LED_ON;
    }
  }
/******************************************************************************/
// Selftest
/******************************************************************************/
  else if (bThrState >= THR_STATE_SELFTEST)     // while selftest is active show rotating LEDs
  {
    // -> fast rotation shows selftest active
    if (bit_is_set(LED_PORT, LED_GREEN_R))      // -> slow rotation shows selftest done
    {
      LED_PORT &= ~_BV(LED_GREEN_R);
      LED_PORT |=  _BV(LED_GREEN_L); 
      LED_PORT &= ~_BV(LED_RED); 
    }
    else if (bit_is_set(LED_PORT, LED_GREEN_L))
    {
      LED_PORT &= ~_BV(LED_GREEN_R);
      LED_PORT &= ~_BV(LED_GREEN_L); 
      LED_PORT |=  _BV(LED_RED);
    }
    else
    {
      LED_PORT |=  _BV(LED_GREEN_R);
      LED_PORT &= ~_BV(LED_GREEN_L); 
      LED_PORT &= ~_BV(LED_RED); 
    }
  }
/******************************************************************************/
// dispatching
/******************************************************************************/
  else  // show alternating blinking between red and green 
  {
    if ( bit_is_set(LED_PORT, LED_GREEN_L))
    {
      LED_PORT |=  _BV(LED_RED); 
      LED_PORT &= ~_BV(LED_GREEN_R) ; 
      LED_PORT &= ~_BV(LED_GREEN_L) ; 
    }
    else
    {
      LED_PORT &= ~_BV(LED_RED); 
      LED_PORT |=  _BV(LED_GREEN_R) ; 
      LED_PORT |=  _BV(LED_GREEN_L) ; 
    }
  }
  return bLEDReload;
}


/******************************************************FunctionHeaderBegin******
 * FUNCTION    : KeyTimerAction
 * CREATED     : 2004-12-20
 * AUTHOR      : Olaf Funke
 * DESCRIPTION : polling of keys, the direction switch and the analog value
 *               with 2ms cycle
 *               This is called from processTimerActions()
 *******************************************************************************
 * ARGUMENTS   : void *UserPointer
 * RETURN VALUE: byte, allways KEY_POLL_TIME (2ms)
 *******************************************************FunctionHeaderEnd******/
byte KeyTimerAction( void *UserPointer)
{
  static  byte bLastEncSwitch   = 1;
  byte bActEncSwitch;
  static  byte bLastKey         = 1;
  byte bActKey;
  static  byte bLastDirSwitch   = 1; 
  byte bActDirSwitch;

/******************************************************************************/
//  keys
/******************************************************************************/
  bActKey = KEYPIN_PIN & KEYPIN_ALL ; // 0 means pressed, 1 means released

  if (bActKey != bLastKey)
  {
    bEvent      |= EVENT_KEY;
    bLastKey     = bActKey;

    bCurrentKey &= ~KEYPIN_ALL;               // clear all possible keys
    bCurrentKey |= (~bActKey) & KEYPIN_ALL;   // set relevant keys
  }

/******************************************************************************/
//  Stop button or increment button
/******************************************************************************/

  bActEncSwitch = ENC_PIN & _BV(ENC_SWITCH);

  if (bActEncSwitch != bLastEncSwitch)  // Change from 0->1 and 1->0
  {
    bEvent        |= EVENT_KEY;
    bLastEncSwitch = bActEncSwitch;

    if (bActEncSwitch)                          // 0 means switch is pressed, 1 key is releaseds
    {
      bCurrentKey &= ~Key_Stop;                 // released key stop
    }
    else
    {
      bCurrentKey |= Key_Stop;                  // pressed key stop
    }
  }

/******************************************************************************/
//  direction button
/******************************************************************************/

  if (  (bFrediVersion == FREDI_VERSION_INCREMENT_SWITCH)
     || (bFrediVersion == FREDI_VERSION_ANALOG          ))
  {
    bActDirSwitch = DIRSWITCH_PIN & _BV(DIRSWITCH);

    if (bActDirSwitch != bLastDirSwitch)         // change of direction
    {
      bEvent        |= EVENT_KEY;
      bLastDirSwitch = bActDirSwitch;

      if (bActDirSwitch)
      {
        bCurrentKey &= ~Key_Dir;
      }
      else
      {
        bCurrentKey |= Key_Dir;
      }
    }
  }

/******************************************************************************/
//  poti
/******************************************************************************/

  if (bFrediVersion == FREDI_VERSION_ANALOG)
  {
    potAdcTimerAction();
  }

/******************************************************************************/

  return KEY_POLL_TIME ;
}


/******************************************************FunctionHeaderBegin******
 * FUNCTION    : MessageTimerAction
 * CREATED     : 2005-04-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION : Timer for repeat of Messages. There are three possibilities
 *               - In state "Connected" there is a cyclic sending of speed 
 *                 message for announcing throttle is still alive
 *               - In case of dispatching there it could be that the message 
 *                 hasn't been sent out correctly. So send message again, if 
 *                 there was no reaction of further message
 *               - In state "init" or unconnected is nothing to do so stop timer
 *******************************************************************************
 * ARGUMENTS   : void *UserPointer
 * RETURN VALUE: byte
 *******************************************************FunctionHeaderEnd******/
byte MessageTimerAction( void *UserPointer)
{
  byte bRetVal = MESSAGE_TIME;

  switch (bThrState)
  {
  case THR_STATE_CONNECTED:
    sendLocoNet4BytePacket(OPC_LOCO_SPD,rSlot.slot,rSlot.spd);
    bRetVal = SPEED_TIME;
    break;
  case THR_STATE_ACQUIRE_LOCO_GET:
  case THR_STATE_ACQUIRE_LOCO_WRITE:
  case THR_STATE_RECONNECT_GET_SLOT:
  case THR_STATE_RECONNECT_WRITE:
  case THR_STATE_RECONNECT_NULL_MOVE:
    vSetState(THR_STATE_RECONNECT_GET_SLOT);
    if (sendLocoNet4BytePacket(OPC_LOCO_ADR, rSlot.adr2, rSlot.adr) != LN_DONE)
    {
      bRetVal = MESSAGE_TIME;
    }
    else
    {
      bRetVal = RESPONSE_TIME;
    }
    break;
  case THR_STATE_UNCONNECTED_WRITE:
    {
      lnMsg SendPacket ;

      SendPacket.sd.command   = OPC_WR_SL_DATA  ; //opcode
      SendPacket.sd.mesg_size = 14              ; // length
      SendPacket.sd.slot      = rSlot.slot      ; // slot    2    
      SendPacket.sd.stat      = rSlot.stat      ; // stat    3    
      SendPacket.sd.adr       = rSlot.adr       ; // adr     4    
      SendPacket.sd.spd       = rSlot.spd       ; // spd     5    
      SendPacket.sd.dirf      = rSlot.dirf      ; // dirf    6    
      SendPacket.sd.trk       = rSlot.trk       ; // trk     7    
      SendPacket.sd.ss2       = rSlot.ss2       ; // ss2     8    
      SendPacket.sd.adr2      = rSlot.adr2      ; // adr2    9    
      SendPacket.sd.snd       = rSlot.snd       ; // snd    10    
      SendPacket.sd.id1       = rSlot.id1       ; // id1    11   
      SendPacket.sd.id2       = rSlot.id2       ; // id2    12   

      LN_STATUS status = sendLocoNetPacket( &SendPacket );

      if (status != LN_DONE)
      {
        bRetVal = MESSAGE_TIME;
      }
      else
      {
        bRetVal = RESPONSE_TIME;
      }
    }
    break;
  case THR_STATE_INIT:          // stop timer, there is nothing to do
  case THR_STATE_UNCONNECTED:
  default:
    bRetVal = 0;
    break;
  }

  return bRetVal;
}


byte ReleaseStopTimerAction( void *UserPointer)
{
	bStopPressed = FALSE;
	return 0;
}


/******************************************************FunctionHeaderBegin******
 * CREATED     : 2004-12-20
 * AUTHOR      : Olaf Funke
 * DESCRIPTION : Set hardware for the keys and LEDs to defined state
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void initKeys( void )
{
  /***************************************/
  //  init encoder or poti
  /***************************************/

  if (bFrediVersion == FREDI_VERSION_ANALOG)
  {
    potAdcInit();
  }
  else // FREDI_VERSION_INCREMENT or FREDI_VERSION_INCREMENT_SWITCH
  {
    potAdcPowerOff();
		bStopPressed = FALSE;
		addTimerAction(&ReleaseStopTimer, 0, ReleaseStopTimerAction, 0, TIMER_SLOW ) ;
    // set data direction register for encoder
    ENC_DDR &= ~( _BV(ENC_BIT_0) | _BV(ENC_BIT_1) ) ;

    // Enable the pull-ups
    ENC_PORT |= ( _BV(ENC_BIT_0) | _BV(ENC_BIT_1) ) ;

    addTimerAction(&IncrementTimer, 0, IncrementTimerAction, 0, TIMER_FAST) ;

    // detect actual state of pin to set next interrupt edge
    ENC_ISC_REG |= _BV(ENC_ISC_BIT1);

    if ( bit_is_set(ENC_PIN, ENC_BIT_1))
    { // set falling edge
      ENC_ISC_REG &= ~_BV(ENC_ISC_BIT0);
    }
    else
    { // set rising edge
      ENC_ISC_REG |= _BV(ENC_ISC_BIT0);
    }

    // clear pending interrupt
    ENC_EIRF_REG |= _BV(ENC_EIRF_BIT);
    // set interrupt activ
    ENC_EIRE_REG |= _BV(ENC_EIRE_BIT);
  }

  /***************************************/
  //  init keys
  /***************************************/

  // set data direction register for encoder
  ENC_DDR &= ~( _BV(ENC_SWITCH) ) ;

  // Enable the pull-ups
  ENC_PORT |= ( _BV(ENC_SWITCH) ) ;

  // set data direction register for encoder
  ENC_DDR &= ~( _BV(ENC_SWITCH) ) ;

  // Enable the pull-ups
  ENC_PORT |= ( _BV(ENC_SWITCH) ) ;

  // set data direction register for keys
  KEYPIN_DDR  &= ~KEYPIN_ALL ;
  // Enable the pull-ups
  KEYPIN_PORT |=  KEYPIN_ALL ;

  if (  (bFrediVersion == FREDI_VERSION_INCREMENT_SWITCH)
        || (bFrediVersion == FREDI_VERSION_ANALOG          ))
  {
    // set data direction register for direction switch
    DIRSWITCH_DDR   &= ~( _BV(DIRSWITCH) );
    // Enable the pull-up
    DIRSWITCH_PORT  |=  ( _BV(DIRSWITCH) );
  }

  addTimerAction(&KeyTimer, KEY_POLL_TIME, KeyTimerAction, 0, TIMER_FAST ) ;

  /***************************************/
  //  init LEDs
  /***************************************/

  LED_DDR  |=  _BV(LED_GREEN_L); 
  LED_PORT &= ~_BV(LED_GREEN_L); 

  LED_DDR  |=  _BV(LED_GREEN_R); 
  LED_PORT &= ~_BV(LED_GREEN_R); 

  LED_DDR  |=  _BV(LED_RED); 
  LED_PORT |= _BV(LED_RED);       // set red LED at startup

  addTimerAction(&LEDTimer, LED_BLINK_TIME, LEDTimerAction, 0, TIMER_SLOW ) ;
}


/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-21
 * AUTHOR      : Olaf Funke
 * DESCRIPTION : Hardware interrupt for the increment decoder. On every interrupt
 *               it is nessecary to look for the edge and the level of the other
 *               encoder pin. A result of this data is a step either in right (++)
 *               or in left direction (--)
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
ISR(ENC_INT_vect)
{
  // set interrupt inactiv while debouncing is active
  ENC_EIRE_REG &= ~_BV(ENC_EIRE_BIT);

  if ( bit_is_set(ENC_ISC_REG, ENC_ISC_BIT0))
  { // falling edge
    if ( bit_is_set(ENC_PIN, ENC_BIT_0))
    { //++
      sEncDir--;
    }
    else
    { //--
      sEncDir++;
    }
  }
  else
  { // rising edge
    if ( bit_is_set(ENC_PIN, ENC_BIT_0))
    { //--
      sEncDir++;
    }
    else
    { //++
      sEncDir--;
    }
  }
}


/******************************************************FunctionHeaderBegin******
 * CREATED     : 2004-12-20
 * AUTHOR      : Olaf Funke
 * DESCRIPTION : Every change of state is handed by this function. A change of 
 *               state is responsible for changing the LEDs and starting the
 *               blinking timer
 *******************************************************************************
 * ARGUMENTS   : byte bState, new state to set, could be the same as before
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void vSetState( byte bState )
{
  bThrState = bState;

  switch (bThrState)
  {
  case THR_STATE_UNCONNECTED:       // show red LED
  case THR_STATE_INIT:
  case THR_STATE_UNCONNECTED_WRITE:
    LED_PORT &= ~_BV(LED_GREEN_R);
    LED_PORT &= ~_BV(LED_GREEN_L); 
    LED_PORT |=  _BV(LED_RED); 
    bLEDReload = LED_ON;
    break;

  case THR_STATE_CONNECTED:         // show direction at state connected
    if (rSlot.dirf & 0x20)
    {
      LED_PORT &= ~_BV(LED_GREEN_R);
      LED_PORT |=  _BV(LED_GREEN_L); 
    }
    else
    {
      LED_PORT &= ~_BV(LED_GREEN_L); 
      LED_PORT |=  _BV(LED_GREEN_R);
    }
    LED_PORT &= ~_BV(LED_RED);

    if (  (bFrediVersion == FREDI_VERSION_ANALOG)
          && (!fSetSpeed))                 // if analog value does not correspond, show blinking
    {
      bLEDReload = LED_BLINK_TIME;
    }
    else
    {
      bLEDReload = LED_ON;
    }
    break;

  case THR_STATE_ACQUIRE_LOCO_GET:
  case THR_STATE_ACQUIRE_LOCO_WRITE:
  case THR_STATE_RECONNECT_GET_SLOT:
  case THR_STATE_RECONNECT_NULL_MOVE:
  case THR_STATE_RECONNECT_WRITE:
    bLEDReload = LED_BLINK_TIME;
    break;
  case THR_STATE_SELFTEST:
    bLEDReload = LED_SELFTEST_TIME;
    break;
  case THR_STATE_SELFTEST_DONE:
    bLEDReload = LED_SELFTEST_DONE_TIME;
    break;
  default:                                 // not allowed state, show by slow blinking
    bLEDReload = (LED_BLINK_TIME*10);
    break;
  }

  resetTimerAction(&LEDTimer, bLEDReload);
}  


/******************************************************FunctionHeaderBegin******
 * CREATED     : 2004-12-20
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: int, never reached, cause of endless loop
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
int main(void)
{
  RESET_RESET_SOURCE(); // Clear Reset Status Register (WDRF,BORF,EXTRF,PORF)

  byte bCount = 0;

  /***************************************/
  //  init analog input for getting 
  //  FrediVersion
  /***************************************/

  DDRC  &= ~_BV(DDC5); // set version detector to tristate to get kind of fredi
  PORTC |=  _BV(PC5);

  if ( bit_is_set(PINC, PINC5))
  {
    bFrediVersion = FREDI_VERSION_ANALOG;
  }
  else
  {
    bFrediVersion = FREDI_VERSION_INCREMENT;
  }
  
  /***************************************/
  //  init throttle slot
  /***************************************/

  bSpdCnt = 0;

  rSlot.command   = OPC_WR_SL_DATA;
  rSlot.mesg_size = 14;
  rSlot.slot      = 0;                        /* slot number for this request                         */
  rSlot.stat      = 0;                        /* slot status                                          */
  rSlot.adr       = 0;                        /* loco address                                         */
  rSlot.spd       = 0;                        /* command speed                                        */
  rSlot.dirf      = 0;                        /* direction and F0-F4 bits                             */
  rSlot.trk       = 0;                        /* track status                                         */
  rSlot.ss2       = 0;                        /* slot status 2 (tells how to use ID1/ID2 & ADV Consist*/
  rSlot.adr2      = 0;                        /* loco address high                                    */
  rSlot.snd       = 0;                        /* Sound 1-4 / F5-F8                                    */

  if ((eeprom_read_byte(&abEEPROM[EEPROM_IMAGE]) != EEPROM_IMAGE_DEFAULT))
  {
    vSetState(THR_STATE_SELFTEST);

    eeprom_write_byte(&abEEPROM[EEPROM_ADR_LOCO_LB], 0);                 // no loco active at selftest
    eeprom_write_byte(&abEEPROM[EEPROM_ADR_LOCO_HB], 0);

    eeprom_write_byte(&abEEPROM[EEPROM_DECODER_TYPE], EEPROM_DECODER_TYPE_DEFAULT);

    eeprom_write_byte(&abEEPROM[EEPROM_SW_INDEX_HB], HIBYTE(SW_INDEX));  // write Version in EEPROM on first startup
    eeprom_write_byte(&abEEPROM[EEPROM_SW_INDEX_LB], LOBYTE(SW_INDEX));

    eeprom_write_byte(&abEEPROM[EEPROM_SW_DAY],      SW_DAY);            // write date of SW in EEPROM
    eeprom_write_byte(&abEEPROM[EEPROM_SW_MONTH],    SW_MONTH);
    eeprom_write_byte(&abEEPROM[EEPROM_SW_YEAR],     SW_YEAR);

    eeprom_write_byte(&abEEPROM[EEPROM_VERSION],     bFrediVersion);     // store detected HW version
  }
  else
  { // selftest was successful before
    if (  (eeprom_read_byte(&abEEPROM[EEPROM_SW_INDEX_HB]) != HIBYTE(SW_INDEX))
          || (eeprom_read_byte(&abEEPROM[EEPROM_SW_INDEX_LB]) != LOBYTE(SW_INDEX))
          || (eeprom_read_byte(&abEEPROM[EEPROM_SW_DAY])      != SW_DAY)
          || (eeprom_read_byte(&abEEPROM[EEPROM_SW_MONTH])    != SW_MONTH)
          || (eeprom_read_byte(&abEEPROM[EEPROM_SW_YEAR])     != SW_YEAR))
    { // sw index or date has changed
      eeprom_write_byte(&abEEPROM[EEPROM_SW_INDEX_HB], HIBYTE(SW_INDEX));// write Version in EEPROM on first startup
      eeprom_write_byte(&abEEPROM[EEPROM_SW_INDEX_LB], LOBYTE(SW_INDEX));

      eeprom_write_byte(&abEEPROM[EEPROM_SW_DAY],      SW_DAY);          // write date of SW in EEPROM
      eeprom_write_byte(&abEEPROM[EEPROM_SW_MONTH],    SW_MONTH);
      eeprom_write_byte(&abEEPROM[EEPROM_SW_YEAR],     SW_YEAR);
    }

    vSetState(THR_STATE_INIT);
    rSlot.adr   = eeprom_read_byte(&abEEPROM[EEPROM_ADR_LOCO_LB]);
    rSlot.adr2  = eeprom_read_byte(&abEEPROM[EEPROM_ADR_LOCO_HB]);
    rSlot.stat  = eeprom_read_byte(&abEEPROM[EEPROM_DECODER_TYPE]);
  }

  rSlot.id1   = eeprom_read_byte(&abEEPROM[EEPROM_ID1]); // get ID from EEPROM
  rSlot.id2   = eeprom_read_byte(&abEEPROM[EEPROM_ID2]);

  if ((rSlot.id1 & 0x80) || (rSlot.id2 & 0x80))
  { // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // if no slot ID was programmed, you get the ID "0xff 0xff"
		// or if an unguilty ID was programmed
    // stop program at this point and switch all leds on
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    LED_DDR  |= _BV(LED_GREEN_L); 
    LED_PORT |= _BV(LED_GREEN_L); 
    LED_DDR  |= _BV(LED_GREEN_R); 
    LED_PORT |= _BV(LED_GREEN_R); 
    LED_DDR  |= _BV(LED_RED); 
    LED_PORT |= _BV(LED_RED);
    while (1);
  }
  
  #ifdef LOCONET_LEVEL_TEST
    LED_DDR  |= _BV(LED_GREEN_L);
    LED_PORT |= _BV(LED_GREEN_L);
    LED_DDR  |= _BV(LED_GREEN_R);
    LED_PORT |= _BV(LED_GREEN_R);
    LED_DDR  |= _BV(LED_RED);
    LED_PORT |= _BV(LED_RED);
    ACSR  = (0<<ACD)  | (0<<ACBG) | (0<<ACO)   | (0<<ACI)
          | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);
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
    while (1) {
      if( bit_is_set(ACSR,ACO) ) {
        LED_PORT |= _BV(LED_GREEN_L);
        LED_PORT |= _BV(LED_GREEN_R);
        LED_PORT |= _BV(LED_RED);
      } else {
        LED_PORT &= ~_BV(LED_GREEN_L);
        LED_PORT &= ~_BV(LED_GREEN_R);
        LED_PORT &= ~_BV(LED_RED);
      }
    }
  #endif

  /***************************************/
  //  init loconet
  /***************************************/

  initLocoNet(&RxBuffer) ;

  /***************************************/
  //  init keys and timer
  /***************************************/

  initKeys();
  initTimer();

  addTimerAction(&MessageTimer, 0, MessageTimerAction, 0, TIMER_SLOW ) ;

  /***************************************/
  //  set state and start interrupts
  /***************************************/

  sei();

  if (bThrState < THR_STATE_SELFTEST)
  {
    // if a address for a loco is available, show blinking state
    if ((rSlot.adr != 0) || (rSlot.adr2 != 0))
    {
      vSetState(THR_STATE_RECONNECT_GET_SLOT);
    }
    else
    {
      vSetState(THR_STATE_UNCONNECTED);
    }

    while (bit_is_clear(ACSR, ACO))     // wait for start of loconet
    {
      processTimerActions();
    }
    // loconet is available, now
    for (bCount =0;bCount < 50;bCount++)
    {
      delayTimer( 10 );                 // wait a little bit longer
      processTimerActions();
    }

    if (rSlot.adr)                      // wait for a pseudo random time
    {
      delayTimer(rSlot.adr);
    }

    if (rSlot.adr2)
    {
      delayTimer(rSlot.adr2);
    }

    // if a address for a loco is available, try to reconnect
    if (bThrState == THR_STATE_RECONNECT_GET_SLOT)
    {
      sendLocoNetAdr(&rSlot);
    }
  }
  else
  {
    sendLocoNetFredCd( bCount );
  }

/******************************************************************************/
// main endless loop 
/******************************************************************************/

  while (1)
  {
	for (int i = 0; i < numberOfSlots; i++) {  
		vProcessRxLoconetMessage(&rSlot);
		vProcessKey(&rSlot);
		vProcessRxLoconetMessage(&rSlot);

		if (bFrediVersion == FREDI_VERSION_ANALOG) {
			vProcessPoti(&rSlot);
		}
		else
		{
			vProcessEncoder(&rSlot);
		}
		vProcessRxLoconetMessage(&rSlot);
		processTimerActions();
	} //end of for
  } // end of while(1)
} // end of main


/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void vProcessRxLoconetMessage(rwSlotDataMsg *currentSlot)
{
  RxPacket = recvLocoNetPacket() ;

  if (RxPacket)
  {
    switch (RxPacket->data[0]) // OP_CODE
    {
/***************************************/
//  Slot Read Data
/***************************************/
    case OPC_SL_RD_DATA:
      switch (bThrState)
      {
      case THR_STATE_ACQUIRE_LOCO_GET: // response of Dispatch Get
        {
          if ((RxPacket->data[3] & LOCO_IDLE) == LOCO_IDLE)
          {
            vCopySlotFromRxPacket();
            vSetState(THR_STATE_ACQUIRE_LOCO_WRITE);
            sendLocoNetWriteSlotData(&currentSlot);
          }
        }
        break;
      case THR_STATE_RECONNECT_GET_SLOT: // response of Get Slot By Adress
        {
          if (  (currentSlot->adr  == RxPacket->data[4])
             && (currentSlot->adr2 == RxPacket->data[9]))
          { // slot not changed and in use , so we can use this slot further on
            if ((RxPacket->data[3] & LOCO_IN_USE) == LOCO_IN_USE)
            {
              if (  (  (currentSlot->id1 == RxPacket->data[11]) && (currentSlot->id2 == RxPacket->data[12]))
                 || (  (0         == RxPacket->data[11]) && (0         == RxPacket->data[12])))
              {
                vCopySlotFromRxPacket();
                vSetState(THR_STATE_RECONNECT_WRITE);
                sendLocoNetWriteSlotData(&currentSlot);
              }
              else
              {
                vSetUnconnected();
              }
            }
            else
            {
              vSetState(THR_STATE_RECONNECT_NULL_MOVE);
              sendLocoNetMove(RxPacket->data[2], RxPacket->data[2]);
            }
          }
        }
        break;
      case THR_STATE_RECONNECT_NULL_MOVE:
        {
          if (  (currentSlot->adr  == RxPacket->data[4]) 
             && (currentSlot->adr2 == RxPacket->data[9]))
          { // slot not changed and in use , so we can use this slot further on
            if ((RxPacket->data[3] & LOCO_IN_USE) == LOCO_IN_USE)
            {
              vCopySlotFromRxPacket();
              vSetState(THR_STATE_RECONNECT_WRITE);
              sendLocoNetWriteSlotData(&currentSlot);
            }
            else
            {
              vSetUnconnected();
            }
          }
        }
        break;
      }
      break;
/***************************************/
//  Long Acknowledge
/***************************************/
    case OPC_LONG_ACK:
      switch (bThrState)
      {
      case THR_STATE_UNCONNECTED_WRITE:
        if (RxPacket->data[1] == (OPC_WR_SL_DATA & 0x7f))
        {
          vSetUnconnected();
        }
        break;
      case THR_STATE_RECONNECT_WRITE: // response of Get Slot By Adress
      case THR_STATE_ACQUIRE_LOCO_WRITE:
        if (RxPacket->data[1] == (OPC_WR_SL_DATA & 0x7f))
        {
          eeprom_write_byte(&abEEPROM[EEPROM_ADR_LOCO_LB],  currentSlot->adr);
          eeprom_write_byte(&abEEPROM[EEPROM_ADR_LOCO_HB],  currentSlot->adr2);
          eeprom_write_byte(&abEEPROM[EEPROM_DECODER_TYPE], currentSlot->stat & DEC_MODE_MASK);

          vSetState(THR_STATE_CONNECTED);
        }
        break;
      case THR_STATE_ACQUIRE_LOCO_GET: // response of Dispatch Get
        vSetUnconnected();
        break;
      }
      break;
/***************************************/
//  Set Slot Speed
/***************************************/
    case OPC_LOCO_SPD:
      if (  (bThrState         == THR_STATE_CONNECTED)
         && (RxPacket->data[1] == currentSlot->slot         ))
      {
        int i;

        currentSlot->spd = RxPacket->data[2];
        
        GET_SPDCNT_BY_SLOTSPD
      }
      break;
/***************************************/
//  Set Slot Direction and F0 to F4
/***************************************/
    case OPC_LOCO_DIRF:
      if (  (bThrState         == THR_STATE_CONNECTED)
         && (RxPacket->data[1] == currentSlot->slot         ))
      {

        if (  (bFrediVersion == FREDI_VERSION_INCREMENT_SWITCH)
              || (bFrediVersion == FREDI_VERSION_ANALOG          ))
        {
          // if switch is not in direction of direction flag, so change 
          // directionflag to synchronize the direction
          if ( (currentSlot->dirf & 0x20) != (RxPacket->data[2] & 0x20))
          {
            currentSlot->dirf &= 0x20;                       // get direction of fredi
            currentSlot->dirf |= (RxPacket->data[2] & ~0x20); // and add F0..F4
            sendLocoNetDirf(&currentSlot);
          }
          else
          {
            currentSlot->dirf = RxPacket->data[2];          // direction is equal, so take F0..F4
          }
        }
        else
        { // take direction as new one
          currentSlot->dirf = RxPacket->data[2];
        }

        if (currentSlot->dirf & 0x20)
        {
          LED_PORT &= ~_BV(LED_GREEN_R);
          LED_PORT |=  _BV(LED_GREEN_L); 
        }
        else
        {
          LED_PORT &= ~_BV(LED_GREEN_L); 
          LED_PORT |=  _BV(LED_GREEN_R);
        }
      }
      break;
/***************************************/
//  Set Slot Sound Functions, 
/***************************************/
    case OPC_LOCO_SND:
      if (  (bThrState         == THR_STATE_CONNECTED)
         && (RxPacket->data[1] == currentSlot->slot         ))
      {
        currentSlot->snd = RxPacket->data[2];
      }
      break;
/***************************************/
//  All other, not used at the moment
/***************************************/
    default:
      break;
    }

    RxPacket = 0;  
  }
}


/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *
 * Bug: if you set shift and another key and you release the shift key first
 *      the remaining key is executed.
 *      So press shift, then press Fx key and release key Fx first before 
 *      releasing the shift key
 *******************************************************FunctionHeaderEnd******/
void vProcessKey(rwSlotDataMsg *currentSlot)
{
  if (bEvent & EVENT_KEY)
  {
    static byte bLastCurrentkey = 0;
    byte bSet;
    bEvent &= ~EVENT_KEY;

    if (bThrState < THR_STATE_SELFTEST)
    {
      if ((bLastCurrentkey & Key_SHIFT) && ( !(bCurrentKey & Key_SHIFT)))
      { // the changing key was a release of shift -> no action
      }
			else if ((bLastCurrentkey & Key_Stop) && ( !(bCurrentKey & Key_Stop)))
			{
				resetTimerAction(&ReleaseStopTimer, RELEASE_STOP_TIME); 
			}
      else if ((bLastCurrentkey & Key_Dir) != (bCurrentKey & Key_Dir))
      { // dir switch changed
        if (  (bThrState == THR_STATE_CONNECTED)
           && (  (bFrediVersion == FREDI_VERSION_ANALOG)
              || (bFrediVersion == FREDI_VERSION_INCREMENT_SWITCH)))
        {
          sEncDir = 0;
          bSpdCnt = 0;

          if (  (bFrediVersion == FREDI_VERSION_ANALOG)
                && (currentSlot->spd > 1))
          {
            fSetSpeed = FALSE;
            vSetState(THR_STATE_CONNECTED);
          }

          currentSlot->spd = 1; // Not stop

          sendLocoNetSpd(&currentSlot);

          if (bCurrentKey & Key_Dir)
          { // dir switch was pressed
            currentSlot->dirf |= 0x20;
            LED_PORT   &= ~_BV(LED_GREEN_L); 
            LED_PORT   |=  _BV(LED_GREEN_R);
          }
          else
          { // dir switch was released
            currentSlot->dirf &= ~0x20;
            LED_PORT &= ~_BV(LED_GREEN_R);
            LED_PORT |=  _BV(LED_GREEN_L); 
          }
          sendLocoNetDirf(&currentSlot);

          // Fredi is connected, so this causes an sendLocoNetSpd after 100ms
          // it seems to be the last sendLocoNetSpd is ignored by intellibox in some cases
          resetTimerAction(&MessageTimer, 1); 
        }
      }
      else
      {
        if (bThrState == THR_STATE_CONNECTED)
        {
          if (bCurrentKey & Key_SHIFT)
          {
            bSet = currentSlot->snd; 

            switch (bCurrentKey & ~Key_Dir)
            {
            case (Key_Stop | Key_SHIFT): // undispatch
              currentSlot->stat = 0x20;

              vSetState(THR_STATE_UNCONNECTED_WRITE);

              sendLocoNetWriteSlotData(&currentSlot);
              break;
            case Key_F5:  bSet ^= 0x01; break;
            case Key_F6:  bSet ^= 0x02; break;
            case Key_F7:  bSet ^= 0x04; break;
            case Key_F8:  bSet ^= 0x08; break;
            default:                    break;
            }

            if (bSet != currentSlot->snd)
            {
              currentSlot->snd = bSet; 
              sendLocoNetSnd(&currentSlot);
            }
          }
          else
          {
            bSet = currentSlot->dirf; 

            switch (bCurrentKey & ~Key_Dir)
            {
            case Key_Stop: // increment pushbutton or extra button on analog fredi
              sEncDir = 0;
              bSpdCnt = 0;

              if (  (bFrediVersion == FREDI_VERSION_ANALOG)
                 && (currentSlot->spd > 1))
              {
                fSetSpeed = FALSE;                // show blinking LED
                vSetState(THR_STATE_CONNECTED);
              }
							else
							{
								bStopPressed = TRUE;
							}

              if (currentSlot->spd > 1)
							{
								currentSlot->spd = 1;                      // Emergency stop
							}
							else
							{
								currentSlot->spd = 0;                      // Normal stop
							}

              sendLocoNetSpd(&currentSlot);

              if (bFrediVersion == FREDI_VERSION_INCREMENT) // invert direction
              {
								if (bSet & 0x20)
								{
									bSet &= ~0x20;

									LED_PORT &= ~_BV(LED_GREEN_R);
									LED_PORT |=  _BV(LED_GREEN_L);     // show left direction
								}
								else
								{
									bSet |= 0x20;

									LED_PORT &= ~_BV(LED_GREEN_L);     // show right direction
									LED_PORT |=  _BV(LED_GREEN_R);
								}
              }
              break;
            case Key_F0:  bSet ^= 0x10; break;
            case Key_F1:  bSet ^= 0x01; break;
            case Key_F2:  bSet ^= 0x02; break;
            case Key_F3:  bSet ^= 0x04; break;
            case Key_F4:  bSet ^= 0x08; break;
            default:                    break;
            }

            if (bSet != currentSlot.dirf)
            {
              currentSlot.dirf = bSet; 
              sendLocoNetDirf(&currentSlot);
            }
          } // end of else if(bCurrentKey & Key_SHIFT)
        } // end of if(bThrState == THR_STATE_CONNECTED)
        else if (bThrState == THR_STATE_UNCONNECTED)
        {
          if ((bCurrentKey & ~Key_Dir) == (Key_Stop | Key_SHIFT))   // try to dispatch
          {
            vSetState(THR_STATE_ACQUIRE_LOCO_GET);
            sendLocoNetMove(0, 0);
          }
        }
        else
        {
          if ((bCurrentKey & (Key_Stop | Key_SHIFT)) == (Key_Stop | Key_SHIFT))
          {
            currentSlot->stat = 0x20;

            vSetState(THR_STATE_UNCONNECTED_WRITE);

            sendLocoNetWriteSlotData(&currentSlot);
          }
        }
      }
    }
    else
    {
      wSelfTest |= (uint16_t) (bCurrentKey);
      sendLocoNetFredButton(bCurrentKey);
      vCheckSelfTestEnd();
    }

    bLastCurrentkey = bCurrentKey;
  } // end of if (bEvent & EVENT_KEY)
} // end of void vProcessKey(void)

/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void vProcessEncoder(rwSlotDataMsg *currentSlot)
{
  if (sEncDir != 0)
  {
    resetTimerAction(&IncrementTimer, INCREMENT_TIME); // start timer for debouncing

    if (bThrState == THR_STATE_CONNECTED)
    {
			if (!(bStopPressed))                // get encoder steps only if stop is not pressed
			{
				if (sEncDir < 0)                  // left rotation
				{
					sEncDir *= -1;                  // get absolut value

					if (sEncDir > bSpdCnt)
					{
						bSpdCnt = 0;
					}
					else
					{
						bSpdCnt -= (byte) sEncDir;
					}
				}
				else                              // right rotation
				{
					bSpdCnt += sEncDir;
				}

      if (bSpdCnt > MAX_SPEED)          // limit the value for speed
      {
        bSpdCnt = MAX_SPEED;
      }

      if (currentSlot->spd != abSpd[bSpdCnt])       // get speedvalue for incrementvalue
      {
        currentSlot->spd = abSpd[bSpdCnt];
					sendLocoNetSpd(&currentSlot);           // anounce new speed value
				}
			}
    }
    else if (bThrState >= THR_STATE_SELFTEST)
    {
      if (sEncDir < 0)                  // left rotation decoded
      {
        wSelfTest |= Key_Enc_L;
      }
      else                              // right ratation decoded
      {
        wSelfTest |= Key_Enc_R;
      }
      sendLocoNetFredButton( sEncDir );

      vCheckSelfTestEnd();
    }

    sEncDir = 0;
  }
}

/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void vProcessPoti(rwSlotDataMsg *currentSlot)
{
  if (bThrState == THR_STATE_CONNECTED)
  {
    byte fOldSetSpeed = fSetSpeed;

    if (!fSetSpeed)
    {
      if (potAdcSpeedValue == 0)     // potivalue is in right range for  set speed again
      {
        fSetSpeed = TRUE;
      }
    }

    if (fSetSpeed)
    {
      byte bSpd = potAdcSpeedValue;
      if (currentSlot->spd != bSpd)
      {
        currentSlot->spd = bSpd;
        sendLocoNetSpd(&currentSlot);
      }
    }

    if (fOldSetSpeed != fSetSpeed)
    {
      vSetState(THR_STATE_CONNECTED); // reset of blinking LEDs
    }
  }
  else if (bThrState >= THR_STATE_SELFTEST)
  {
    static uint16_t bOldValue = 0xffff; // init of potAdcSpeedValue is different,
                                        // so set first value anyway

    if (potAdcSpeedValue == 0)          // Poti on left side
    {
      wSelfTest |= Key_Poti_L;
    }
    else if (potAdcSpeedValue >= 126)   // Poti on right side
    {
      wSelfTest |= Key_Poti_R;
    }

    if ( bOldValue != potAdcRawValue )
    {
      bOldValue = potAdcRawValue;
      sendLocoNetFredAdc( potAdcRawValue );
    }

    vCheckSelfTestEnd();
  }
}


/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void sendLocoNetSpd(rwSlotDataMsg *pSlot)
{
  sendLocoNet4BytePacket(OPC_LOCO_SPD,pSlot->slot,pSlot->spd);
  resetTimerAction(&MessageTimer, SPEED_TIME);
}

/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void sendLocoNetDirf(rwSlotDataMsg *pSlot)
{
  sendLocoNet4BytePacket(OPC_LOCO_DIRF,pSlot->slot,pSlot->dirf);
}

/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void sendLocoNetSnd(rwSlotDataMsg *pSlot)
{
  sendLocoNet4BytePacket(OPC_LOCO_SND,pSlot->slot,pSlot->snd);
}

/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void sendLocoNetWriteSlotData(rwSlotDataMsg *pSlot)
{
  lnMsg SendPacket ;

  SendPacket.sd.command   = OPC_WR_SL_DATA  ; //opcode
  SendPacket.sd.mesg_size = 14              ; // length
  SendPacket.sd.slot      = pSlot->slot   ; // slot    2    
  SendPacket.sd.stat      = pSlot->stat   ; // stat    3    
  SendPacket.sd.adr       = pSlot->adr    ; // adr     4    
  SendPacket.sd.spd       = pSlot->spd    ; // spd     5    
  SendPacket.sd.dirf      = pSlot->dirf   ; // dirf    6    
  SendPacket.sd.trk       = pSlot->trk    ; // trk     7    
  SendPacket.sd.ss2       = pSlot->ss2    ; // ss2     8    
  SendPacket.sd.adr2      = pSlot->adr2   ; // adr2    9    
  SendPacket.sd.snd       = pSlot->snd    ; // snd    10    
  SendPacket.sd.id1       = pSlot->id1    ; // id1    11   
  SendPacket.sd.id2       = pSlot->id2    ; // id2    12   

  if (sendLocoNetPacket( &SendPacket ) != LN_DONE)
  { // send message failed, so set new state
    resetTimerAction(&MessageTimer, MESSAGE_TIME);

    switch (bThrState)
    {
    case THR_STATE_ACQUIRE_LOCO_WRITE:
    case THR_STATE_RECONNECT_WRITE:
      vSetState(THR_STATE_RECONNECT_GET_SLOT);
      break;
    case THR_STATE_UNCONNECTED_WRITE:
      vSetState(THR_STATE_CONNECTED);
      break;
    }
  }
  else
  {
    resetTimerAction(&MessageTimer, RESPONSE_TIME);
  }
}

/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void sendLocoNetMove(byte bSrc, byte bDest)
{
  if (sendLocoNet4BytePacket(OPC_MOVE_SLOTS, bSrc, bDest) != LN_DONE)
  {
    resetTimerAction(&MessageTimer, MESSAGE_TIME);
    // send message failed, so set new state
    if (bThrState == THR_STATE_RECONNECT_NULL_MOVE)
    {
      vSetState(THR_STATE_RECONNECT_GET_SLOT);
    }
    else
    {
      vSetState(THR_STATE_UNCONNECTED);
    }
  }
  else
  {
    resetTimerAction(&MessageTimer, RESPONSE_TIME);
  }
}

/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void sendLocoNetAdr(rwSlotDataMsg *pSlot)
{
  if (sendLocoNet4BytePacket(OPC_LOCO_ADR, rSlot.adr2, rSlot.adr) != LN_DONE)
  {
    resetTimerAction(&MessageTimer, MESSAGE_TIME);
  }
  else
  {
    resetTimerAction(&MessageTimer, RESPONSE_TIME);
  }
}

/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
/*void sendLocoNetSelfTest(byte bTestCase, byte bValue)
{
  if (bThrState >= THR_STATE_SELFTEST)
  {
    if (bValue & 0x80)
    {
      bTestCase |=  0x01;
      bValue    &= ~0x80;
    }
    sendLocoNet4BytePacket(OPC_SELFTEST, bTestCase, bValue);
  }
}*/

/**
 * Send ADC raw value according to FRED.
 * @param raw the unfiltered value (0..1023)
 */
void sendLocoNetFredAdc( uint16_t raw )
{
  sendLocoNet4BytePacket( OPC_FRED_ADC, 0x7f & raw, 0x7f & raw>>7 );
}

/**
 * Send button press/release code according to FRED.
 * @param button a code from 1 to 127
 */
void sendLocoNetFredCd( uint8_t cdTime )
{
  sendLocoNet4BytePacket( OPC_FRED_BUTTON, 42, 0x7f & cdTime );
}

/**
 * Send button press/release code according to FRED.
 * @param button a code from 1 to 127
 */
void sendLocoNetFredButton( uint8_t button )
{
  sendLocoNet4BytePacket( OPC_FRED_BUTTON, 43, 0x7f & button );
}
/******************************************************FunctionHeaderBegin******
 * CREATED     : 2005-01-29
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void vCheckSelfTestEnd(void)
{
  byte fSelfTestEnd = FALSE;

  switch (bFrediVersion)
  {
  case FREDI_VERSION_INCREMENT:
    if (wSelfTest == Key_Fredi_Inkrement)
    {
      fSelfTestEnd = TRUE;        
    }
    break;
  case FREDI_VERSION_INCREMENT_SWITCH:
    if (wSelfTest == Key_Fredi_Inkrement_Switch)
    {
      fSelfTestEnd = TRUE;        
    }
    break;
  case FREDI_VERSION_ANALOG:
    if (wSelfTest == Key_Fredi_Poti)
    {
      fSelfTestEnd = TRUE;        
    }
    break;
  default:
    break;
  }

  if (  (fSelfTestEnd == TRUE)
     && (eeprom_read_byte(&abEEPROM[EEPROM_IMAGE]) != EEPROM_IMAGE_DEFAULT))
  {
    vSetState(THR_STATE_SELFTEST_DONE);
    delayTimer( 100 ); // wait a little bit longer
    sendLocoNetFredButton( 0x7F );

    eeprom_write_byte(&abEEPROM[EEPROM_IMAGE], EEPROM_IMAGE_DEFAULT);
  }
}

/******************************************************FunctionHeaderBegin******
 * CREATED     : 2006-04-21
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void vCopySlotFromRxPacket(void)
{
  byte i;                                               // needed for GET_SPDCNT_BY_SLOTSPD

  if (bThrState == THR_STATE_ACQUIRE_LOCO_GET)
  {
    rSlot.stat    = RxPacket->data[ 3];                 // slot status
    rSlot.adr     = RxPacket->data[ 4];                 // loco address
    rSlot.adr2    = RxPacket->data[ 9];                 // loco address high
  }
  else
  {
    rSlot.stat    |= RxPacket->data[ 3] & ~DEC_MODE_MASK; // slot status
  }

  rSlot.slot      = RxPacket->data[ 2];                 // slot number for this request

  if (  (bFrediVersion == FREDI_VERSION_INCREMENT_SWITCH)      
        || (bFrediVersion == FREDI_VERSION_ANALOG          ))
  {
    rSlot.dirf = RxPacket->data[ 6] & ~0x20;            // get direction by switch position                          

    if (bCurrentKey & Key_Dir)
    {
      rSlot.dirf |=  0x20;                                    
    }

    if ((rSlot.dirf & 0x20) != (RxPacket->data[ 6] & 0x20)) // compare Fredi direction with slot direction
    {
      rSlot.spd = 1;                                    // direction isn't matching so stop train 
      fSetSpeed = FALSE;                                // and show blinking
    }
    else if (bFrediVersion == FREDI_VERSION_ANALOG)
    {
      rSlot.spd = potAdcSpeedValue;                     // direction is matching, get speed by poti
    }
    else
    {
      rSlot.spd = RxPacket->data[ 5];                   // a increment-switch-Fredi takes speed from slot
    }                                                         
  }
  else
  {
    rSlot.spd   = RxPacket->data[ 5];                   // command speed
    rSlot.dirf  = RxPacket->data[ 6];                   // direction and function keys
  }                                                           

  rSlot.trk       = RxPacket->data[ 7];                 // track status
  rSlot.ss2       = RxPacket->data[ 8];                 // slot status 2 (tells how to use ID1/ID2 & ADV Consist
  rSlot.snd       = RxPacket->data[10];                 // Sound 1-4 / F5-F8

  GET_SPDCNT_BY_SLOTSPD                                 // calculate real speed
}

/******************************************************FunctionHeaderBegin******
 * CREATED     : 2006-04-21
 * AUTHOR      : Olaf Funke
 * DESCRIPTION :   -
 *******************************************************************************
 * ARGUMENTS   : none
 * RETURN VALUE: none
 * NOTES       :   -
 *******************************************************FunctionHeaderEnd******/
void vSetUnconnected(void)
{
  rSlot.adr   = 0;
  rSlot.adr2  = 0;

  eeprom_write_byte(&abEEPROM[EEPROM_ADR_LOCO_LB],  rSlot.adr);
  eeprom_write_byte(&abEEPROM[EEPROM_ADR_LOCO_HB],  rSlot.adr2);
  eeprom_write_byte(&abEEPROM[EEPROM_DECODER_TYPE], EEPROM_DECODER_TYPE_DEFAULT);

  vSetState(THR_STATE_UNCONNECTED);
}

