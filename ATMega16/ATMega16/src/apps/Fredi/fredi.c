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
/**
 * \file
 */

#include <stdint.h>         // typedef int8_t, typedef uint8_t,
                            // typedef int16_t, typedef uint16_t
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>     // #define EEMEM
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>

#include "sysdef.h"
#include "common_defs.h"
#include "ln_sw_uart.h"
#include "ln_interface.h"
#include "systimer.h"

#include "processor.h"
#include "potAdc.h"         // potAdcSpeedValue, potAdcRawValue
                            // potAdcPowerOff(), potAdcInit(),
                            // potAdcTimerAction()
							
#include <util/delay.h>



/******************************************************************************/
// functions
/******************************************************************************/
void vSetState( byte bState );
void vProcessRxLoconetMessage(void);
void vProcessRxMonitorMessage(void);
void vProcessKey(void);
void vProcessPoti(uint8_t);

void vCopySlotFromRxPacket(void);
void vSetUnconnected(void);

void sendLocoNetSpd(rwSlotDataMsg *pSlot);
void sendLocoNetDirf(rwSlotDataMsg *pSlot);
void sendLocoNetSnd(rwSlotDataMsg *pSlot);
void sendLocoNetWriteSlotData(rwSlotDataMsg *pSlot);
void sendLocoNetMove(byte bSrc, byte bDest);
void sendLocoNetAdr(rwSlotDataMsg *pSlot);
void sendLocoNetFredAdc( uint16_t raw );
void sendLocoNetFredCd( uint8_t cdTime );
void sendLocoNetFredButton( uint8_t button );

inline uint8_t debounce(uint8_t pin, uint8_t port);
uint16_t potiRead(uint8_t ch);

//NILS sendLocoNetAdr()

/******************************************************************************/
// main defines & variables
/******************************************************************************/

#define TICK_RELOAD           (256L - (F_CPU/(TIMER_TICK_FREQUENCY * TIMER_PRESCALER_COUNT)))
#define TIMER_PRESCALER_CODE  4  
#define TIMER_PRESCALER_COUNT 256L

//#define UART					// AUSKOMMENTIEREN WENN UART NICHT ANGESCHLOSSEN IST!

static volatile byte	UART_ON = 0;

static volatile byte	rSlot1State = 0;
static volatile byte	rSlot2State = 0;

static volatile byte	ch = 5;

static volatile byte	debounceCounter		= 0;
static volatile byte	debounceShort		= 4;
static volatile byte	debounceLong		= 12;		// ca 3x vom debounceShort
static volatile byte	debounceMax			= 3;		// need to set ticks for keys
static volatile byte	keyID				= 0;

static volatile byte    lastLED1State     = 0;
static volatile byte    lastLED2State     = 0;
static volatile byte    lastLED3State     = 0;
static volatile byte    lastLED4State     = 0;
static volatile byte	F0counter = 0;

static volatile byte	keyPressed = 0;
static volatile byte	ReglerKeysActive = 0;




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

static byte bSpdCnt[4] = {0};

#define MAX_SPEED  33

const byte abSpd[MAX_SPEED+4] =
{
  0,  3,  5,  7, 11, 15, 19, 23, 27, 31,
  35, 39, 43, 47, 51, 55, 59, 63, 67, 71,
  75, 79, 83, 87, 91, 95, 99,103,107,111,
  115,119,123,127,127,127,127
};

#define GET_SPDCNT_BY_SLOTSPD bSpdCnt[ch] = MAX_SPEED;                                        \
                              for (i = 0; i < MAX_SPEED; i++)                             \
                              {                                                      \
                                if ((abSpd[i] <= rSlot[ch].spd) && (abSpd[i+1] > rSlot[ch].spd))  \
                                {                                                         \
                                  bSpdCnt[ch] = i;                                            \
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

volatile rwSlotDataMsg rSlot[4];
//volatile rwSlotDataMsg rSlotAry[4];

//volatile rwSlotDataMsg rSlot1;
//volatile rwSlotDataMsg rSlot2;
//volatile rwSlotDataMsg rSlot[3];


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
static volatile byte    fSetSpeed[4]     = {TRUE};
byte fOldSetSpeed[4];
byte bSpd[4];
static volatile byte    bStopPressed  = FALSE;

/******************************************************************************/



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
/*
byte MessageTimerAction( void *UserPointer)
{
  byte bRetVal = MESSAGE_TIME;

  switch (bThrState)
  {
  case THR_STATE_CONNECTED:
    sendLocoNet4BytePacket(OPC_LOCO_SPD,rSlot[0].slot,rSlot[0].spd);
    bRetVal = SPEED_TIME;
    break;
  case THR_STATE_ACQUIRE_LOCO_GET:
  case THR_STATE_ACQUIRE_LOCO_WRITE:
  case THR_STATE_RECONNECT_GET_SLOT:
  case THR_STATE_RECONNECT_WRITE:
  case THR_STATE_RECONNECT_NULL_MOVE:
    vSetState(THR_STATE_RECONNECT_GET_SLOT);
    if (sendLocoNet4BytePacket(OPC_LOCO_ADR, rSlot[0].adr2, rSlot[0].adr) != LN_DONE)
    {
      bRetVal = MESSAGE_TIME;
    }
    else
    {
      bRetVal = RESPONSE_TIME;
    }
    break;
  case THR_STATE_UNCONNECTED_WRITE:
    {/*
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
*/

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
	//Regler
	DDR_A   &= ~( _BV(REGLER_1) );
	DDR_A   &= ~( _BV(REGLER_2) );
	DDR_A   &= ~( _BV(REGLER_3) );
	DDR_A   &= ~( _BV(REGLER_4) );
		
	//Tasten
		
	PORT_C |= FUNK_L_1;
	DDR_C   &= ~( _BV(FUNK_L_1) );
		
	PORT_B |= FUNK_L_2;
	DDR_B   &= ~( _BV(FUNK_L_2) );
	
	PORT_C |= FUNK_L_3;
	DDR_C   &= ~( _BV(FUNK_L_3) );
	
	PORT_A |= FUNK_L_4;
	DDR_A   &= ~( _BV(FUNK_L_4) );
		
	PORT_D |=  KEYPIN_FUNK_PORT_D ;
	DDR_D	&= ~(KEYPIN_FUNK_PORT_D);
		
	DDR_C   &= ~( _BV(RICHTG_1) );
	PORT_C  |=  ( _BV(RICHTG_1) );
		
	DDR_A   &= ~( _BV(RICHTG_2) );
	PORT_A  |=  ( _BV(RICHTG_2) );
		
	DDR_C   &= ~( _BV(RICHTG_3) );
	PORT_C  |=  ( _BV(RICHTG_3) );
		
	DDR_C   &= ~( _BV(RICHTG_4) );
	PORT_C  |=  ( _BV(RICHTG_4) );
		
		
	//LEDs
	DDR_A	|=	_BV(LED1);
	PORT_A	&=	~_BV(LED1);
		
	DDR_A	|=	_BV(LED2);
	PORT_A	&=	~_BV(LED2);
		
	DDR_C	|=	_BV(LED3);
	PORT_C	&=	~_BV(LED3);
		
	DDR_C	|=	_BV(LED4);
	PORT_C	&=	~_BV(LED4);
	#if defined(UART)
		printf("Init Keys \n");
	#endif		
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
	  //PORT_A	|=	_BV(LED1);
    bLEDReload = LED_ON;
    break;

  case THR_STATE_CONNECTED:         // show direction at state connected
    if (rSlot[0].dirf & 0x20)
    {

	  PORT_A	|=	_BV(LED1);
    }
    else
    {
	  PORT_A	&=	~_BV(LED1);
    }
    //LED_PORT &= ~_BV(LED_RED);

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
/*UART Functions ******/

#define BAUD 9600UL                                         //Baudrate auf 38.4k gesetzt
#define UBRR_VAL ((F_CPU+BAUD*8)/(BAUD*16)-1)               //clever runden
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))                 //Reale Baudrate
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD)                     //Fehler in Promille, 1000 = kein Fehler.
//#include <util/setbaud.h>
  
static FILE mystdout;
int count;
int ncount;
  
  
int uart_putchar(char c, FILE *stream);                        //Deklaration der primitiven Ausgabefunktion
//Umleiten der Standardausgabe stdout (Teil 1)
static FILE mystdout = FDEV_SETUP_STREAM( uart_putchar, NULL, _FDEV_SETUP_WRITE );
  
  
 int uart_putchar( char c, FILE *stream )                    //Definition der Ausgabefunktion
 {
	if( c == '\n' )
	uart_putchar( '\r', stream );
	  
	loop_until_bit_is_set( UCSRA, UDRE );
	UDR = c;
	return 0;
}
  
 void uart_init(void)                                        //Serielle Schnittstelle initialisiert
 {
	UBRRH = UBRR_VAL >> 8;
	UBRRL = UBRR_VAL & 0xFF;
	  
	UCSRB = (1<<RXCIE)|(1<<RXEN) |(1<<TXEN);                  // UART TX einschalten
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);                  // Asynchron 8N1 Betrieb festlegen
	stdout = &mystdout;
	  
	printf("UART INIT");
	printf("\n ------------------------- \n");
 }

/********************************UARTFunctionsEnd******/

void rSlotInit() {
	for(int i = 0;i <= 3; i++) {
		rSlot[i].command   = OPC_WR_SL_DATA;
		rSlot[i].mesg_size = 14;
		rSlot[i].slot      = 0;                        //slot number for this request
		rSlot[i].stat      = 0;                        // slot status
		rSlot[i].adr       = 0;                        // loco address
		rSlot[i].spd       = 0;						   // command speed
		rSlot[i].dirf      = 0;                        // direction and F0-F4 bits
		rSlot[i].trk       = 0;                        // track status
		rSlot[i].ss2       = 0;                        // slot status 2 (tells how to use ID1/ID2 & ADV Consist
		rSlot[i].adr2      = 0;                        // loco address high
		rSlot[i].snd       = 0;                        // Sound 1-4 / F5-F8
	}	
}

void rSlotReconnect() {
			if(rSlot[0].adr != 0 || rSlot[0].adr2 != 0) {
				//printf("sendLocoNetAdr : %u \n", rSlot[0].adr);
				bThrState = THR_STATE_RECONNECT_GET_SLOT;
				ch = 0;
				sendLocoNetAdr(&rSlot[0]);
				rSlot1State = 1;
			}
			#if defined(UART)
			printf("ch %u \n",ch);
			#endif
			_delay_ms(1000);
			vProcessRxLoconetMessage();
			vProcessRxLoconetMessage();

			if(rSlot[1].adr != 0 || rSlot[1].adr2 != 0) {
				//printf("sendLocoNetAdr : %u \n", rSlot[1].adr);
				bThrState = THR_STATE_RECONNECT_GET_SLOT;
				ch = 1;
				sendLocoNetAdr(&rSlot[1]);
				rSlot2State = 1;
			}
			#if defined(UART)
			printf("ch %u \n",ch);
			#endif

}
void setUnconnectAll() {
		ch = 0;
		vSetUnconnected();
		ch = 1;
		vSetUnconnected();
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
	
	#if defined(UART)
		uart_init();
	#endif	

	RESET_RESET_SOURCE(); // Clear Reset Status Register (WDRF,BORF,EXTRF,PORF)
	  
	byte bCount = 0;
	bFrediVersion = FREDI_VERSION_ANALOG;	
	  
	//bSpdCnt = 0;
	
	rSlotInit();

/*
		rSlot[0].command   = OPC_WR_SL_DATA;
		rSlot[0].mesg_size = 14;
		rSlot[0].slot      = 0;                        //slot number for this request
		rSlot[0].stat      = 0;                        // slot status
		rSlot[0].adr       = 0;                        // loco address
		rSlot[0].spd       = 0;						// command speed
		rSlot[0].dirf      = 0;                        // direction and F0-F4 bits
		rSlot[0].trk       = 0;                        // track status
		rSlot[0].ss2       = 0;                        // slot status 2 (tells how to use ID1/ID2 & ADV Consist
		rSlot[0].adr2      = 0;                        // loco address high
		rSlot[0].snd = 0;								// Sound 1-4 / F5-F8	
*/		
	//printf("rslot 1: %u rslot 2 %u \n",rSlot[0].adr ,rSlot[1].adr);
	/*
	ch = 0;
	vSetUnconnected();
	ch = 1;
	vSetUnconnected();
	*/
	if(rSlot[0].adr == 0) {

		rSlot[0].adr		= eeprom_read_byte(&abEEPROM[EEPROM_ADR_R1_LOCO_LB]);
		rSlot[0].adr2		= eeprom_read_byte(&abEEPROM[EEPROM_ADR_R1_LOCO_HB]);
		rSlot[0].stat		= eeprom_read_byte(&abEEPROM[EEPROM_R1_STAT]);

	}
	
	if(rSlot[1].adr == 0) {
		rSlot[1].adr		= eeprom_read_byte(&abEEPROM[EEPROM_ADR_R2_LOCO_LB]);
		rSlot[1].adr2		= eeprom_read_byte(&abEEPROM[EEPROM_ADR_R2_LOCO_HB]);
		rSlot[1].stat		= eeprom_read_byte(&abEEPROM[EEPROM_R2_STAT]);		
	}
	

	
	
		initLocoNet(&RxBuffer) ;
		#if defined(UART)
			printf("Init Loconet \n");
		#endif	
		
		//Regler
		potAdcInit();	  
		
		//Keys
		initKeys();

		//Timer
		//initTimer();

		// switch LED 4
		PORTC |= (1 << LED4);

		//Timer
		
		TCNT0 = (byte) TICK_RELOAD ;
		sbi(TIFR, TOV0) ;
		sbi(TIMSK, TOIE0) ;
		TCCR0 = (TCCR0 & 0xF8) | TIMER_PRESCALER_CODE ;
		
		sei();

		//setUnconnectAll();
		rSlotReconnect();

	while (1)
	{	
		vProcessRxLoconetMessage();
		vProcessKey();
		vProcessRxLoconetMessage();
		
		//printf(" poti value: %x ", potiRead(1));
		//printf("\n");
		
		//printf("ADCV POTI 1 %u \n", potiRead(0));
		
		/*
		if(F0counter < 100) {
			F0counter++;
		}
		if(F0counter == 50)	{
			potiRead(0);
			potAdcTimerAction();
			vProcessRxLoconetMessage();
			vProcessPoti(0);
			vProcessRxLoconetMessage();		
		}else if(F0counter == 100) {
		potiRead(1);
		potAdcTimerAction();
		vProcessRxLoconetMessage();
		vProcessPoti(1);
		vProcessRxLoconetMessage();		
		F0counter = 0;	
		}
		*/
		
		potiRead(0);
		potAdcTimerAction(0);
		//vProcessRxLoconetMessage();
		vProcessPoti(0);
		//vProcessRxLoconetMessage();
	
		//printf("ADCV POTI 2 %u \n", potiRead(1));		
		
		potiRead(1);
		potAdcTimerAction(1);
		//vProcessRxLoconetMessage();
		vProcessPoti(1);
		//vProcessRxLoconetMessage();	

		//printf("geht durch");
		/*
		//printf("potAdcSpeedValue %x " , potAdcSpeedValue);
		//printf("  bSpdCnt %x  " , bSpdCnt);		
		//printf(" adc var %x  " , ADC);
		//printf("\n");		
		
		//vProcessRxLoconetMessage();
		//processTimerActions();
	*/
	}
	
/*
	int16_t counter = 0;
	//sendLocoNetMove(0,88);
	//sendLocoNet4BytePacketTry(0xA0,0x05,0x26,0x19);
	while (1)
	{
		vProcessRxLoconetMessage();
		vProcessKey();
		
		counter++;
		//printf("counter = %u", counter);
		//printf("\n");
		if(counter >= 250) {	
			//sendLocoNet4BytePacketTry(0xA0,0x08,0x13,0x44);
			counter = 0;
			//printf("counter > 200 \n");
			
			//sendLocoNetWriteSlotData(&rSlot);
			
		}
	}*/
} // end of main

void printRxLocoNetMessage(void) {
	for(int i = 0;i < 16; i++) {
		printf("%02hhx", RxPacket->data[i]);
	}
	printf("\n");
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
void vProcessRxLoconetMessage(void)
{
	
  RxPacket = recvLocoNetPacket() ;
  
  //for(int i = 0;i < 16; i++) 
	//printf("%02hhx", RxPacket->data[i]);
	//printf("\n");
	//printRxLocoNetMessage();
	//sendLocoNetWriteSlotData(&rSlot);
	
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
			#if defined(UART)
				printf("OPC_SL_RD_DATA : ACQUIRE_LOCO_GET: ");
				printRxLocoNetMessage();
			#endif
          if ((RxPacket->data[3] & LOCO_IDLE) == LOCO_IDLE)
          {
            vCopySlotFromRxPacket();
            vSetState(THR_STATE_ACQUIRE_LOCO_WRITE);
            sendLocoNetWriteSlotData(&rSlot[ch]);
          }
        }
        break;
      case THR_STATE_RECONNECT_GET_SLOT: // response of Get Slot By Adress
        {
			#if defined(UART)
				printf("OPC_SL_RD_DATA : RECONNECT_GET_SLOT CH : %u  :" ,ch);
				printRxLocoNetMessage();
			#endif
			
          if (  (rSlot[ch].adr  == RxPacket->data[4])
          && (rSlot[ch].adr2 == RxPacket->data[9]))
          { // slot not changed and in use , so we can use this slot further on
	          if ((RxPacket->data[3] & LOCO_IN_USE) == LOCO_IN_USE)
	          {
		          if (  (  (rSlot[ch].id1 == RxPacket->data[11]) && (rSlot[ch].id2 == RxPacket->data[12]))
		          || (  (0         == RxPacket->data[11]) && (0         == RxPacket->data[12])))
		          {
			          vCopySlotFromRxPacket();
			          vSetState(THR_STATE_RECONNECT_WRITE);
			          sendLocoNetWriteSlotData(&rSlot[ch]);
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
			#if defined(UART)
				printf("OPC_SL_RD_DATA : RECONNECT_NULL_MOVE: ");
				printRxLocoNetMessage();
			#endif
          if (  (rSlot[ch].adr  == RxPacket->data[4])
          && (rSlot[ch].adr2 == RxPacket->data[9]))
          { // slot not changed and in use , so we can use this slot further on
	          if ((RxPacket->data[3] & LOCO_IN_USE) == LOCO_IN_USE)
	          {
		          vCopySlotFromRxPacket();
		          vSetState(THR_STATE_RECONNECT_WRITE);
		          sendLocoNetWriteSlotData(&rSlot[ch]);
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
			if(ch == 0) {
				eeprom_write_byte(&abEEPROM[EEPROM_ADR_R1_LOCO_LB],  rSlot[0].adr);
			    eeprom_write_byte(&abEEPROM[EEPROM_ADR_R1_LOCO_HB],  rSlot[0].adr2);
			    eeprom_write_byte(&abEEPROM[EEPROM_R1_STAT], rSlot[0].stat);	
			}
			if( ch == 1) {
				eeprom_write_byte(&abEEPROM[EEPROM_ADR_R2_LOCO_LB],  rSlot[1].adr);
				eeprom_write_byte(&abEEPROM[EEPROM_ADR_R2_LOCO_HB],  rSlot[1].adr2);
				eeprom_write_byte(&abEEPROM[EEPROM_R2_STAT], rSlot[1].stat);	
			}

			
		//printf("adr im eeprom nah loco write%u \n", eeprom_read_byte(&abEEPROM[EEPROM_ADR_R1_LOCO_HB] ));
			
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
	   #if defined(UART)
		 printf("OPC_LOCO_SPD : ");
		 printRxLocoNetMessage();
		 printf(" \n stat : %u \n",RxPacket->data[1]);
	  #endif
      if (  (bThrState         == THR_STATE_CONNECTED)
         && (RxPacket->data[1] == rSlot[ch].slot         ))
      {
        int i;

        rSlot[ch].spd = RxPacket->data[2];

        GET_SPDCNT_BY_SLOTSPD

      }
      break;
/***************************************/
//  Set Slot Direction and F0 to F4
/***************************************/
    case OPC_LOCO_DIRF:
	#if defined(UART)
		printf("OPC_LOCO_DIRF : ");
		printRxLocoNetMessage();
	#endif
      if (  (bThrState         == THR_STATE_CONNECTED)
         && (RxPacket->data[1] == rSlot[0].slot         ))
      {

        if (  (bFrediVersion == FREDI_VERSION_INCREMENT_SWITCH)
              || (bFrediVersion == FREDI_VERSION_ANALOG          ))
        {
          // if switch is not in direction of direction flag, so change 
          // directionflag to synchronize the direction
          if ( (rSlot[0].dirf & 0x20) != (RxPacket->data[2] & 0x20))
          {
            rSlot[0].dirf &= 0x20;                       // get direction of fredi
            rSlot[0].dirf |= (RxPacket->data[2] & ~0x20); // and add F0..F4
            //sendLocoNetDirf(&rSlot);
          }
          else
          {
            rSlot[0].dirf = RxPacket->data[2];          // direction is equal, so take F0..F4
          }
        }
        else
        { // take direction as new one
          rSlot[0].dirf = RxPacket->data[2];
        }

        if (rSlot[0].dirf & 0x20)
        {
		  //PORT_A	|=	_BV(LED1);
        }
        else
        {

        }
      }
      break;
/***************************************/
//  Set Slot Sound Functions, 
/***************************************/
    case OPC_LOCO_SND:
	#if defined(UART)
		printf("OPC_LOCO_SND: ");
		printRxLocoNetMessage();
	#endif
      if (  (bThrState         == THR_STATE_CONNECTED)
         && (RxPacket->data[1] == rSlot[0].slot         ))
      {
        rSlot[0].snd = RxPacket->data[2];
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

void debounceCounterKeyPressed(uint8_t kID, uint8_t dC, uint8_t kP) {
	keyID				= kID;
	debounceCounter		= dC;
	keyPressed			= kP;
}

void vProcessKey(void)
{
	//printf("ProcessKey \n");
	//RICHT-G-1 = keyID = 1
		if (bit_is_clear(PINC,0) && keyPressed == 0) { //LED1, Richtungstaste1
			if(rSlot1State == 0) {
				keyID = 1;
				if(debounceCounter < debounceShort) {
					debounceCounter++;
				} else if (debounceCounter == debounceShort) {
					vSetState(THR_STATE_ACQUIRE_LOCO_GET);
					ch = 0;
					sendLocoNetMove(0, 0);				
					if(rSlot[0].adr != 0) {
						rSlot1State = 1;
						ch = 5;
						//printf("rslot1State \n");
					}
					
					debounceCounterKeyPressed(1,0,1);
				}
			} else {
				keyID = 1;
				if(debounceCounter < debounceMax) {
					debounceCounter++;
					//printf("LED1State = 1 ,  counter \n");
				} else if (debounceCounter == debounceMax) {
					if(lastLED1State == 0) {
						PORTA |= (1 << LED1);
						rSlot[0].dirf |= 0x20;
						sendLocoNetDirf(&rSlot[0]);
						#if defined(UART)
							printf("DIR R1 Rueckwaerts = %u \n", rSlot[0].dirf);
						#endif
						lastLED1State = 1;
					} else if (lastLED1State == 1) {
						PORTA &= ~(1 << LED1);
						rSlot[0].dirf &= 0x00;
						sendLocoNetDirf(&rSlot[0]);
						#if defined(UART)
							printf("DIR R1 Vorwaerts = %u \n", rSlot[0].dirf);
						#endif
						lastLED1State = 0;
					}
					debounceCounterKeyPressed(1,0,1);
				}
			}
		} else if(bit_is_set(PINC,0) && keyID == 1) {
			debounceCounterKeyPressed(0,0,0);
		}
		
		// RICHT-G-2 keyID = 2
		if (bit_is_clear(PINA,3) && keyPressed == 0) {
			if(rSlot2State == 0) {
				keyID = 2;
				if(debounceCounter < debounceShort) {
					debounceCounter++;
					} else if (debounceCounter == debounceShort) {
					vSetState(THR_STATE_ACQUIRE_LOCO_GET);
					ch = 1;
					sendLocoNetMove(0, 0);
					if(rSlot[1].adr != 0) {
						rSlot2State = 1;
						ch = 5;
					}
					
					debounceCounterKeyPressed(2,0,1);
				}
			} else {			
				keyID = 2;
				if(debounceCounter < debounceMax) {
					debounceCounter++;
					} else if (debounceCounter == debounceMax) {
					if(lastLED2State == 0) {
						PORTA |= (1 << LED2);
						rSlot[1].dirf |= 0x20;
						sendLocoNetDirf(&rSlot[1]);
						#if defined(UART)
							printf("DIR R2 Rueckwaerts = %u \n", rSlot[1].dirf);
						#endif					
						lastLED2State = 1;
					} else if (lastLED2State == 1) {
						PORTA &= ~(1 << LED2);
						rSlot[1].dirf &= 0x00;
						sendLocoNetDirf(&rSlot[1]);
						#if defined(UART)
							printf("DIR R2 Vorwaerts = %u \n", rSlot[1].dirf);
						#endif					
						lastLED2State = 0;
					}
					debounceCounterKeyPressed(2,0,1);
				}
			}
		} else if(bit_is_set(PINA,3) && keyID == 2) {
			debounceCounterKeyPressed(0,0,0);
		}

		// RICHT-G-3 keyID = 3
		if (bit_is_clear(PINC,2) && keyPressed == 0) { 
			keyID = 3;
			if(debounceCounter < debounceMax) {
				debounceCounter++;
				//printf("LED2State = 1 ,  counter \n");
			} else if (debounceCounter == debounceMax) {
				if(lastLED3State == 0) {
					PORTC |= (1 << LED3);
					lastLED3State = 1;
					} else if (lastLED3State == 1) {
					PORTC &= ~(1 << LED3);
					lastLED3State = 0;
				}
				debounceCounterKeyPressed(3,0,1);
			}
		} else if(bit_is_set(PINC,2) && keyID == 3) {
			debounceCounterKeyPressed(0,0,0);
		}

		// RICHT-G-4 keyID = 4
		if (bit_is_clear(PINC,1) && keyPressed == 0) {
			keyID = 4;
			if(debounceCounter < debounceMax) {
				debounceCounter++;
				//printf("LED4State = 1 ,  counter \n");
			} else if (debounceCounter == debounceMax) {
				if(lastLED4State == 0) {
					PORTC |= (1 << LED4);
					lastLED4State = 1;
					} else if (lastLED4State == 1) {
					PORTC &= ~(1 << LED4);
					lastLED4State = 0;
				}
				debounceCounterKeyPressed(4,0,1);
			}
		} else if(bit_is_set(PINC,1) && keyID == 4) {
			debounceCounterKeyPressed(0,0,0);
		}		

	//Funktionstasten-L1 , keyID = 5
	
	if (bit_is_clear(PINC,7) && keyPressed == 0 && (ReglerKeysActive != 1)) { 
		keyID = 5;
		if(debounceCounter < debounceMax) {
				debounceCounter++;
		} else if (debounceCounter == debounceMax) {
				ReglerKeysActive = 1;
				debounceCounterKeyPressed(5,0,1);
				
				#if defined(UART)
					printf("ReglerKeyActive 1 \n");
				#endif
		}
	} else if(bit_is_set(PINC,7) && keyID == 5) {
			debounceCounterKeyPressed(0,0,0);
	}
	
	// Funktionstaste-L2 , keyID = 6
	if (bit_is_clear(PINB,0) && keyPressed == 0 && (ReglerKeysActive != 2)) { //LED1, Richtungstaste1
		keyID = 6;
		if(debounceCounter < debounceMax) {
				debounceCounter++;
			} else if (debounceCounter == debounceMax) {
				ReglerKeysActive = 2;
				debounceCounterKeyPressed(6,0,1);
				#if defined(UART)
					printf("ReglerKeyActive 2 \n");
				#endif
		}
	} else if(bit_is_set(PINB,0) && keyID == 6) {
		debounceCounterKeyPressed(0,0,0);
	}
	
	// Funktionstaste-L3 , keyID = 7
	if (bit_is_clear(PINC,5) && keyPressed == 0 && (ReglerKeysActive != 3)) { 
		keyID = 7;
		if(debounceCounter < debounceMax) {
				debounceCounter++;
			} else if (debounceCounter == debounceMax) {
				ReglerKeysActive = 3;
				debounceCounterKeyPressed(7,0,1);
				#if defined(UART)
					printf("ReglerKeyActive 3 \n");
				#endif
		}
	} else if(bit_is_set(PINC,5) && keyID == 7) {
		debounceCounterKeyPressed(0,0,0);
	}	
	
	// Funktionstaste-L4 , keyID = 8
	if (bit_is_clear(PINA,2) && keyPressed == 0 && (ReglerKeysActive != 4)) {
		keyID = 8;
		if(debounceCounter < debounceMax) {
				debounceCounter++;
			} else if (debounceCounter == debounceMax) {
				ReglerKeysActive = 4;
				debounceCounterKeyPressed(8,0,1);
				#if defined(UART)
					printf("ReglerKeyActive 4 \n");
				#endif
				//printf("ThrState %u ", bThrState);
				//printf("\n");
				//printf("adr2 %u \n", eeprom_read_byte(&abEEPROM[EEPROM_ADR_LOCO_LB]));
				//printf("\n");
				//printf("slot %u \n", rSlot.slot);
				//printf("\n");				
		}
	} else if(bit_is_set(PINA,2) && keyID == 8) {;
		debounceCounterKeyPressed(0,0,0);
	}

	//Funktionstasten

	// Funktionstaste-1 , keyID = 9
	if (bit_is_clear(PIND,3) && keyPressed == 0 ) {
		keyID = 9;
		if(debounceCounter < debounceMax) {
			debounceCounter++;
		} else if (debounceCounter == debounceMax) {
			switch (ReglerKeysActive)
			{
				case 0:
				break;
				case 1:

				#if defined(UART)
					printf(" adr = %u adr2 = %u ", rSlot[0].adr ,rSlot[0].adr2);
					printf(" id = %u , id2 = %u ", rSlot[0].id1 , rSlot[0].id2);
					printf("adr im EEPROM LOCO_HB  %u ", eeprom_read_byte(&abEEPROM[EEPROM_ADR_R1_LOCO_LB] ));
					printf(" stat = %u \n", rSlot[0].stat);
				#endif
				vSetUnconnected();
				break;
				case 2:
				rSlot[0].spd = 50;
				#if defined(UART)
					printf("spd = %u \n", rSlot[0].spd);
					printf(" adr = %u \n", rSlot[0].adr);
				#endif
				sendLocoNetSpd(&rSlot[0]);
				break;
			}				
			debounceCounterKeyPressed(9,0,1);
		}
	} else if(bit_is_set(PIND,3) && keyID == 9) {
		debounceCounterKeyPressed(0,0,0);
	}
/*

		//Funktionstaste-1
		if( bit_is_clear(PIND, 3) ) { //Funktionstaste-1
			switch (ReglerKeysActive)
			{
				case 0:
				break;
				case 1:
				
				vSetState(THR_STATE_ACQUIRE_LOCO_GET);
				printf(" id = % ", rSlot.id1);
				printf("\n");
				sendLocoNetMove(0, 0);
				printf(" id = % ", rSlot.id1);
				printf("\n");				
				PORTA |= (1 << LED1); // ROT
				break;
				case 2:
				PORTA &= ~(1 << LED2); // GRÜN
				break;
			}
			//Loconet
		}
*/	
/*
		//Funktionstaste-2
		if( bit_is_clear(PIND, 4) ) { //Funktionstaste-2
			switch (ReglerKeysActive)
			{
				case 0:
				break;
				case 1:
				PORTA &= ~(1 << LED1); // GRÜN
				break;
				case 2:
				PORTA |= (1 << LED2); // ROT
				break;
			}
			//Loconet
		}	
	
		//Funktionstaste-3
		if( bit_is_clear(PIND, 5) ) { //Funktionstaste-3
			switch (ReglerKeysActive)
			{
				case 0:
				break;
				case 1:
				PORTA |= (1 << LED3); // ROT
				break;
				case 2:
				PORTA &= ~(1 << LED3); // GRÜN
				break;
			}
			//Loconet
		}

		//Funktionstaste-4
		if( bit_is_clear(PIND, 6) ) { //Funktionstaste-4
			switch (ReglerKeysActive)
			{
				case 0:
				break;
				case 1:
				PORTA &= ~(1 << LED4); // GRÜN
				break;
				case 2:
				PORTA |= (1 << LED4); // ROT
				break;
			}
				    //Loconet
		}
		*/
		/*
		if(rSlot.adr != 0) {
			F0counter = 20;
			while(F0counter > 0) {
				PORTA &= ~(1 << LED1); // GRÜN
				_delay_ms(1000);
				PORTA |= (1 << LED1); // ROT
				_delay_ms(1000);
				F0counter--;
			}
			
		}*/

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
void vProcessPoti(uint8_t ch)
{	/*
	#if defined(UART)
		printf("ch = 0 : fospd %u - fsspd %u", fOldSetSpeed[0] ,fSetSpeed[0]);
	#endif
  /*if (bThrState == THR_STATE_CONNECTED)
  {*//*
	  #if defined(UART) 
		printf(" oldspd: %u ", fOldSetSpeed);  
	  #endif
	  */
    fOldSetSpeed[ch] = fSetSpeed[ch];
	/*
	  #if defined(UART) 
		  printf(" after oldspd: %u ", fOldSetSpeed);
	  #endif
	  */
	//Wenn Richtung gewechselt, pot in richtige richtung bringen um die Geschwindigkeit zu ändern.
    if (!fSetSpeed[ch])
    {
      if (potAdcSpeedValue[ch] == 0)     // potivalue is in right range for  set speed again
      {
        fSetSpeed[ch] = TRUE;
      }
    }

    if (fSetSpeed[ch])
    {
      bSpd[ch] = potAdcSpeedValue[ch];
	  /*
	  	  #if defined(UART)
	  	  printf(" bSpD: %u \n", bSpd[ch]);
	  	  #endif
			*/
      if (rSlot[ch].spd != bSpd[ch])
      {
		  
        rSlot[ch].spd = bSpd[ch];
		#if defined(UART)
			printf(" channel %u spd : %u \n", ch, bSpd[ch]);
		#endif
        sendLocoNetSpd(&rSlot[ch]);
		
      }
    }

   /* if (fOldSetSpeed != fSetSpeed)
    {
      vSetState(THR_STATE_CONNECTED); // reset of blinking LEDs
    }*/
  }
  /*else if (bThrState >= THR_STATE_SELFTEST)
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
}/*


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
	//printf("reconnect write \n");
      vSetState(THR_STATE_RECONNECT_GET_SLOT);
      break;
    case THR_STATE_UNCONNECTED_WRITE:
      vSetState(THR_STATE_CONNECTED);
      break;
    }
  }
  else
  {
    //resetTimerAction(&MessageTimer, RESPONSE_TIME);
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
		//printf("state reconnect null move \n");
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

if (sendLocoNet4BytePacket(OPC_LOCO_ADR, rSlot[ch].adr2, rSlot[ch].adr) != LN_DONE)
{
	
}
/*
if (sendLocoNet4BytePacket(OPC_LOCO_ADR, rSlot[0].adr2, rSlot[0].adr) != LN_DONE)
{
	#if defined(UART) 
		 printf("LN_DONE");
	#endif

}
  else
  {

  }
  */
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
    rSlot[ch].stat    = RxPacket->data[ 3];                 // slot status
    rSlot[ch].adr     = RxPacket->data[ 4];                 // loco address
    rSlot[ch].adr2    = RxPacket->data[ 9];                 // loco address high
	#if defined(UART)
		printf("rslot bei copy:");
		printf("%02hhx",rSlot[0].adr2);
		printf("\n");
	#endif
  }
  else
  {
    rSlot[ch].stat    |= RxPacket->data[ 3] & ~DEC_MODE_MASK; // slot status
  }

  rSlot[ch].slot      = RxPacket->data[ 2];                 // slot number for this request

  if (  (bFrediVersion == FREDI_VERSION_INCREMENT_SWITCH)      
        || (bFrediVersion == FREDI_VERSION_ANALOG          ))
  {
    rSlot[ch].dirf = RxPacket->data[ 6] & ~0x20;            // get direction by switch position                          

    if (bCurrentKey & Key_Dir_1)
    {
      rSlot[ch].dirf |=  0x20;                                    
    }

    if ((rSlot[ch].dirf & 0x20) != (RxPacket->data[ 6] & 0x20)) // compare Fredi direction with slot direction
    {
      rSlot[ch].spd = 1;                                    // direction isn't matching so stop train 
      fSetSpeed[ch] = FALSE;                                // and show blinking
    }
    else if (bFrediVersion == FREDI_VERSION_ANALOG)
    {
      rSlot[ch].spd = potAdcSpeedValue;                     // direction is matching, get speed by poti
    }
    else
    {
      rSlot[ch].spd = RxPacket->data[ 5];                   // a increment-switch-Fredi takes speed from slot
    }                                                         
  }
  else
  {
    rSlot[ch].spd   = RxPacket->data[ 5];                   // command speed
    rSlot[ch].dirf  = RxPacket->data[ 6];                   // direction and function keys
  }                                                           

  rSlot[ch].trk       = RxPacket->data[ 7];                 // track status
  rSlot[ch].ss2       = RxPacket->data[ 8];                 // slot status 2 (tells how to use ID1/ID2 & ADV Consist
  rSlot[ch].snd       = RxPacket->data[10];                 // Sound 1-4 / F5-F8

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
	
  rSlot[ch].adr   = 0;
  rSlot[ch].adr2  = 0;
  if(ch == 0) {
	   eeprom_write_byte(&abEEPROM[EEPROM_ADR_R1_LOCO_LB],  rSlot[0].adr);
	   eeprom_write_byte(&abEEPROM[EEPROM_ADR_R1_LOCO_HB],  rSlot[0].adr2);
	   eeprom_write_byte(&abEEPROM[EEPROM_R1_STAT], EEPROM_DECODER_TYPE_DEFAULT); 
  }
  if(ch == 1) {
	   eeprom_write_byte(&abEEPROM[EEPROM_ADR_R2_LOCO_LB],  rSlot[0].adr);
	   eeprom_write_byte(&abEEPROM[EEPROM_ADR_R2_LOCO_HB],  rSlot[0].adr2);
	   eeprom_write_byte(&abEEPROM[EEPROM_R2_STAT], EEPROM_DECODER_TYPE_DEFAULT);	  
  }
  /*
  eeprom_write_byte(&abEEPROM[EEPROM_ADR_R1_LOCO_LB],  rSlot[0].adr);
  eeprom_write_byte(&abEEPROM[EEPROM_ADR_R1_LOCO_HB],  rSlot[0].adr2);
  eeprom_write_byte(&abEEPROM[EEPROM_R1_STAT], EEPROM_DECODER_TYPE_DEFAULT);
	*/
  vSetState(THR_STATE_UNCONNECTED);
}
