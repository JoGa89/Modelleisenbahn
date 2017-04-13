#ifndef _LN_INTERFACE_H_
#define _LN_INTERFACE_H_



#include "sysdef.h"       // #define LOCONET_MASTER, F_CPU
#include "common_defs.h"
#include "ln_buf.h"

/*
** TIMER1 is used for the bit period timings. It must be setup so that
** its prescaler is set to 1 so that the timer runs at the clock frequency.
** The bit period is then the clock frequency / LocoNet Baud rate
*/

#define LN_BIT_PERIOD       (F_CPU / 16666)

typedef enum
{
	LN_CD_BACKOFF = 0,
	LN_PRIO_BACKOFF,
	LN_NETWORK_BUSY,
	LN_DONE,
	LN_COLLISION,
	LN_UNKNOWN_ERROR,
	LN_RETRY_ERROR
} LN_STATUS ;

// CD Backoff starts after the Stop Bit (Bit 9) and has a minimum or 20 Bit Times
// but initially starts with an additional 20 Bit Times 
#define   LN_CARRIER_TICKS      20  // carrier detect backoff - all devices have to wait this
#define   LN_MASTER_DELAY        6  // non master devices have to wait this additionally
#define   LN_INITIAL_PRIO_DELAY 20  // initial attempt adds priority delay
#define   LN_BACKOFF_MIN      (LN_CARRIER_TICKS + LN_MASTER_DELAY)      // not going below this
#define   LN_BACKOFF_INITIAL  (LN_BACKOFF_MIN + LN_INITIAL_PRIO_DELAY)  // for the first normal tx attempt
#define   LN_BACKOFF_MAX      (LN_BACKOFF_INITIAL + 10)                 // lower priority is not supported


// Interface to lower layer
// Must be implemented by hardware level module, ln_sw_uart.c or ln_hw_uart.c or...
void initLocoNetHardware( LnBuf *RxBuffer );
LN_STATUS sendLocoNetPacketTry(lnMsg *TxData, unsigned char ucPrioDelay);

// Interface to application
// Implemented in ln_interface.c
void initLocoNet( LnBuf *RxBuffer ) ;
lnMsg * recvLocoNetPacket( void ) ;
LN_STATUS sendLocoNetPacket( lnMsg *TxPacket ) ;
LN_STATUS sendLocoNet4BytePacket( byte OpCode, byte Data1, byte Data2 ) ;
LN_STATUS sendLocoNet4BytePacketTry( byte OpCode, byte Data1, byte Data2, byte PrioDelay ) ;


// Additional functionallity to implement the tight timing requirements for a master
#ifdef LOCONET_MASTER

	void performLocoNetBusy(char bNewState);
	char sendingLocoNetBusy(void);

#endif



#endif
