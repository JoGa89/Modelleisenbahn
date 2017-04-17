/*
 * dammi.c
 *
 * Created: 17.04.2017 01:32:59
 *  Author: Tobias Rettemeyer
 */ 

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "sysdef.h"
#include "common_defs.h"
#include "ln_sw_uart.h"
#include "ln_interface.h"


#include "systimer.h"
#include "keyInp.h"

/******************************************************************************/
// loconet
/******************************************************************************/
//aus Fredi.c übernommen, unvollständig
LnBuf RxBuffer;

lnMsg *RxPacket;

//LN_STATUS RxStatus ;
//word      RxMsgCount ;
lnMsg TxPacket;

rwSlotDataMsg rSlot;




  