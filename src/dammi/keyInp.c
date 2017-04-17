/**
 *  $Id: keyInp.c,v 1.1 2013/11/18 22:38:47 pischky Exp $
 *
 *  Copyright (C) 2013 by Martin Pischky (mailto:martin@pischky.de)
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// includes --------------------------------------------------------------------

#include <stdint.h>         // typedef uint8_t, uint16_t
#include <stddef.h>         // #define NULL
#include <avr/io.h>         // #define PORTB, #define DDRB, #define PINB,
                            // #define PORTC, #define DDRC, #define PINC,
                            // #define PORTD, #define DDRD, #define PIND,
                            // #define SREG
#include <avr/interrupt.h>  // cli(),
#include "keyInp.h"         // KeyEvent, onKeyDown, onKeyUp, keyInpInit()
                            // keyInpTimerAction(), keyInpPoll()

// port and i/o usage ----------------------------------------------------------

#define KPIN_DIR0		PA3
#define KPIN_F0			PA2


#define KPIN_PORT1		PORTB
#define KPIN_DDR1		DDRB
#define KPIN_PIN1		PINB

#define KPIN_F1			PB4
#define KPIN_F2			PB0

#define KPIN_PORT2		PORTC
#define KPIN_DDR2		DDRC
#define KPIN_PIN2		PINC


#define KPIN_F3			PC7
#define KPIN_DIR1		PC2
#define KPIN_DIR2		PC1
#define KPIN_DIR3		PC0

#define KPIN_PORT3		PORTD
#define KPIN_DDR3		DDRD
#define KPIN_PIN3		PIND

#define KPIN_F4			PD6
#define KPIN_F5			PD5
#define KPIN_F6			PD4
#define KPIN_F7			PD3


// extern variables ------------------------------------------------------------

KeyEvent onKeyDown = (KeyEvent) NULL;
KeyEvent onKeyUp = (KeyEvent) NULL;

// static variables ------------------------------------------------------------

/**
 * Bit array of current key states. A '1' means that the key is pressed.
 */
static volatile uint16_t lastKeys = 0;

/**
 * Bit array of key states that needs processing by keyInpPoll(). A '1' means
 * that processing is required.
 */
static volatile uint16_t needsProcessing = 0;

/**
 * Bit array of current modifier states. A '1' means that modifiers key is
 * pressed.
 */
static volatile uint8_t  lastModifiers = 0;

// functions -------------------------------------------------------------------

void keyInpInit(void)
{
  // set data direction register for keys as inputs
  KPIN_DDR1  &= ~( (1<<KPIN_F0) | (1<<KPIN_F1)
                   | (1<<KPIN_F2) | (1<<KPIN_F3)
                   | (1<<KPIN_F4) );
  // enable the pull-ups
  KPIN_PORT1 |=  ( (1<<KPIN_F0) | (1<<KPIN_F1)
                   | (1<<KPIN_F2) | (1<<KPIN_F3)
                   | (1<<KPIN_F4) );


}

void keyInpTimerAction(void) {
  uint16_t keys = 0;
  if (!(KPIN_PIN1 & (1 << KPIN_F0)))      keys |= (1 << (KEY_F0 - 1));
  if (!(KPIN_PIN1 & (1 << KPIN_F1)))      keys |= (1 << (KEY_F1 - 1));
  if (!(KPIN_PIN1 & (1 << KPIN_F2)))      keys |= (1 << (KEY_F2 - 1));
  if (!(KPIN_PIN1 & (1 << KPIN_F3)))      keys |= (1 << (KEY_F3 - 1));
  if (!(KPIN_PIN1 & (1 << KPIN_F4)))      keys |= (1 << (KEY_F4 - 1));
  if (!(KPIN_PIN1 & (1 << KPIN_F5)))      keys |= (1 << (KEY_F5 - 1));
  if (!(KPIN_PIN1 & (1 << KPIN_F6)))      keys |= (1 << (KEY_F6 - 1));
  if (!(KPIN_PIN1 & (1 << KPIN_F7)))      keys |= (1 << (KEY_F7 - 1));
  if (!(KPIN_PIN1 & (1 << KEY_DIR0)))     keys |= (1 << (KEY_DIR0 - 1));
  if (!(KPIN_PIN1 & (1 << KEY_DIR1)))     keys |= (1 << (KEY_DIR1 - 1));
  if (!(KPIN_PIN1 & (1 << KEY_DIR2)))	  keys |= (1 << (KEY_DIR2 - 1));
  if (!(KPIN_PIN1 & (1 << KEY_DIR3)))     keys |= (1 << (KEY_DIR3 - 1));



//Anpassung benötigt, da kein Shift Key vorhanden
  uint8_t modifiers = 0;
  if (keys) modifiers;
  if (keys) modifiers;
  if (modifiers) { // F0 is only modifier when used with S0 or S1
    if (keys) modifiers;
  }

  // inform keyInpPoll() about changes
  // FIXME: this assumes that events are consumed fast
  needsProcessing = lastKeys ^ keys;
  lastKeys = keys;
  lastModifiers = modifiers;
}

void keyInpPoll(void) {
  if (needsProcessing) {
    uint8_t sreg = SREG;
    cli();
    uint16_t np = needsProcessing;
    uint16_t keys = lastKeys;
    uint8_t modifiers = lastModifiers;
    needsProcessing = 0;
    SREG = sreg;
    uint16_t curBit = (1 << 0);
    for (KeyCode curKey = KEY_NONE+1; curKey < KEY_LAST; curKey++) {
      if (np & curBit) {
        if (keys & curBit) {
          if (onKeyDown != (KeyEvent) NULL) {
            onKeyDown(curKey | (modifiers << 8));
          }
        } else {
          if (onKeyUp != (KeyEvent) NULL) {
            onKeyUp(curKey | (modifiers << 8));
          }
        }
      }
      curBit <<= 1;
    }
  }
}
