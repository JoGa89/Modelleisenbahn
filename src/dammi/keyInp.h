/**
 *  $Id: keyInp.h,v 1.1 2013/11/18 22:38:47 pischky Exp $
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

#ifndef _KEYINP_H_
#define _KEYINP_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Enumeration used as event parameter by onKeyDown and onKeyUp. Should
 * fit into a 16 bit unsigned integer. Low byte is key number. High byte
 * contains modifiers pressed while event is raised. Used as flags.
 * E.g.: onKeyDown(KEY_F2 | KEY_MODIFIER_1 | KEY_MODIFIER_2) means
 * that F2-key has been pressed while SHIFT1 and SHIFT2 have been down.
 */
typedef enum {
    KEY_NONE			= 0,
    KEY_F0				= 1,
    KEY_F1				= 2,
    KEY_F2				= 3,
    KEY_F3				= 4,
    KEY_F4				= 5,
    KEY_F5				= 6,
    KEY_F6				= 7,
    KEY_F7				= 8,
	KEY_DIR0			= 9,
	KEY_DIR1			= 10,
	KEY_DIR2			= 11,
	KEY_DIR3			= 12,
	KEY_SHIFT_KEY_F0	= 13,				// Von keyInp.c benötigt, muss umgeschrieben werden, da bei uns kein Shift Key vorhanden
	KEY_LAST          = KEY_SHIFT_KEY_F0,	//
} KeyCode;

/**
 * Function pointer type used by onKeyDown/Up events.
 * @param
 */
typedef void (*KeyEvent) (KeyCode);

/**
 * Install your event handling hooks here.
 */
extern KeyEvent onKeyDown;
extern KeyEvent onKeyUp;

/**
 * Initialization of the ADC.
 * Required by the following operations.
 */
void keyInpInit(void);

/**
 * Should be called periodically.
 * Install as TimerAction in main and called every XXms.
 */
void keyInpTimerAction(void);

/**
 * Fires key events detected by keyInpTimerAction by calling
 * onKeyDown and onKeyUp. Call this in your main loop.
 */
void keyInpPoll(void);

#ifdef __cplusplus
}
#endif

#endif /* _KEYINP_H_ */
