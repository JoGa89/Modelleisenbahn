/*
 * potAdc.h
 *
 *  Created on: 29.07.2011
 *      Author: martin
 */
// $Id: potAdc.h,v 1.1 2011/07/31 15:54:12 pischky Exp $

#ifndef POTADC_H_
#define POTADC_H_

#include <stdint.h>         // typedef uint8_t, uint16_t

/**
 * The current position of the pot as DCC speed value (0,2..126).
 * Note that ESTOP=1 is skipped.
 * This value is already "filtered".
 * It is updated by calls to potAdcTimerAction().
 */
extern volatile uint8_t potAdcSpeedValue;

/**
 * The current position of the pot as raw value (0..1023).
 */
extern volatile uint16_t potAdcRawValue;


/**
 * Turn the ADC circuit off to save power.
 */
void potAdcPowerOff(void);

/**
 * Initialization of the ADC.
 * Required by the following operations.
 */
void potAdcInit(void);

/**
 * Should be called periodically to update potAdcSpeedValue
 * and potAdcRawValue.
 * Installed as TimerAction in main and called every 10ms.
 */
void potAdcTimerAction(void);

#endif /* POTADC_H_ */
