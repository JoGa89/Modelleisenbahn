/*
 * potAdc.c
 *
 *  Created on: 29.07.2011
 *      Author: martin
 */
// $Id: potAdc.c,v 1.1 2011/07/31 15:54:12 pischky Exp $

#include <stdint.h>         // typedef uint8_t, uint16_t
#include <stdlib.h>         // abs()
#include <avr/io.h>         // ADCH, ADCSRA, ADSC, ADMUX, ADLAR, REFS0, ADEN
#include <stdint.h>         // typedef uint8_t, uint16_t
#include "sysdef.h"         // #define F_CPU
#include "potadc.h"         // potAdcSpeedValue, potAdcRawValue
                            // potAdcPowerOff(), potAdcInit(),
                            // potAdcTimerAction()

// extern variables ------------------------------------------------------------

volatile uint8_t potAdcSpeedValue = 0;
volatile uint16_t potAdcRawValue = 0;

// defines ---------------------------------------------------------------------

#define ADC_CLOCK_SETVAL 100000 // should be 50kHz to 200kHz to get maximal
                                // resolution

#if F_CPU / ADC_CLOCK_SETVAL >= 128
  #define ADPS ((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0))
#elif F_CPU / ADC_CLOCK_SETVAL >= 64
  #define ADPS ((1<<ADPS2) | (1<<ADPS1) | (0<<ADPS0))
#elif F_CPU / ADC_CLOCK_SETVAL >= 32
  #define ADPS ((1<<ADPS2) | (0<<ADPS1) | (1<<ADPS0))
#elif F_CPU / ADC_CLOCK_SETVAL >= 16
  #define ADPS ((1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0))
#elif F_CPU / ADC_CLOCK_SETVAL >= 8
  #define ADPS ((0<<ADPS2) | (1<<ADPS1) | (1<<ADPS0))
#elif F_CPU / ADC_CLOCK_SETVAL >= 4
  #define ADPS ((0<<ADPS2) | (1<<ADPS1) | (0<<ADPS0))
#else // F_CPU / ADC_CLOCK_SETVAL >= 2
  #define ADPS ((0<<ADPS2) | (0<<ADPS1) | (1<<ADPS0))
#endif

#define DIVISION_FACTOR (1<<(ADPS))         // 64 on FREDI with 7.3728MHz
#define ADC_CLOCK (F_CPU / DIVISION_FACTOR) // 115200 on FREDI => a conversation
                                            // takes about 113µs

#if ADC_CLOCK > 200000
  #error "adc clock frequency to high"
#endif

#if ADC_CLOCK < 50000
  #error "adc clock frequency to low"
#endif


// static functions ------------------------------------------------------------

/**
 * filter by delta of measurements
 * @param adcValue
 * @return
 */
static inline uint16_t deltaFilter( uint16_t adcVal ) {

  #define FILTER_DELTA 3    // testing shows that 1 should be ok
                            // 0: disable filter (for testing)

  static uint16_t deltaFilterState = 0;

  int16_t delta = abs( (int16_t)deltaFilterState - (int16_t)adcVal);
  if( delta >= FILTER_DELTA ) {
    deltaFilterState = adcVal;
  }
  return deltaFilterState;
}

/**
 * Map pot values 0..1023 to dcc/loconet speed 0,2..126
 * @param speed
 * @return
 */
static inline uint8_t mapSpeedVal( uint16_t adcValue ) {
  uint8_t speedVal = (uint8_t) (adcValue >> 3);
  if( speedVal > 0 ) {
    speedVal++; //skip E_STOP
  }
  if( speedVal > 126 ) {
    speedVal = 126;
  }
  return speedVal;
}

static inline uint8_t timeFilter( uint8_t speedVal ) {

  #define FILTER_TIME 10 // (FILTER_TIME * KEY_POLL_TIME
                         // * 2^NB_SAMPLES_LOG2) is the minimal speed update
                         // time. potAdcTimerAction() is called every
                         // KEY_POLL_TIME ms. (-> sysdef.h)

  static uint8_t timeFilterState = 0;
  static uint8_t count = 0;
  if( count > 0 ) {
    count--;
  }
  if( count == 0 && speedVal != timeFilterState ) {
    timeFilterState = speedVal;
    count = FILTER_TIME;
  }
  return timeFilterState;
}

// functions -------------------------------------------------------------------

void potAdcPowerOff(void) {
  ADCSRA = (0<<ADEN); // disable ADC
  #if defined(__AVR_ATmega48__)  | defined(__AVR_ATmega48A__)  \
    | defined(__AVR_ATmega48P__) | defined(__AVR_ATmega88__)   \
    | defined(__AVR_ATmega88A__) | defined(__AVR_ATmega88P__)  \
    | defined(__AVR_ATmega168__) | defined(__AVR_ATmega168A__) \
    | defined(__AVR_ATmega168P__)| defined(__AVR_ATmega328__)  \
    | defined(__AVR_ATmega328P__)
    PRR   |= (1<<PRADC);  // shut down ADC.
  #endif
}

void potAdcInit(void) {
  #if defined(__AVR_ATmega48__)  | defined(__AVR_ATmega48A__)  \
    | defined(__AVR_ATmega48P__) | defined(__AVR_ATmega88__)   \
    | defined(__AVR_ATmega88A__) | defined(__AVR_ATmega88P__)  \
    | defined(__AVR_ATmega168__) | defined(__AVR_ATmega168A__) \
    | defined(__AVR_ATmega168P__)| defined(__AVR_ATmega328__)  \
    | defined(__AVR_ATmega328P__)
    PRR   &= ~(1<<PRADC); // enable ADC in Power Reduction Register.
    DIDR0 |= (1<<ADC0D);  // disable digital input buffer to reduce
                          // power consumption
  #endif

  ADMUX  = (0<<REFS1) | (1<<REFS0) // AVCC pin as reference
         | (0<<ADLAR)              // 0 = left adjust, 1 = right adjust
         | (0<<MUX3)  | (0<<MUX2)  | (0<<MUX1)  | (0<<MUX0); // select ADC0 pin

  #if defined(__AVR_ATmega48__)  | defined(__AVR_ATmega48A__)  \
    | defined(__AVR_ATmega48P__) | defined(__AVR_ATmega88__)   \
    | defined(__AVR_ATmega88A__) | defined(__AVR_ATmega88P__)  \
    | defined(__AVR_ATmega168__) | defined(__AVR_ATmega168A__) \
    | defined(__AVR_ATmega168P__)| defined(__AVR_ATmega328__)  \
    | defined(__AVR_ATmega328P__) | defined(__AVR_ATmega16__)
    ADCSRA = (1<<ADEN)  // Enable ADC
           | (1<<ADSC)  // Start single conversion
           | (0<<ADATE) // disable auto trigger
           | (0<<ADIF)  // do not modify interrupt flag
           | (0<<ADIE)  // disable interrupt
           | ADPS;      // ADC precaler selection
    // ADATE is 0 here, so we do not care about ADTS[2:0] in ADCSRB
  #elif defined(__AVR_ATmega8__)
    ADCSRA = (1<<ADEN)  // Enable ADC
           | (1<<ADSC)  // Start single conversion
           | (0<<ADFR)  // deselect ADC Free Running Mode
           | (0<<ADIF)  // do not modify interrupt flag
           | (0<<ADIE)  // disable interrupt
           | ADPS;      // ADC precaler selection
  #else
    #error "unknown mcu"
  #endif

  while (bit_is_set(ADCSRA, ADSC)); // wait for measurement
  ADCH;                             // throw away first analog value
  potAdcSpeedValue = 0;             // ignore first analog value
  ADCSRA      |= _BV(ADSC);         // start new measurement
}

void potAdcTimerAction(void) {

  #define NB_SAMPLES_LOG2 0 // log to basis 2 of number of samples
                            // 3: take arithmetic mean of 8 samples
                            // 2: four samples
                            // 0: single sample

  if (bit_is_clear(ADCSRA, ADSC)) // measurement done
  {
    potAdcRawValue = ADCW;
    ADCSRA |= _BV(ADSC);          // start new measurement

    #if NB_SAMPLES_LOG2 == 0

      potAdcSpeedValue
                = timeFilter( mapSpeedVal( deltaFilter( potAdcRawValue ) ) );

    #elif NB_SAMPLES_LOG2 > 0

      static uint8_t sampleCnt = 0;
      static uint16_t meanVal = 0;

      meanVal += potAdcRawValue;
      sampleCnt++;

      if (sampleCnt >= (1<<NB_SAMPLES_LOG2))
      {
        uint16_t measurement = meanVal >> NB_SAMPLES_LOG2;
        potAdcSpeedValue
                  = timeFilter( mapSpeedVal( deltaFilter( measurement ) ) );
        meanVal = 0;
        sampleCnt = 0;
      }

    #endif
  }
}

