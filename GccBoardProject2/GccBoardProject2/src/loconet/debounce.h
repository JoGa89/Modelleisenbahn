/*
 * This debounce routine comes from Scott Dattalo <scott@dattalo.com> 
 * http://www.dattalo.com/technical/software/pic/debounce.html
 *
 * Modified by Alex Shepherd to pass in state as a parameter
 *
 */

#include "common_defs.h"

typedef struct
{
  byte clock_A;
  byte clock_B;
  byte debounced_state;
} DEBOUNCE_STATE ;

byte debounce( DEBOUNCE_STATE *state, byte new_sample ) ;
