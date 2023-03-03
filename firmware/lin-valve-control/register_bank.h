#ifndef __register_bank_h_
#define __register_bank_h_

#include <Arduino.h>


typedef enum {
  REG_BOARD_TYPE = 0,
  REG_LOCATION,
  REG_VALVE_CONTROL,
  REG_VALVE_STATUS,
  REG_VALVE_CURRENT_HI,
  REG_VALVE_CURRENT_LO,
  MAX_REGISTERS,
} register_valve_controller_t;


#endif
