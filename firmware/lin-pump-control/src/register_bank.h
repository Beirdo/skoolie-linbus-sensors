#ifndef __register_bank_h_
#define __register_bank_h_

#include <Arduino.h>


typedef enum {
  REG_BOARD_TYPE = 0,
  REG_LOCATION,
  REG_PUMP_CONTROL,
  REG_PUMP_CURRENT_HI,
  REG_PUMP_CURRENT_LO,
  MAX_REGISTERS,
} register_pump_controller_t;


#endif
