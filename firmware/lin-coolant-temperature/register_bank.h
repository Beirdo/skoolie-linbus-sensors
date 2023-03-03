#ifndef __register_bank_h_
#define __register_bank_h_

#include <Arduino.h>


typedef enum {
  REG_BOARD_TYPE = 0,
  REG_LOCATION,
  REG_COOLANT_TEMP0_HI,
  REG_COOLANT_TEMP0_LO,
  REG_COOLANT_TEMP1_HI,
  REG_COOLANT_TEMP1_LO,
  MAX_REGISTERS,
} register_coolant_temperature_t;


#endif
