#ifndef __register_bank_h_
#define __register_bank_h_

#include <Arduino.h>

typedef enum {
  REG_BOARD_TYPE = 0,
  REG_LOCATION,
  REG_PELTIER_CONTROL,
  REG_PELTIER_CURRENT_HI,
  REG_PELTIER_CURRENT_LO,
  MAX_REGISTERS,
} register_peltier_controller_t;

#endif
