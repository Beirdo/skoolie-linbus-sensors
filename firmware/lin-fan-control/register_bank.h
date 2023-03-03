#ifndef __register_bank_h_
#define __register_bank_h_

#include <Arduino.h>


typedef enum {
  REG_BOARD_TYPE = 0,
  REG_LOCATION,
  REG_FAN_CONTROL,
  REG_FAN_RPM_HI,
  REG_FAN_RPM_LO,
  REG_INTERNAL_TEMP,
  REG_EXTERNAL_TEMP_HI,
  REG_EXTERNAL_TEMP_LO,
  REG_ALERT_STATUS,
  REG_ALERT_MASK,
  REG_POR_STATUS,
  MAX_REGISTERS,
} register_fan_controller_t;


#endif
