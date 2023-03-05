#ifndef __register_bank_h_
#define __register_bank_h_

#include <Arduino.h>


typedef enum {
  REG_BOARD_TYPE = 0,
  REG_LOCATION,
  REG_FLOW_RATE0_HI,
  REG_FLOW_RATE0_LO,
  REG_FLOW_RATE1_HI,
  REG_FLOW_RATE1_LO,
  MAX_REGISTERS,
} register_flow_sensor_t;


#endif
