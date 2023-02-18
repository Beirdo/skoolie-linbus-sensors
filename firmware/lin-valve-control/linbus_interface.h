#ifndef __linbus_interface_h_
#define __linbus_interface_h_

#include <Arduino.h>
#include <SparkFun_TCA9534.h>


typedef enum {
  REG_BOARD_TYPE = 0,
  REG_VALVE_CONTROL,
  REG_VALVE_STATUS,
  REG_VALVE_CURRENT_HI,
  REG_VALVE_CURRENT_LO,
  MAX_REGISTERS,
} register_valve_controller_t;

typedef enum {
  BOARD_TYPE_VALVE_CONTROL = 0,
} board_types_t;

extern LINBus_stack linbus;
extern uint8_t linbus_address;
extern TCA9534 tca9534;

void init_linbus(uint8_t address);
void update_linbus(void);
void process_linbus(void);

#endif