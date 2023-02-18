#ifndef __linbus_interface_h_
#define __linbus_interface_h_

#include <Arduino.h>
#include <SparkFun_TCA9534.h>

extern LINBus_stack linbus;
extern uint8_t linbus_address;
extern TCA9534 tca9534;

void init_linbus(uint8_t address);
void update_linbus(void);
void process_linbus(void);

#endif