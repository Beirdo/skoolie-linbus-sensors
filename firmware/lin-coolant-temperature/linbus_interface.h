#ifndef __linbus_interface_h_
#define __linbus_interface_h_

#include <Arduino.h>

extern LINBus_stack linbus;
extern uint8_t linbus_address;

void init_linbus(uint8_t address);
void update_linbus(void);
void process_linbus(void);

#endif
