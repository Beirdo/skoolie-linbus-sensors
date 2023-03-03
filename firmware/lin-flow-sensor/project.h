#ifndef __project_h_
#define __project_h_

#include <Arduino.h>
#include <Beirdo-Utilities.h>

#define PIN_SCL           PIN_PB0
#define PIN_SDA           PIN_PB1
#define PIN_TXD           PIN_PB2
#define PIN_RXD           PIN_PB3
#define PIN_UPDI          PIN_PA0
#define PIN_FLOW_PULSE0   PIN_PA1
#define PIN_FLOW_PULSE1   PIN_PA2
#define PIN_RESET         PIN_PA3
#define PIN_LIN_SLP       PIN_PA4
#define PIN_LIN_WAKE      PIN_PA5
#define PIN_LED           PIN_PA7

extern uint16_t pulse0_count;
extern uint16_t pulse1_count;
extern int last_millis;

#endif
