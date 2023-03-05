#ifndef __project_h_
#define __project_h_

#include <Arduino.h>
#include <Beirdo-Utilities.h>
#include <TCA9534-GPIO.h>

#define PIN_SCL           PIN_PB0
#define PIN_SDA           PIN_PB1
#define PIN_TXD           PIN_PB2
#define PIN_RXD           PIN_PB3
#define PIN_UPDI          PIN_PA0
#define PIN_VPROPI        PIN_PA1
#define PIN_I2C_INT       PIN_PA2
#define PIN_RESET         PIN_PA3
#define PIN_PHASE_PWM     PIN_PA4
#define PIN_LIN_SLP       PIN_PA5
#define PIN_LIN_WAKE      PIN_PA6
#define PIN_LED           PIN_PA7

// On the TCA9534
#define PIN_ENABLE        0
#define PIN_SLEEP         1
#define PIN_VALVE_CLOSED  4
#define PIN_VALVE_OPENED  5
#define PIN_FAULT         6

#define VALVE_CLOSED      BIT(3)
#define VALVE_OPENED      BIT(2)
#define VALVE_CLOSE       BIT(1)
#define VALVE_OPEN        BIT(0)

#define I2C_ADDR0_LCD     0x27
#define I2C_ADDR1_LCD     0x3F
#define I2C_ADDR_TCA9534  0x20

void setPhasePWM(bool cw, bool ccw);

extern TCA9534 tca9534;

#endif
