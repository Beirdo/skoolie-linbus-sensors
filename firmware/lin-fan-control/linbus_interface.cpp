#include <Arduino.h>
#include <LINBus_stack.h>
#include <LM96163.h>
#include <Wire.h>

#include "project.h"
#include "linbus_interface.h"
#include "linbus_registers.h"

uint8_t registerIndex = 0xFF;
uint8_t linbus_address;
uint8_t linbus_buf[2];
uint8_t linbus_buf_len = 2;

LINBusRegister registers[] = {
  LINBusRegister(0x00, BOARD_TYPE_FAN_CONTROL),
  LINBusRegister(0x00, 0xFF),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x00),
  LINBusRegister(0x00, 0x5B),
  LINBusRegister(0x00, 0x00),
};

LINBus_stack linbus(Serial, 19200);

// Lookup table for the fan when in heating mode
// The hotter the exhaust air, the lower the fan needs to run
// temperatures are in 0.5degC
const LM96163_LUT_t heat_lut[12] = {
  {0, 100},   // 0C    - 100%
  {20, 80},   // 10C   - 80%
  {27, 75},   // 13.5C - 75%
  {34, 65},   // 17C   - 65%
  {40, 60},   // 20C   - 60%
  {45, 55},   // 22.5C - 55%
  {50, 50},   // 25C   - 50%
  {55, 38},   // 27.5C - 38%
  {60, 25},   // 30C   - 25%
  {67, 20},   // 33.5C - 20%
  {74, 10},   // 37C   - 10%
  {80, 0},    // 40C   - 0%
};
uint8_t heat_hysteresis = 2;

// Lookup table for the fan when in cooling mode
// The colder the exhaust air, the lower the fan needs to run
const LM96163_LUT_t cool_lut[12] = {
  {0, 0},     // 0C    - 0%
  {20, 10},   // 10C   - 10%
  {27, 20},   // 13.5C - 20%
  {34, 25},   // 17C   - 25%
  {40, 38},   // 20C   - 38%
  {45, 50},   // 22.5C - 50%
  {50, 55},   // 25C   - 55%
  {55, 60},   // 27.5C - 55%
  {60, 65},   // 30C   - 65%
  {67, 75},   // 33.5C - 75%
  {74, 80},   // 37C   - 80%
  {80, 100},  // 40C   - 100%
  };
uint8_t cool_hysteresis = 2;

void init_linbus(uint8_t address)
{
  linbus_address = address & 0x1F;
  linbus.begin(PIN_LIN_WAKE, PIN_LIN_SLP, linbus_address);

  lm96163.begin(&Wire, PIN_ALERT, PIN_TCRIT);
}

void update_linbus(void)
{
  uint16_t rpm = lm96163.getStatus(LM96163_STATUS_TACH);
  registers[REG_FAN_RPM_HI].write(HI_BYTE(rpm), true);
  registers[REG_FAN_RPM_LO].write(LO_BYTE(rpm), true);

  registers[REG_INTERNAL_TEMP].write(lm96163.getStatus(LM96163_STATUS_LOCAL_TEMP), true);

  uint16_t remoteTemp = lm96163.getStatus(LM96163_STATUS_REMOTE_TEMP);
  // Now to normalize to our temperature (degC * 100)
  int32_t scaledTemp = (int16_t)(remoteTemp);
  scaledTemp *= 100;
  scaledTemp >>= 8;
  remoteTemp = (uint16_t)(scaledTemp & 0xFFFF);

  registers[REG_EXTERNAL_TEMP_HI].write(HI_BYTE(remoteTemp), true);
  registers[REG_EXTERNAL_TEMP_LO].write(LO_BYTE(remoteTemp), true);

  registers[REG_ALERT_STATUS].write(lm96163.getStatus(LM96163_STATUS_ALERT_STATUS), true);
  registers[REG_ALERT_MASK].write(lm96163.getStatus(LM96163_STATUS_ALERT_MASK), true);
  registers[REG_POR_STATUS].write(lm96163.getStatus(LM96163_STATUS_POR_STATUS), true);
}

void process_linbus(void)
{
  size_t read_;

  if (!linbus.waitBreak(50)) {
    return;    
  }

  if (linbus.read(linbus_buf, linbus_buf_len, &read_)) {
    if (read_) { 
      uint8_t addr = linbus_buf[0];
      uint8_t data = linbus_buf[1];

      if (addr & 0x80) {
        registerIndex = addr & 0x7F;
      } else if (addr < MAX_REGISTERS) {
        // this was a packet written to us
        registers[addr].write(data);

        if (addr == REG_FAN_CONTROL) {
          uint8_t pwm_value = clamp<uint8_t>((data & 0x3F) << 2, 0, 100);
          bool heat = data & 0x80;
          bool cool = data & 0x40;

          if (!heat && !cool) {
            lm96163.fanOnOff(true, pwm_value);
          } else if (heat) {
            lm96163.setLUT((LM96163_LUT_t *)heat_lut, heat_hysteresis);
          } else {
            lm96163.setLUT((LM96163_LUT_t *)cool_lut, cool_hysteresis);
          }
        } else if (addr == REG_ALERT_MASK) {
          lm96163.setAlertMask(data);
        }
      }
    } else {
      registerIndex = clamp<int>(registerIndex, 0, MAX_REGISTERS - 1);
      linbus_buf[0] = registerIndex < MAX_REGISTERS ? registers[registerIndex++].read() : 0x00;

      registerIndex = clamp<int>(registerIndex, 0, MAX_REGISTERS - 1);
      linbus_buf[1] = registerIndex < MAX_REGISTERS ? registers[registerIndex++].read() : 0x00;

      linbus.writeResponse(linbus_buf, 2);
    } 
  } else {
    linbus.sleep(STATE_SLEEP);
  }  
}