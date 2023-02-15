#include <Arduino.h>
#include <stdlib.h>
#include <EEPROM.h>

#include "project.h"
#include "sensor_eeprom.h"

eeprom_data_t eeprom_data;
uint8_t eeprom_i2c_addr;
uint8_t bridge_i2c_addr;

uint8_t eeprom_checksum(void *data, int len)
{
  uint8_t *buf = static_cast<uint8_t *>(data);
  uint8_t checksum = 0;
  for (int i = 0; i < len; i++) {
    checksum ^= *buf;
  }
  return checksum;
}

void init_eeprom(void) 
{
  uint8_t addr = 0x70;
  addr |= digitalRead(PIN_I2C_A2) ? BIT(2) : 0;        
  addr |= digitalRead(PIN_I2C_A1) ? BIT(1) : 0;        
  addr |= digitalRead(PIN_I2C_A0) ? BIT(0) : 0;

  eeprom_i2c_addr = addr;
  bridge_i2c_addr = addr | 0x08;

  memset((char *)&eeprom_data, 0xFF, sizeof(eeprom_data));
  
  // Read it from the EEPROM
  EEPROM.get(0, eeprom_data);

  if (eeprom_checksum(&eeprom_data, sizeof(eeprom_data))) {
    // Bad checksum...  Overwrite it.
    eeprom_data.current.version = 0xFF;
  }

  // If not initialized, we need to put in values as appropriate
  if (eeprom_data.current.version == 0xFF) {
    memset((char *)&eeprom_data, 0xFF, sizeof(eeprom_data));

    eeprom_data.current.version = CURRENT_EEPROM_VERSION;
    eeprom_data.current.board_num = addr & 0x07;
    eeprom_data.current.capabilities = CAPABILITIES_LINBUS_BRIDGE;
    eeprom_data.current.addr_linbus_bridge = bridge_i2c_addr;
  }

  eeprom_data.current.linbus_slaves = linbus_probe();
  eeprom_data.current.checksum = eeprom_checksum(&eeprom_data, sizeof(eeprom_data));
  EEPROM.put(0, eeprom_data);
}
