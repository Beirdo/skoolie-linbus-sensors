#ifndef __sensor_eeprom_h_
#define __sensor_eeprom_h_

#include <Arduino.h>
#include <assert.h>

#define INTERNAL_EEPROM_SIZE      1024
#define INTERNAL_EEPROM_PAGE_SIZE 32

struct eeprom_v1_s {
  uint8_t version;
  uint8_t checksum;
  uint8_t board_num;
  uint8_t capabilities;
  uint8_t addr_pca9501_gpio;
  uint8_t addr_mcp96l01;
  uint8_t addr_ads7823;
  uint8_t addr_ds2482;
  uint8_t addr_linbus_bridge;
  uint32_t linbus_slaves;     // We support IDs 0x00 - 0x1F
};

typedef union {
  struct eeprom_v1_s v1;
  struct eeprom_v1_s current;
} eeprom_data_t;

static_assert(sizeof(eeprom_data_t) <= INTERNAL_EEPROM_SIZE, "Structure eeprom_data_t is larger than the EEPROM!");

#define CURRENT_EEPROM_VERSION  1
#define MAX_EEPROM_VERSION      1

#define CAPABILITIES_EXTERNAL_TEMP    0x01
#define CAPABILITIES_BATTERY_VOLTAGE  0x02
#define CAPABILITIES_COOLANT_TEMP     0x04
#define CAPABILITIES_EXHAUST_TEMP     0x08
#define CAPABILITIES_IGNITION_SOURCE  0x10
#define CAPABILITIES_LINBUS_BRIDGE    0x20

extern eeprom_data_t eeprom_data;
extern uint8_t eeprom_i2c_addr;
extern uint8_t bridge_i2c_addr;

void init_eeprom(void);
uint8_t eeprom_checksum(void *buf, int len);
uint32_t linbus_probe(void);

#endif