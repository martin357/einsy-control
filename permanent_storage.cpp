#include "permanent_storage.h"

Permanent_storage storage;



void Permanent_storage::save(){
  const uint8_t size = sizeof(storage);
  uint8_t* view = reinterpret_cast<uint8_t*>(&storage);
  uint16_t crc = 0;
  for(uint8_t i = 0; i < size; i++) crc ^= (uint16_t)*(view+i) << 8*(i % 2);

  EEPROM.update(EEPROM_START, size);
  EEPROM.update(EEPROM_START + sizeof(size), crc);

  uint8_t to_write = sizeof(storage);
  uint8_t eeprom_pos = EEPROM_START + sizeof(size) + sizeof(crc);
  while(to_write--) EEPROM.update(eeprom_pos++, *view++);

}


void Permanent_storage::load(){
  const uint8_t size = sizeof(storage);
  uint8_t* view = reinterpret_cast<uint8_t*>(&storage);
  uint16_t crc = 0;
  for(uint8_t i = 0; i < size; i++) crc ^= (uint16_t)*(view+i) << 8*(i % 2);

  uint8_t size_in_eeprom;
  uint16_t crc_in_eeprom;
  EEPROM.get(EEPROM_START, size_in_eeprom);
  EEPROM.get(EEPROM_START + sizeof(size_in_eeprom), crc_in_eeprom);

  if(size_in_eeprom == 0xFF) return;
  if(size_in_eeprom != size) return;
  if(crc_in_eeprom == crc) return;

  uint8_t to_read = sizeof(storage);
  uint8_t eeprom_pos = EEPROM_START + sizeof(size) + sizeof(crc);
  while(to_read--) EEPROM.get(eeprom_pos++, *view++);

}
