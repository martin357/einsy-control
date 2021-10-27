#pragma once
#include "custom.h"
#include <Arduino.h>
#include <EEPROM.h>

#define EEPROM_START E2END + 1 - 128



struct Permanent_storage {
  void save();
  void load();
  #ifdef CUSTOM_PERMANENT_STORAGE
    CUSTOM_PERMANENT_STORAGE
  #endif
};


extern Permanent_storage storage;
