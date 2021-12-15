#pragma once
#include "../../custom.h"

#ifdef CUSTOM_HEATBED
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "heatbed"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP
  #define PIN_HEATER HEATER_BED_PIN

  #define CUSTOM_PERMANENT_STORAGE  \
    uint8_t target_temperature = 40;  \

#endif
