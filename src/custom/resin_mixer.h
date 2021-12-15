#pragma once
#include "../../custom.h"

#ifdef CUSTOM_RESIN_MIXER
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "resin_mixer"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP

  #define PIN_HEATER HEATER_BED_PIN
  #define HEATER_THERM 2

  #define CUSTOM_PERMANENT_STORAGE  \
    uint8_t target_temperature = 40;  \
    uint8_t mixing_duration = 1;  \
    uint16_t mixing_speed = 60;  \

#endif
