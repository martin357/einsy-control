#pragma once
#include "../../custom.h"

#ifdef CUSTOM_CW
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "cw"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP

  #define PIN_UV_LED 76
  #define PIN_HEATER 73
  #define PIN_CAP_IN 12
  #define PIN_WATER_PUMP HEATER_0_PIN
  #define PIN_VALVE HEATER_BED_PIN
#endif
