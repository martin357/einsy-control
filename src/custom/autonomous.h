#pragma once
#include "../../custom.h"

#ifdef CUSTOM_AUTONOMOUS
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "autonomous"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP

  #define PIN_UV_LED 73
  #define PIN_HEATER 76
  #define PIN_CAP_IN 12
  #define PIN_WATER_PUMP HEATER_0_PIN
  #define PIN_VALVE HEATER_BED_PIN
#endif
