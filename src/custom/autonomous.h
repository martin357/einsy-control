#pragma once
#include "../../custom.h"

#ifdef CUSTOM_AUTONOMOUS
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "autonomous"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP

  #define PIN_WASHING HEATER_0_PIN
  #define PIN_HEATING HEATER_BED_PIN
  #define PIN_VALVE_OUT 76
  #define PIN_VALVE_IN A8
  #define PIN_DRYING_FAN 2 // nAC_FAULT (black)
  #define PIN_UV_LED 73 // PJ3 (brown)

  #include <Arduino.h>
  #include "../../menu_system_basics.h"

#endif
