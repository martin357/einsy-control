#pragma once
#include "../../custom.h"

#ifdef CUSTOM_AUTONOMOUS
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "autonomous"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP

  #define PIN_WASHING HEATER_BED_PIN
  #define PIN_HEATING HEATER_0_PIN
  #define PIN_VALVE_0 76
  #define PIN_VALVE_1 A8

  #include <Arduino.h>
  #include "../../menu_system_basics.h"

#endif
