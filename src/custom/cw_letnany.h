#pragma once
#include "../../custom.h"

#ifdef CUSTOM_CW_LETNANY
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "cw_letnany"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP

  #define PIN_HEATER HEATER_BED_PIN
  #define PIN_WASHING_PUMP HEATER_0_PIN

#endif
