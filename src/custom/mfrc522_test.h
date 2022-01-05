#pragma once
#include "../../custom.h"

#ifdef CUSTOM_MFRC522_TEST
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "mfrc522_test"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP
  #define PIN_HEATER HEATER_BED_PIN

#endif
