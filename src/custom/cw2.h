#pragma once
#include "../../custom.h"

#ifdef CUSTOM_CW2
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "cw2"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP

  #define PIN_OUT_1 X_MIN_PIN
  #define PIN_OUT_2 Y_MIN_PIN
  #define PIN_OUT_3 Z_MIN_PIN
  #define PIN_OUT_4 PIN_FAN_0 // 3.6V?
  #define PIN_OUT_5 PIN_FAN_1 // 3.6V?
  #define PIN_OUT_6 76 // PJ5
  #define PIN_OUT_7 A8 // PK0
  #define PIN_OUT_8 SDA
  #define PIN_OUT_9 SCL
  #define PIN_OUT_10 73 // PJ3
  #define PIN_OUT_11 53 // PROC_nCS
  #define PIN_OUT_12 2 // nAC_FAULT

#endif
