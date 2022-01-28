#pragma once
#include "../../custom.h"
#include "../../serial.h"

#ifdef CUSTOM_GESTURES_TEST
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "gestures_test"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  // #define CUSTOM_LOOP
  #define CUSTOM_TIMER0_ISR

  #define PIN_IR_1 12 // X_MIN
  #define PIN_IR_2 11 // Y_MIN
  #define PIN_IR_3 10 // Z_MIN
  #define PIN_IR_4 73 // PJ3

#endif
