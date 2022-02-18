#pragma once
#include "../../custom.h"

#ifdef CUSTOM_TILT_TEST
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "tilt_test"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP

  #define CUSTOM_PERMANENT_STORAGE  \
    uint8_t steps = 10;  \
    uint8_t speed = 30;  \

#endif
