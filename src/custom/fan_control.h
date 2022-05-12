#pragma once
#include "../../custom.h"

#ifdef CUSTOM_FAN_CONTROL
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "fan_control"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP

#endif
