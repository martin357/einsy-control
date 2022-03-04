#pragma once
#include "../../custom.h"

#ifdef CUSTOM_CAP_SENSE_TEST
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "cap_sense_test"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP
  #define DISABLE_STALLGUARD_TRIGGERED_PRINT_TO_SERIAL

  typedef enum
  {
    UNKNOWN = -1,
    BELOW = 0,
    NORMAL,
    ABOVE,
  } ResinLevel;

#endif
