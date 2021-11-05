#pragma once
#include "../../custom.h"

#ifdef CUSTOM_CW
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "cw"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP

  #define PIN_UV_LED 76
  #define PIN_HEATER 73
  #define PIN_CAP_IN_PRIMARY PIN_TACH_0
  #define PIN_CAP_OUT_PRIMARY PIN_FAN_0
  #define PIN_CAP_IN_CRITICAL PIN_TACH_1
  #define PIN_CAP_OUT_CRITICAL PIN_FAN_1
  #define PIN_WATER_PUMP HEATER_0_PIN
  #define PIN_VALVE HEATER_BED_PIN

  #define CUSTOM_PERMANENT_STORAGE  \
    uint8_t washing_duration = 4;  \
    uint8_t drying_duration = 5;  \
    uint8_t curing_duration = 3;  \
    uint8_t stabilization_duration = 0;  \
    uint8_t model_height = 0;  \
    bool watch_for_critical_level = true;  \

  void turn_water_level_sensors(bool);

#endif
