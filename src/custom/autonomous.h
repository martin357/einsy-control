#pragma once
#include "../../custom.h"

#ifdef CUSTOM_AUTONOMOUS
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "autonomous"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP

  #define PIN_HEATER HEATER_BED_PIN
  #define PIN_WASHING HEATER_0_PIN
  #define PIN_VALVE_OUT 76
  #define PIN_VALVE_IN A8
  #define PIN_DRYING_FAN 2 // nAC_FAULT (black)
  #define PIN_UV_LED 73 // PJ3 (brown)
  #define HEATER_THERM 2

  #define CUSTOM_PERMANENT_STORAGE  \
    uint8_t target_temperature = 40;  \
    uint16_t drying_preheat = 30;  \
    uint8_t drying_cycles = 1;  \
    uint8_t curing_cycles = 1;  \
    uint8_t  wash__valve_in_on_time = 5;  \
    uint8_t  wash__pump_on_valve_in_off_delay = 8;  \
    uint16_t wash__washing_duration = 10;  \
    uint8_t  wash__empty_pump_duration = 10;  \
    uint8_t cycle__do_collect = 1;  \
    uint8_t cycle__do_wash = 1;  \
    uint8_t cycle__do_dry = 1;  \
    uint8_t cycle__do_cure = 1;  \
    uint8_t cycle__do_hand_it_over = 1;  \

  extern void custom_gcode_do_cycle();
  extern void custom_gcode_home_x();
  extern void custom_gcode_home_z();
  extern void custom_gcode_home_e();
  extern void custom_gcode_collect();
  extern void custom_gcode_hand_it_over();

  #define CUSTOM_GCODE \
    else if(strcmp_P(rx_command, F("do_cycle"))) custom_gcode_do_cycle();  \
    else if(strcmp_P(rx_command, F("home_x"))) custom_gcode_home_x();  \
    else if(strcmp_P(rx_command, F("home_z"))) custom_gcode_home_z();  \
    else if(strcmp_P(rx_command, F("home_e"))) custom_gcode_home_e();  \
    else if(strcmp_P(rx_command, F("collect"))) custom_gcode_collect();  \
    else if(strcmp_P(rx_command, F("hand_it_over"))) custom_gcode_hand_it_over();  \

#endif
