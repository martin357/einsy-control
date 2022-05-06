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

  extern void custom_gcode_autofill_on_delayed();
  extern void custom_gcode_autofill_on();
  extern void custom_gcode_autofill_off();
  extern void custom_gcode_fill_wait();
  extern void custom_gcode_empty_tank();
  // extern void custom_gcode_home_x();
  // extern void custom_gcode_home_z();
  // extern void custom_gcode_home_e();
  // extern void custom_gcode_collect();
  // extern void custom_gcode_hand_it_over();

  #define CUSTOM_GCODE \
    else if(strcmp_P(rx_command, F("autofill_on_delayed"))) custom_gcode_autofill_on_delayed();  \
    else if(strcmp_P(rx_command, F("autofill_on"))) custom_gcode_autofill_on();  \
    else if(strcmp_P(rx_command, F("autofill_off"))) custom_gcode_autofill_off();  \
    else if(strcmp_P(rx_command, F("fill_wait"))) custom_gcode_fill_wait();  \
    else if(strcmp_P(rx_command, F("empty_tank"))) custom_gcode_empty_tank();  \
    // else if(strcmp_P(rx_command, F("home_x"))) custom_gcode_home_x();  \
    // else if(strcmp_P(rx_command, F("home_z"))) custom_gcode_home_z();  \
    // else if(strcmp_P(rx_command, F("home_e"))) custom_gcode_home_e();  \
    // else if(strcmp_P(rx_command, F("collect"))) custom_gcode_collect();  \
    // else if(strcmp_P(rx_command, F("hand_it_over"))) custom_gcode_hand_it_over();  \

  #define CUSTOM_PERMANENT_STORAGE  \
    float zero_offset = 94.0;  \
    float level_min = 1.0;  \
    float level_optimal = 4.5;  \
    float level_max = 9.0;  \

#endif
