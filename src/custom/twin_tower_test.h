#pragma once
#include "../../custom.h"

#ifdef CUSTOM_TWIN_TOWER_TEST
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "twin_tower_test"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP
  #define DISABLE_STALLGUARD_TRIGGERED_PRINT_TO_SERIAL
  #define DISABLE_MOTOR_INACTIVITY_TIMEOUT_PRINT_TO_SERIAL

  #define ML  2 // motor - tower left
  #define MR  1 // motor - tower right
  #define MP  0 // motor - pump
  #define MT  3 // motor - tilt
  #define CURRENT_MP  1
  #define WAIT_FOR_TOWERS Serial.println(F("wait_for_towers")); delay(10); processCommand(F("wait_for_motor y z"))
  #define DEFAULT_PUMP_AUTO_OFF_TIME  1800500
  // #define WAIT_FOR_TOWERS Serial.println(F("wait_for_towers")); delay(10); processCommand(F("wait_for_motor z"))

  typedef enum
  {
    UNKNOWN = -1,
    BELOW = 0,
    NORMAL,
    ABOVE,
  } ResinLevel;

  extern void custom_gcode_home_weak();
  extern void custom_gcode_home_tilt();
  extern void custom_gcode_home_tower();
  extern void custom_gcode_is_endstop_triggered();

  extern void custom_gcode_autofill_on_delayed();
  extern void custom_gcode_autofill_on();
  extern void custom_gcode_autofill_off();
  extern void custom_gcode_fill_wait();
  extern void custom_gcode_fill_wait_stable();
  extern void custom_gcode_empty_tank();
  extern void custom_gcode_get_level_optimal();
  extern void custom_gcode_get_level_fill();
  extern void custom_gcode_get_level_tolerance();
  extern void custom_gcode_capsense_raw();
  extern void custom_gcode_capsense();
  extern void custom_gcode_set_target_fill();
  extern void custom_gcode_set_target_optimal();

  #define CUSTOM_GCODE \
    else if(strcmp_P(rx_command, F("home_weak"))) custom_gcode_home_weak();  \
    else if(strcmp_P(rx_command, F("home_tilt"))) custom_gcode_home_tilt();  \
    else if(strcmp_P(rx_command, F("home_tower"))) custom_gcode_home_tower();  \
    else if(strcmp_P(rx_command, F("is_endstop_triggered"))) custom_gcode_is_endstop_triggered();  \
      \
    else if(strcmp_P(rx_command, F("autofill_on_delayed"))) custom_gcode_autofill_on_delayed();  \
    else if(strcmp_P(rx_command, F("autofill_on"))) custom_gcode_autofill_on();  \
    else if(strcmp_P(rx_command, F("autofill_off"))) custom_gcode_autofill_off();  \
    else if(strcmp_P(rx_command, F("fill_wait"))) custom_gcode_fill_wait();  \
    else if(strcmp_P(rx_command, F("empty_tank"))) custom_gcode_empty_tank();  \
    else if(strcmp_P(rx_command, F("get_level_fill"))) custom_gcode_get_level_fill();  \
    else if(strcmp_P(rx_command, F("get_level_optimal"))) custom_gcode_get_level_optimal();  \
    else if(strcmp_P(rx_command, F("get_level_tolerance"))) custom_gcode_get_level_tolerance();  \
    else if(strcmp_P(rx_command, F("capsense_raw"))) custom_gcode_capsense_raw();  \
    else if(strcmp_P(rx_command, F("capsense"))) custom_gcode_capsense();  \
    else if(strcmp_P(rx_command, F("set_target_fill"))) custom_gcode_set_target_fill();  \
    else if(strcmp_P(rx_command, F("set_target_optimal"))) custom_gcode_set_target_optimal();  \

  #define CUSTOM_PERMANENT_STORAGE  \
    double zero_offset = 32.66;  \
    double level_min = 1.0;  \
    double level_fill = 3.47;  \
    double level_optimal = 5.65;  \
    double level_max = 7.03;  \

    // double zero_offset = 28.37;  \
    // double level_min = 1.0;  \
    // double level_fill = 2.41;  \
    // double level_optimal = 3.72;  \
    // double level_max = 6.29;  \

    // double zero_offset = 94.0;  \
    // double level_min = 1.0;  \
    // double level_optimal = 4.5;  \
    // double level_fill = 4.5;  \
    // double level_max = 9.0;  \

#endif
