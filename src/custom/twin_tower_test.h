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
  // #define DISABLE_STALLGUARD_TRIGGERED_PRINT_TO_SERIAL

  #define ML  2
  #define MR  1
  #define CURRENT_MP  1
  // #define CURRENT_MP  2
  #define WAIT_FOR_TOWERS Serial.println(F("wait_for_towers")); delay(10); processCommand(F("wait_for_motor y z"))
  // #define WAIT_FOR_TOWERS Serial.println(F("wait_for_towers")); delay(10); processCommand(F("wait_for_motor z"))

  extern void custom_gcode_home_weak();
  extern void custom_gcode_home_tilt();
  extern void custom_gcode_home_tower();
  extern void custom_gcode_is_endstop_triggered();
  // extern void custom_gcode_autofill_on();

  #define CUSTOM_GCODE \
    else if(strcmp_P(rx_command, F("home_weak"))) custom_gcode_home_weak();  \
    else if(strcmp_P(rx_command, F("home_tilt"))) custom_gcode_home_tilt();  \
    else if(strcmp_P(rx_command, F("home_tower"))) custom_gcode_home_tower();  \
    else if(strcmp_P(rx_command, F("is_endstop_triggered"))) custom_gcode_is_endstop_triggered();  \
  //   else if(strcmp_P(rx_command, F("autofill_on"))) custom_gcode_autofill_on();  \
  //   else if(strcmp_P(rx_command, F("autofill_off"))) custom_gcode_autofill_off();  \
  //   else if(strcmp_P(rx_command, F("fill_wait"))) custom_gcode_fill_wait();  \
  //   else if(strcmp_P(rx_command, F("empty_tank"))) custom_gcode_empty_tank();  \
  //   else if(strcmp_P(rx_command, F("home_x"))) custom_gcode_home_x();  \
  //   else if(strcmp_P(rx_command, F("home_z"))) custom_gcode_home_z();  \
  //   else if(strcmp_P(rx_command, F("home_e"))) custom_gcode_home_e();  \
  //   else if(strcmp_P(rx_command, F("collect"))) custom_gcode_collect();  \
  //   else if(strcmp_P(rx_command, F("hand_it_over"))) custom_gcode_hand_it_over();  \

#endif
