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
  #define TILT_TEMPERATURE  temperature[2]
  #define FEP_TEMPERATURE  temperature[1]

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

  extern void custom_gcode_autofill_on_delayed_short();
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
  extern void custom_gcode_tilt_temp();
  extern void custom_gcode_level_ir_gate();

  #define CUSTOM_GCODE \
    else if(strcmp_P(rx_command, F("home_weak"))) custom_gcode_home_weak();  \
    else if(strcmp_P(rx_command, F("home_tilt"))) custom_gcode_home_tilt();  \
    else if(strcmp_P(rx_command, F("home_tower"))) custom_gcode_home_tower();  \
    else if(strcmp_P(rx_command, F("is_endstop_triggered"))) custom_gcode_is_endstop_triggered();  \
      \
    else if(strcmp_P(rx_command, F("autofill_on_delayed_short"))) custom_gcode_autofill_on_delayed_short();  \
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
    else if(strcmp_P(rx_command, F("tilt_temp"))) custom_gcode_tilt_temp();  \
    else if(strcmp_P(rx_command, F("level_ir_gate"))) custom_gcode_level_ir_gate();  \

  #define CUSTOM_PERMANENT_STORAGE  \
    float zero_offset = -2.5;  \
    float level_min = 1.0;  \
    float level_fill = 1.0;  \
    float level_optimal = 2.5;  \
    float level_max = 6.0;  \
    float tilt_max_temperature = 60.0;  \
    float valid_min = -1.2;  \
    float valid_max = 50.0;  \

    // float zero_offset = -3.32;  \
    // float level_min = 1.0;  \
    // float level_fill = 2.5;  \
    // float level_optimal = 4.5;  \
    // float level_max = 6.0;  \

    // float zero_offset = -3.32;  \
    // float level_min = 1.0;  \
    // float level_fill = 3.47;  \
    // float level_optimal = 5.65;  \
    // float level_max = 7.03;  \

    // float zero_offset = 32.66;  \
    // float zero_offset = 28.37;  \
    // float level_min = 1.0;  \
    // float level_fill = 2.41;  \
    // float level_optimal = 3.72;  \
    // float level_max = 6.29;  \

    // float zero_offset = 94.0;  \
    // float level_min = 1.0;  \
    // float level_optimal = 4.5;  \
    // float level_fill = 4.5;  \
    // float level_max = 9.0;  \


    // #define SET_PIN(n)   PIN_##n##_PORT |= PIN_##n##_BIT
    // #define RESET_PIN(n) PIN_##n##_PORT ^= PIN_##n##_BIT
    // #define SETUP_PIN(n) \
    //   pinModeOutput(PIN_OUT_##n); \
    //   digitalWriteExt(PIN_OUT_##n, LOW); \
    //
    // #define PIN_OUT_1  X_MIN_PIN
    // #define PIN_OUT_2  Y_MIN_PIN
    // #define PIN_OUT_3  53 // PB0
    // #define PIN_OUT_4  73 // PJ3
    // #define PIN_OUT_5  76 // PJ5
    // #define PIN_OUT_6  A8 // PK0
    //
    // #define PIN_1_PORT  PORTB // PB6 .. X_MIN_PIN
    // #define PIN_2_PORT  PORTB // PB5 .. Y_MIN_PIN
    // #define PIN_3_PORT  PORTB // PB0 .. PROC_nCS
    // #define PIN_4_PORT  PORTJ // PJ3
    // #define PIN_5_PORT  PORTJ // PJ5
    // #define PIN_6_PORT  PORTK // PK0 .. A8
    //
    // #define PIN_1_BIT  _BV(6)
    // #define PIN_2_BIT  _BV(5)
    // #define PIN_3_BIT  _BV(0)
    // #define PIN_4_BIT  _BV(3)
    // #define PIN_5_BIT  _BV(5)
    // #define PIN_6_BIT  _BV(0)

#endif
