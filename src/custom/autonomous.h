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
  // #define PIN_WASHING_PUMP HEATER_0_PIN
  #define PIN_WASHING HEATER_0_PIN
  #define PIN_VALVE_OUT 76
  #define PIN_VALVE_IN A8
  #define PIN_DRYING_FAN 2 // nAC_FAULT (black)
  #define PIN_UV_LED 73 // PJ3 (brown)

  #define CUSTOM_PERMANENT_STORAGE  \
    uint8_t target_temperature = 40;  \
    // uint8_t washing_duration = 1;  \

  #include <Arduino.h>
  #include "../../menu_system_basics.h"
  const short temperature_table_einsy[][2] PROGMEM = {
    {23, 300}, {25, 295}, {27, 290}, {28, 285}, {31, 280}, {33, 275}, {35, 270},
    {38, 265}, {41, 260}, {44, 255}, {48, 250}, {52, 245}, {56, 240}, {61, 235},
    {66, 230}, {71, 225}, {78, 220}, {84, 215}, {92, 210}, {100, 205}, {109, 200},
    {120, 195}, {131, 190}, {143, 185}, {156, 180}, {171, 175}, {187, 170}, {205, 165},
    {224, 160}, {245, 155}, {268, 150}, {293, 145}, {320, 140}, {348, 135}, {379, 130},
    {411, 125}, {445, 120}, {480, 115}, {516, 110}, {553, 105}, {591, 100}, {628, 95},
    {665, 90}, {702, 85}, {737, 80}, {770, 75}, {801, 70}, {830, 65}, {857, 60},
    {881, 55}, {903, 50}, {922, 45}, {939, 40}, {954, 35}, {966, 30}, {977, 25},
    {985, 20}, {993, 15}, {999, 10}, {1004, 5}, {1008, 0}};

  #define TEMPERATURE_TABLE_EINSY_LEN (sizeof(temperature_table_einsy)/sizeof(*temperature_table_einsy))

  #define BED_OFFSET 10
  #define BED_OFFSET_START 40
  #define BED_OFFSET_CENTER 50

  #define HEATER_THERM 2

  #define PGM_RD_W(x)   (short)pgm_read_word(&x)

  extern void custom_gcode_home_x();
  extern void custom_gcode_home_z();
  extern void custom_gcode_home_e();
  extern void custom_gcode_collect();
  extern void custom_gcode_hand_it_over();

  #define CUSTOM_GCODE \
    else if(strcmp_P(rx_command, F("home_x"))) custom_gcode_home_x();  \
    else if(strcmp_P(rx_command, F("home_z"))) custom_gcode_home_z();  \
    else if(strcmp_P(rx_command, F("home_e"))) custom_gcode_home_e();  \
    else if(strcmp_P(rx_command, F("collect"))) custom_gcode_collect();  \
    else if(strcmp_P(rx_command, F("hand_it_over"))) custom_gcode_hand_it_over();  \

#endif
