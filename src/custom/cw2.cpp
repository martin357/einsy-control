#include "cw.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#include "../../serial.h"
#ifdef CUSTOM_CW2


// custom CW2 stuff
// bool is_valve_on(){ return digitalReadExt(PIN_VALVE); }
// void do_valve_on(){ digitalWriteExt(PIN_VALVE, HIGH); }
// void do_valve_off(){ digitalWriteExt(PIN_VALVE, LOW); }
// const char pgmstr_valve_on[] PROGMEM = "Valve: on";
// const char pgmstr_valve_off[] PROGMEM = "Valve: off";
// MenuItemToggleCallable item_valve_on_off(&is_valve_on, pgmstr_valve_on, pgmstr_valve_off, &do_valve_off, &do_valve_on);

#define SETUP_PIN(n) \
  pinModeOutput(PIN_OUT_##n); \
  digitalWriteExt(PIN_OUT_##n, LOW); \

// #define SET_PIN(n, v) \
//   digitalWriteExt(PIN_OUT_##n, v); \


void setupCustom(){
  SETUP_PIN(1);
  SETUP_PIN(2);
  SETUP_PIN(3);
  SETUP_PIN(4);
  SETUP_PIN(5);
  SETUP_PIN(6);
  SETUP_PIN(7);
  SETUP_PIN(8);
  SETUP_PIN(9);
  SETUP_PIN(10);
  SETUP_PIN(11);
  SETUP_PIN(12);
}

void loopCustom(){
  const uint32_t _millis = millis();

  // static bool val = false;
  // Serial.print(F("new value: "));
  // Serial.println(val);
  // SET_PIN(1, val);
  // SET_PIN(2, val);
  // SET_PIN(3, val);
  // SET_PIN(4, val);
  // SET_PIN(5, val);
  // SET_PIN(6, val);
  // SET_PIN(7, val);
  // SET_PIN(8, val);
  // SET_PIN(9, val);
  // SET_PIN(10, val);
  // SET_PIN(11, val);
  // SET_PIN(12, val);
  // val = !val;

  Serial.println(F("ON"));
  SET_PIN(1);
  SET_PIN(2);
  SET_PIN(3);
  SET_PIN(4);
  SET_PIN(5);
  SET_PIN(6);
  SET_PIN(7);
  SET_PIN(8);
  SET_PIN(9);
  SET_PIN(10);
  SET_PIN(11);
  SET_PIN(12);
  delay(1000);

  Serial.println(F("Off"));
  RESET_PIN(1);
  RESET_PIN(2);
  RESET_PIN(3);
  RESET_PIN(4);
  RESET_PIN(5);
  RESET_PIN(6);
  RESET_PIN(7);
  RESET_PIN(8);
  RESET_PIN(9);
  RESET_PIN(10);
  RESET_PIN(11);
  RESET_PIN(12);

  delay(1000);
}


// debug menu
MenuItem* const debug_menu_items[] PROGMEM = {
  &back,
  // &item_valve_on_off,
  // &motor_x,
  // &motor_y,
  // &motor_z,
  // &motor_e,
};
Menu debug_menu(debug_menu_items, sizeof(debug_menu_items) / 2);
const char pgmstr_debug[] PROGMEM = "!!! Debug";
MenuItem item_debug_menu(pgmstr_debug, &debug_menu);


// main menu
MenuItem* const main_menu_items[] PROGMEM = {
  &item_debug_menu,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);


#endif
