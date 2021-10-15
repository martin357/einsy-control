#include "cw.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#include "../../serial.h"
#ifdef CUSTOM_CW


#define PIN_UV_LED 73
#define PIN_HEATER 76
#define PIN_CAP_IN 12
#define PIN_WATER_PUMP HEATER_0_PIN
#define PIN_VALVE HEATER_BED_PIN


/// custom stuff
bool watch_water_level = false;


void setupCustom(){
  for (size_t i = 0; i < MOTORS_MAX; i++) {
    motors[i].driver.rms_current(400);
    motors[i].driver.sgt(6);
    motors[i].rpm(120);
  }

  // touch capacitance sensor
  pinModeInput(PIN_CAP_IN);

  // CW board trigger pin - UV led
  pinModeOutput(PIN_UV_LED);
  digitalWriteExt(PIN_UV_LED, HIGH);

  // CW board trigger pin - heater
  pinModeOutput(PIN_HEATER);
  digitalWriteExt(PIN_HEATER, HIGH);

  // power output
  pinModeOutput(PIN_VALVE); // bed mosfet
  digitalWriteExt(PIN_VALVE, LOW);
  pinModeOutput(PIN_WATER_PUMP); // extruder mosfet
  digitalWriteExt(PIN_WATER_PUMP, LOW);

}


void loopCustom(){
  uint32_t _millis = millis();

  static uint32_t last_watch_water_level = 0;
  if(watch_water_level && _millis > last_watch_water_level + 100){
    bool water_detected = digitalReadExt(PIN_CAP_IN);
    digitalWriteExt(PIN_WATER_PUMP, !water_detected);
    digitalWriteExt(PIN_VALVE, water_detected);
    last_watch_water_level = _millis;
  }

}



// custom CW stuff
bool is_valve_on(){ return digitalReadExt(PIN_VALVE); }
void do_valve_on(){ digitalWriteExt(PIN_VALVE, HIGH); }
void do_valve_off(){ digitalWriteExt(PIN_VALVE, LOW); }
MenuItemToggleCallable item_valve_on_off(&is_valve_on, "Valve: on", "Valve: off", &do_valve_off, &do_valve_on);

bool is_water_pump_on(){ return digitalReadExt(PIN_WATER_PUMP); }
void do_water_pump_on(){ digitalWriteExt(PIN_WATER_PUMP, HIGH); }
void do_water_pump_off(){ digitalWriteExt(PIN_WATER_PUMP, LOW); }
MenuItemToggleCallable item_water_pump_on_off(&is_water_pump_on, "Water pump: on", "Water pump: off", &do_water_pump_off, &do_water_pump_on);

bool is_watch_water_level_on(){ return watch_water_level; }
void do_watch_water_level_on(){ watch_water_level = true; }
void do_watch_water_level_off(){ watch_water_level = false; digitalWriteExt(PIN_WATER_PUMP, LOW); digitalWriteExt(PIN_VALVE, LOW); }
MenuItemToggleCallable item_watch_water_level_on_off(&is_watch_water_level_on, "Water level: on", "Water level: off", &do_watch_water_level_off, &do_watch_water_level_on);



bool is_homed = false;
void do_home_z_up(){ processCommand(F("home z0")); }
MenuItemCallable item_home_z_up("Home Z up", &do_home_z_up, false);

void do_home_z_down(){
  processCommand(F("home z1 b0.5 f180 g60"));
  processCommand(F("dir z0"));
  processCommand(F("rpm z180"));
  // processCommand(F("move_rot z0.2"));
  processCommand(F("move_rot z0.5"));
  is_homed = true;
}
MenuItemCallable item_home_z_down("Home Z down", &do_home_z_down, false);

void ensure_homed(){
  if(!is_homed){
    do_home_z_down();
    processCommand(F("wait_for_motor z"));
  }
}

// void do_test_m400(){
//   processCommand(F("rpm z60"));
//   processCommand(F("dir z1"));
//   processCommand(F("move_rot z1"));
//   // processCommand(F("start z0"));
//   processCommand(F("wait_for_motor z"));
//   Serial.println(">>> BEEP");
//   beep();
//   processCommand(F("dir z0"));
//   processCommand(F("move_rot z1"));
//   // processCommand(F("start z0"));
// }
// MenuItemCallable item_test_m400("test M400", &do_test_m400, false);

void do_fill_tank(){
  const uint16_t stabilization_duration = 3000;
  const uint32_t start_time = millis();
  uint32_t stabilization_start;
  lcd.clear();
  lcd.print("Filling tank...");
  digitalWriteExt(PIN_VALVE, 0);
  digitalWriteExt(PIN_WATER_PUMP, 1);
  while(1){
    const bool water_detected = digitalReadExt(PIN_CAP_IN);
    if(water_detected){
      digitalWriteExt(PIN_WATER_PUMP, 0);
      digitalWriteExt(PIN_VALVE, 1);
      lcd.clear();
      lcd.print("Stabilizing...");
      stabilization_start = millis();
      break;
    }else{
      delay(100);
    }
  }

  while(millis() < stabilization_start + stabilization_duration){
    const bool water_detected = digitalReadExt(PIN_CAP_IN);
    digitalWriteExt(PIN_VALVE, water_detected);
    digitalWriteExt(PIN_WATER_PUMP, !water_detected);
    delay(200);
  }
  digitalWriteExt(PIN_WATER_PUMP, 0);
  digitalWriteExt(PIN_VALVE, 1);
  delay(500);
  digitalWriteExt(PIN_VALVE, 0);
  const uint32_t duration = millis() - start_time;
  lcd.clear();
  lcd.print("Finished!");
  lcd.setCursor(0, 2);
  lcd.print("In ");
  lcd.print((uint16_t)(duration / 1000));
  lcd.print("s");
  beep(50);
  delay(2000);

}
MenuItemCallable item_fill_tank("Fill tank", &do_fill_tank, false);


void do_empty_tank(){
  const uint16_t emptying_duration = 60 * 1000;
  const uint32_t emptying_end = millis() + emptying_duration;
  digitalWriteExt(PIN_WATER_PUMP, 0);
  digitalWriteExt(PIN_VALVE, 1);
  lcd.clear();
  lcd.print("Emptying tank...");
  int32_t remaining;
  while((remaining = emptying_end - millis()) > 100){
    lcd.setCursor(0, 2);
    lcd.print("Remaining ");
    lcd.print((uint16_t)(remaining / 1000));
    lcd.print("s ");
    delay(300);
  }
  digitalWriteExt(PIN_VALVE, 0);
  lcd.clear();
  lcd.print("Tank is empty");
  beep(50);
  delay(2000);
}
MenuItemCallable item_empty_tank("Empty tank", &do_empty_tank, false);

void do_fill_and_empty(){
  do_fill_tank();
  do_empty_tank();
}
MenuItemCallable item_fill_and_empty("Fill & empty", &do_fill_and_empty, false);


void do_move_up(bool do_wait = true){
  ensure_homed();
  if(do_wait){
    lcd.clear();
    lcd.print("Moving platform up");
  }
  processCommand(F("dir z0"));
  processCommand(F("move_ramp s0.1 f300 a150 d100 z42"));
  if(do_wait){
    processCommand(F("wait_for_motor z"));
    beep(50);
  }
}
MenuItemCallable item_move_up("Move platform up", &do_move_up, false);

void do_move_down(bool do_wait = true){
  if(!is_homed){
    do_home_z_down();
    return;
  }
  if(do_wait){
    lcd.clear();
    lcd.print("Moving platform down");
  }
  processCommand(F("dir z1"));
  processCommand(F("move_ramp s0.1 f300 a150 d100 z42"));
  if(do_wait){
    processCommand(F("wait_for_motor z"));
    beep(50);
  }
}
MenuItemCallable item_move_down("Move platform down", &do_move_down, false);

void do_start_washing(){
  // move down and fill tank
  do_move_down(false);
  do_fill_tank();

  // wash model for some time...
  lcd.clear();
  lcd.print("Washing...");
  processCommand(F("dir x1"));
  processCommand(F("rpm x0.1"));
  processCommand(F("ramp_to x230"));
  processCommand(F("start x1"));
  processCommand(F("wait x15000"));
  processCommand(F("ramp_to x0.1"));
  processCommand(F("wait x2000"));
  processCommand(F("stop x"));

  // processCommand(F("dir x0"));
  // processCommand(F("rpm x0.1"));
  // processCommand(F("ramp_to x230"));
  // processCommand(F("start x1"));
  // processCommand(F("wait x15000"));
  // processCommand(F("ramp_to x0.1"));
  // processCommand(F("wait x2000"));
  // processCommand(F("stop x"));

  processCommand(F("wait_for_motor x"));
  // processCommand(F("print_queue x"));

  // processCommand(F("dir x0"));
  // processCommand(F("move_rot x1"));
  // processCommand(F("wait_for_motor x"));

  // empty tank
  do_move_up(false);
  do_empty_tank();

  beep(50);
}
MenuItemCallable item_start_washing("Washing cycle", &do_start_washing, false);


// main menu
// MenuItem* main_menu_items[] = {
//   // &item_home_z_up,
//   &item_start_washing,
//   &item_move_up,
//   &item_move_down,
//   &item_fill_tank,
//   &item_empty_tank,
//   &item_home_z_down,
//   &item_fill_and_empty,
//   // &item_test_m400,
//   // &motor_x,
//   // &motor_z,
//   &item_valve_on_off,
//   &item_water_pump_on_off,
//   &item_watch_water_level_on_off,
// };
// Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);


#endif
