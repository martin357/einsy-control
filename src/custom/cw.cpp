#include "cw.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#include "../../serial.h"
#ifdef CUSTOM_CW


/// custom stuff
bool watch_water_level = false;


void setupCustom(){
  for (size_t i = 0; i < MOTORS_MAX; i++) {
    motors[i].driver.rms_current(500);
    motors[i].driver.sgt(2);

    // motors[i].driver.en_pwm_mode(0);
    // motors[i].driver.pwm_autoscale(0);
    // motors[i].driver.intpol(0);
    motors[i].driver.en_pwm_mode(1);
    motors[i].driver.pwm_autoscale(1);
    motors[i].driver.intpol(1);

    motors[i].rpm(120);
  }
  motors[0].inactivity_timeout = 5000;


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

bool is_uv_led_on(){ return !digitalReadExt(PIN_UV_LED); }
void do_uv_led_on(){ digitalWriteExt(PIN_UV_LED, LOW); }
void do_uv_led_off(){ digitalWriteExt(PIN_UV_LED, HIGH); }
MenuItemToggleCallable item_uv_led_on_off(&is_uv_led_on, "UV Led: on", "UV Led: off", &do_uv_led_off, &do_uv_led_on);

bool is_heater_on(){ return !digitalReadExt(PIN_HEATER); }
void do_heater_on(){ digitalWriteExt(PIN_HEATER, LOW); }
void do_heater_off(){ digitalWriteExt(PIN_HEATER, HIGH); }
MenuItemToggleCallable item_heater_on_off(&is_heater_on, "Heater: on", "Heater: off", &do_heater_off, &do_heater_on);

bool is_watch_water_level_on(){ return watch_water_level; }
void do_watch_water_level_on(){ watch_water_level = true; }
void do_watch_water_level_off(){ watch_water_level = false; digitalWriteExt(PIN_WATER_PUMP, LOW); digitalWriteExt(PIN_VALVE, LOW); }
MenuItemToggleCallable item_watch_water_level_on_off(&is_watch_water_level_on, "Water level: on", "Water level: off", &do_watch_water_level_off, &do_watch_water_level_on);


uint8_t washing_duration = 4;
uint8_t drying_duration = 3;
uint8_t curing_duration = 3;
// uint8_t washing_duration = 1;
// uint8_t drying_duration = 1;
// uint8_t curing_duration = 1;

MenuRange<uint8_t> menu_washing_duration("Washing time [min]", washing_duration, 1, 255);
MenuItem item_washing_duration("Washing time", &menu_washing_duration);

MenuRange<uint8_t> menu_drying_duration("Drying time [min]", drying_duration, 1, 255);
MenuItem item_drying_duration("Drying time", &menu_drying_duration);

MenuRange<uint8_t> menu_curing_duration("Curing time [min]", curing_duration, 1, 255);
MenuItem item_curing_duration("Curing time", &menu_curing_duration);


void beep_cycle_finished(bool show_on_lcd = true){
  if(show_on_lcd){
    lcd.clear();
    lcd.print("Finished!");
  }

  beep(40);
  delay(400);
  beep(40);
  delay(400);
  beep(40);
}

enum PlatformPos: uint8_t {NA, TOP, BOTTOM};

bool is_homed = false;
PlatformPos platform_pos = PlatformPos::NA;
void do_home_z_up(){ processCommand(F("home z0")); }
MenuItemCallable item_home_z_up("Home Z up", &do_home_z_up, false);

void do_home_z_down(){
  lcd.clear();
  lcd.print("Homing platform...");

  motors[2].driver.en_pwm_mode(0);
  motors[2].driver.pwm_autoscale(0);
  motors[2].driver.intpol(0);

  processCommand(F("home z1 b0.5 f180 g60"));
  processCommand(F("dir z0"));
  processCommand(F("rpm z180"));
  // processCommand(F("move_rot z0.2"));
  processCommand(F("move_rot z0.5"));

  motors[2].driver.en_pwm_mode(1);
  motors[2].driver.pwm_autoscale(1);
  motors[2].driver.intpol(1);

  is_homed = true;
  platform_pos = PlatformPos::BOTTOM;
}
MenuItemCallable item_home_z_down("Home Z down", &do_home_z_down, false);

void ensure_homed(){
  // motors[2].driver.en_pwm_mode(1);
  // motors[2].driver.pwm_autoscale(1);
  // motors[2].driver.intpol(1);

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

void do_move_up(bool, bool);
void do_move_down(bool, bool);

void do_fill_tank(bool do_beep = true){
  const uint16_t stabilization_duration = 5000; // 3000;
  const uint32_t start_time = millis();
  uint32_t stabilization_start;
  do_move_down(false, false);
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

    if(enc_click > 1){
      digitalWriteExt(PIN_WATER_PUMP, 0);
      digitalWriteExt(PIN_VALVE, 0);
      return;
    }
  }

  while(millis() < stabilization_start + stabilization_duration){
    const bool water_detected = digitalReadExt(PIN_CAP_IN);
    digitalWriteExt(PIN_VALVE, water_detected);
    digitalWriteExt(PIN_WATER_PUMP, !water_detected);
    delay(200);

    if(enc_click > 1){
      digitalWriteExt(PIN_WATER_PUMP, 0);
      digitalWriteExt(PIN_VALVE, 0);
      return;
    }
  }
  digitalWriteExt(PIN_WATER_PUMP, 0);
  digitalWriteExt(PIN_VALVE, 1);
  delay(500);
  digitalWriteExt(PIN_VALVE, 0);
  const uint32_t duration = millis() - start_time;
  lcd.clear();
  lcd.print("Finished!");
  if(do_beep){
    lcd.setCursor(0, 2);
    lcd.print("In ");
    lcd.print((uint16_t)(duration / 1000));
    lcd.print("s");
    beep(50);
    delay(2000);
  }
}
MenuItemCallable item_fill_tank("Fill tank", &do_fill_tank, false);


void do_empty_tank(bool do_beep = true){
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
    delay(50);
    if(enc_click > 1) break;
  }
  digitalWriteExt(PIN_VALVE, 0);
  lcd.clear();
  lcd.print("Tank is empty");
  if(do_beep){
    beep(50);
    delay(2000);
  }
}
MenuItemCallable item_empty_tank("Empty tank", &do_empty_tank, false);

void do_fill_and_empty(){
  do_fill_tank(false);
  do_empty_tank();
}
MenuItemCallable item_fill_and_empty("Fill & empty", &do_fill_and_empty, false);


void do_move_up(bool do_wait = true, bool do_beep = true){
  if(platform_pos == PlatformPos::TOP) return;

  ensure_homed();
  if(do_wait){
    lcd.clear();
    lcd.print("Moving platform up");
  }
  processCommand(F("dir z0"));
  processCommand(F("move_ramp s0.1 f300 a150 d150 z42"));
  if(do_wait){
    processCommand(F("wait_for_motor z"));
    if(do_beep) beep(50);
  }
  platform_pos = PlatformPos::TOP;
}
void _do_move_up(){ do_move_up(); }
MenuItemCallable item_move_up("Move platform up", &_do_move_up, false);

void do_move_down(bool do_wait = true, bool do_beep = true){
  if(!is_homed){
    // Serial.println(F("[DO_MOVE_DOWN] is not homed..."));
    do_home_z_down();
    return;
  }

  if(platform_pos == PlatformPos::BOTTOM) return;

  if(do_wait){
    // Serial.println(F("[DO_MOVE_DOWN] do_wait is true, clear lcd"));
    lcd.clear();
    lcd.print("Moving platform down");
  }
  // Serial.println(F("[DO_MOVE_DOWN] move down!"));
  processCommand(F("dir z1"));
  processCommand(F("move_ramp s0.1 f300 a150 d100 z42"));
  if(do_wait){
    processCommand(F("wait_for_motor z"));
    if(do_beep) beep(50);
  }
  platform_pos = PlatformPos::BOTTOM;
}
void _do_move_down(){ do_move_down(); }
MenuItemCallable item_move_down("Move platform down", &_do_move_down, false);


void do_start_washing(bool do_beep = true){
  uint32_t duration_half = ((uint32_t)washing_duration * 500UL * 60UL) - 3000UL;
  char duration_buf[64] = "wait x";
  ultoa(duration_half, &duration_buf[6], 10);

  // move down and fill tank
  processCommand(F("empty_queue x z"));
  // Serial.println(F("[WASH] move down"));
  // do_move_down(false);
  // processCommand(F("start z"));

  Serial.println(F("[WASH] fill tank"));
  do_fill_tank(false);

  // wash model for some time...
  Serial.println(F("[WASH] start"));
  lcd.clear();
  lcd.print("Washing...");

  motors[0].stop_on_stallguard = false;
  // motors[0].print_stallguard_to_serial = true;

  processCommand(F("stop_on_stallguard x0"));
  processCommand(F("dir x1"));
  processCommand(F("rpm x0.1"));
  processCommand(F("ramp_to x333"));
  processCommand(F("start x1"));
  // processCommand(F("wait x5000")); // x15000
  processCommand(duration_buf);
  processCommand(F("ramp_to x0.1"));
  processCommand(F("wait x3000"));
  // processCommand(F("start x1"));
  processCommand(F("stop x"));

  // processCommand(F("print_queue x"));
  processCommand(F("wait_for_motor x"));
  // processCommand(F("empty_queue x"));
  // beep(10);

  processCommand(F("dir x0"));
  processCommand(F("rpm x0.1"));
  processCommand(F("ramp_to x333"));
  processCommand(F("start x1"));
  // processCommand(F("wait x5000")); // x15000
  processCommand(duration_buf);
  processCommand(F("ramp_to x0.1"));
  processCommand(F("wait x3000"));
  // processCommand(F("start x1"));
  processCommand(F("stop x"));

  processCommand(F("wait_for_motor x"));
  // beep(10);

  // empty tank
  do_move_up(false, false);
  do_empty_tank(false);

  if(do_beep) beep_cycle_finished();

  // motors[0].stop_on_stallguard = true;
  motors[0].print_stallguard_to_serial = false;

}
MenuItemCallable item_start_washing("Start washing", &do_start_washing, false);


void do_start_drying(bool do_beep = true){
  const float cycle_rots = 42.0;
  const float rpm = cycle_rots * 2; // 84.0;
  const uint16_t sps = motors[2].rpm2sps(rpm);
  const uint32_t cycle_steps = motors[2].rot2usteps(cycle_rots);
  const uint32_t cycle_duration_ms = (float)cycle_steps / sps * 1000.0;
  const float cycle_duration_min = cycle_rots / rpm;
  const float cycles_per_min = 1.0 / cycle_duration_min;
  const uint8_t cycles_total = ceil(drying_duration * cycles_per_min);

  char do_steps_buf[64] = "do_steps z";
  ultoa(cycle_steps, &do_steps_buf[10], 10);

  do_heater_on();
  do_move_up(true, false);

  // motors[2].driver.en_pwm_mode(1);
  // motors[2].driver.pwm_autoscale(1);
  // motors[2].driver.intpol(1);

  motors[2].rpm(rpm);
  motors[2].dir(false); // false = up

  lcd.clear();
  lcd.print("Drying...");
  processCommand(F("empty_queue z"));
  Serial.println(F("[DRYING] Start"));

  for (size_t i = 0; i < cycles_total; i++) {
    lcd.clear();
    lcd.print("Drying cycle");
    lcd.setCursor(0, 1);
    lcd.print(i);
    lcd.print("/");
    lcd.print(cycles_total);

    // Serial.print(F("[DRYING] Cycle "));
    // Serial.print(i);
    // Serial.print("/");
    // Serial.println(cycles_total);

    // Serial.print(F("[DRYING] Now going: "));
    // Serial.println(motors[2].dir() ? "up" : "down");

    motors[2].dir(!motors[2].dir());
    // processCommand(F("empty_queue z"));
    processCommand(do_steps_buf);
    // processCommand(F("print_queue z"));
    processCommand(F("wait_for_motor z"));
    // beep(10);

  }

  // motors[2].driver.en_pwm_mode(0);
  // motors[2].driver.pwm_autoscale(0);
  // motors[2].driver.intpol(0);

  // Serial.print(F("[DRYING] Ended at: "));
  // Serial.println(motors[2].dir() ? "bottom" : "top");

  do_heater_off();

  if(motors[2].dir()){
    do_move_up(true, false); // move up since we ended at bottom
  }

  // is_homed = false; // force homing in next run
  if(do_beep) beep_cycle_finished();

}
MenuItemCallable item_start_drying("Start drying", &do_start_drying, false);


void do_start_curing(bool do_beep = true){
  const uint32_t duration = curing_duration * 60000;
  int32_t remaining;
  lcd.clear();
  lcd.print("Curing...");

  do_move_up(true, false);
  do_uv_led_on();

  const uint32_t curing_end = millis() + duration;
  while((remaining = curing_end - millis()) > 100){
    lcd.setCursor(0, 2);
    lcd.print("Remaining ");
    lcd.print((uint16_t)(remaining / 1000));
    lcd.print("s ");
    delay(50);

    if(enc_click > 1) break;
  }

  do_uv_led_off();

  if(do_beep) beep_cycle_finished();

}
MenuItemCallable item_start_curing("Start curing", &do_start_curing, false);


void do_wash_dry_cure(){
  do_start_washing(false);
  do_start_drying(false);
  do_start_curing(false);

  beep_cycle_finished();
}
MenuItemCallable item_wash_dry_cure("Wash, dry & cure", &do_wash_dry_cure, false);


// main menu
MenuItem* main_menu_items[] = {
  // &item_home_z_up,
  &item_wash_dry_cure,
  &item_start_washing,
  &item_start_drying,
  &item_start_curing,
  &item_washing_duration,
  &item_drying_duration,
  &item_curing_duration,
  &item_move_up,
  &item_move_down,
  &item_fill_tank,
  &item_empty_tank,
  // &item_home_z_down,
  // &item_test_m400,
  // &item_uv_led_on_off,
  // &item_heater_on_off,
  // &item_valve_on_off,
  // &item_water_pump_on_off,
  // &item_watch_water_level_on_off,
  &item_fill_and_empty,
  // &motor_x,
  // &motor_z,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);


#endif
