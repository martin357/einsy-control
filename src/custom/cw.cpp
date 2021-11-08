#include "cw.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#include "../../serial.h"
#ifdef CUSTOM_CW


/// custom stuff
bool watch_water_level = false;
bool prev_critical_water_detected = false;
uint32_t critical_water_level_reached_at = 0;
bool preventing_water_overflow = false;
const uint16_t preventive_action_delay = 600;

void loopCustom(){
  uint32_t _millis = millis();

  static uint32_t last_watch_water_level = 0;
  if(watch_water_level && _millis > last_watch_water_level + 100){
    bool water_detected = digitalReadExt(PIN_CAP_IN_PRIMARY);
    digitalWriteExt(PIN_WATER_PUMP, !water_detected);
    digitalWriteExt(PIN_VALVE, water_detected);
    last_watch_water_level = _millis;
  }

  const bool critical_water_detected = digitalReadExt(PIN_CAP_IN_CRITICAL);
  if(storage.watch_for_critical_level){
    if(critical_water_detected != prev_critical_water_detected){
      prev_critical_water_detected = critical_water_detected;
      if(critical_water_detected){
        // digitalWriteExt(PIN_VALVE, HIGH);
        // digitalWriteExt(BEEPER, HIGH);
        Serial.println(F("Critical water level detected!"));
        critical_water_level_reached_at = _millis;

      }else{
        // digitalWriteExt(PIN_VALVE, LOW);
        // digitalWriteExt(BEEPER, LOW);
        Serial.print(F("Safe water level reached in "));
        Serial.print(_millis - critical_water_level_reached_at);
        Serial.println(F("ms"));

        if(preventing_water_overflow){
          digitalWriteExt(PIN_VALVE, LOW);
          // digitalWriteExt(BEEPER, LOW);
          preventing_water_overflow = false;
          Serial.println(F("Preventive action stopped."));

        }

      }
    }

    if(critical_water_detected && critical_water_level_reached_at > 0 && _millis - critical_water_level_reached_at > preventive_action_delay){
      digitalWriteExt(PIN_VALVE, HIGH);
      // digitalWriteExt(BEEPER, HIGH);
      digitalWriteExt(PIN_WATER_PUMP, LOW);
      critical_water_level_reached_at = 0;
      preventing_water_overflow = true;
      Serial.println(F("Critical water level detected, preventive action taken!"));

    }

  }

}


void _delay(uint32_t ms){
  const uint32_t finish = millis() + ms;
  while(millis() < finish){
    loopCustom();
    delay(1);
  }
}


class MenuListModelHeight: public Menu{
public:
  MenuListModelHeight();
  void on_enter();
  void on_press(uint16_t);
  void draw(bool = true);
  void move(int8_t);
  const uint8_t items_count;
  uint8_t value;
};

MenuListModelHeight::MenuListModelHeight():
  Menu(nullptr, 0),
  value(0),
  items_count(4){}

void MenuListModelHeight::on_enter(){
  value = storage.model_height;
  lcd.clear();
}

void MenuListModelHeight::on_press(uint16_t duration){
  storage.model_height = value;
  storage.save();
  go_back();
}

void MenuListModelHeight::draw(bool clear){
  const char *items[] = {"Full", "3/4", "2/4", "1/4"};
  lcd.print("\3", 0, 0);
  lcd.print("Model height");
  lcd.print(" \1");

  lcd.print(value > 0 ? "<" : " ", 0, 2);
  lcd.setCursor(8, 2);
  lcd.print(items[value]);
  lcd.print(" ");
  lcd.print(value < items_count - 1 ? ">" : " ", 19, 2);
}

void MenuListModelHeight::move(int8_t amount){
  if((int8_t)value + amount < 0) value = 0;
  else if(value + amount >= items_count) value = items_count - 1;
  else value += amount;

  draw();
}


// custom CW stuff
bool is_valve_on(){ return digitalReadExt(PIN_VALVE); }
void do_valve_on(){ digitalWriteExt(PIN_VALVE, HIGH); }
void do_valve_off(){ digitalWriteExt(PIN_VALVE, LOW); }
const char pgmstr_valve_on[] PROGMEM = "Valve: on";
const char pgmstr_valve_off[] PROGMEM = "Valve: off";
MenuItemToggleCallable item_valve_on_off(&is_valve_on, pgmstr_valve_on, pgmstr_valve_off, &do_valve_off, &do_valve_on);

bool is_water_pump_on(){ return digitalReadExt(PIN_WATER_PUMP); }
void do_water_pump_on(){ digitalWriteExt(PIN_WATER_PUMP, HIGH); }
void do_water_pump_off(){ digitalWriteExt(PIN_WATER_PUMP, LOW); }
const char pgmstr_water_pump_on[] PROGMEM = "Water pump: on";
const char pgmstr_water_pump_off[] PROGMEM = "Water pump: off";
MenuItemToggleCallable item_water_pump_on_off(&is_water_pump_on, pgmstr_water_pump_on, pgmstr_water_pump_off, &do_water_pump_off, &do_water_pump_on);

bool is_uv_led_on(){ return !digitalReadExt(PIN_UV_LED); }
void do_uv_led_on(){ digitalWriteExt(PIN_UV_LED, LOW); }
void do_uv_led_off(){ digitalWriteExt(PIN_UV_LED, HIGH); }
const char pgmstr_uv_led_on[] PROGMEM = "UV Led: on";
const char pgmstr_uv_led_off[] PROGMEM = "UV Led: off";
MenuItemToggleCallable item_uv_led_on_off(&is_uv_led_on, pgmstr_uv_led_on, pgmstr_uv_led_off, &do_uv_led_off, &do_uv_led_on);

bool is_heater_on(){ return !digitalReadExt(PIN_HEATER); }
void do_heater_on(){ digitalWriteExt(PIN_HEATER, LOW); }
void do_heater_off(){ digitalWriteExt(PIN_HEATER, HIGH); }
const char pgmstr_heater_on[] PROGMEM = "Heater: on";
const char pgmstr_heater_off[] PROGMEM = "Heater: off";
MenuItemToggleCallable item_heater_on_off(&is_heater_on, pgmstr_heater_on, pgmstr_heater_off, &do_heater_off, &do_heater_on);

bool is_watch_water_level_on(){ return watch_water_level; }
void do_watch_water_level_on(){ watch_water_level = true; }
void do_watch_water_level_off(){ watch_water_level = false; digitalWriteExt(PIN_WATER_PUMP, LOW); digitalWriteExt(PIN_VALVE, LOW); }
const char pgmstr_watch_water_level_on[] PROGMEM = "Water level: on";
const char pgmstr_watch_water_level_off[] PROGMEM = "Water level: off";
MenuItemToggleCallable item_watch_water_level_on_off(&is_watch_water_level_on, pgmstr_watch_water_level_on,
  pgmstr_watch_water_level_off, &do_watch_water_level_off, &do_watch_water_level_on);



MenuRange<uint8_t> menu_washing_duration("Washing time [min]", storage.washing_duration, 1, 255, true);
const char pgmstr_washing_time[] PROGMEM = "Washing time";
MenuItem item_washing_duration(pgmstr_washing_time, &menu_washing_duration);

MenuRange<uint8_t> menu_drying_duration("Drying time [min]", storage.drying_duration, 1, 255, true);
const char pgmstr_drying_time[] PROGMEM = "Drying time";
MenuItem item_drying_duration(pgmstr_drying_time, &menu_drying_duration);

MenuRange<uint8_t> menu_curing_duration("Curing time [min]", storage.curing_duration, 1, 255, true);
const char pgmstr_curing_time[] PROGMEM = "Curing time";
MenuItem item_curing_duration(pgmstr_curing_time, &menu_curing_duration);

MenuRange<uint8_t> menu_stabilization_duration("Stabil. time [s]", storage.stabilization_duration, 0, 60, true);
const char pgmstr_stabilization_duration[] PROGMEM = "Stabilization time";
MenuItem item_stabilization_duration(pgmstr_stabilization_duration, &menu_stabilization_duration);

MenuListModelHeight menu_model_height;
const char pgmstr_model_height[] PROGMEM = "Model height";
MenuItem item_model_height(pgmstr_model_height, &menu_model_height);


void turn_water_level_sensors(bool value){
  value = true; // hack, keep sensors always on
  digitalWriteExt(PIN_CAP_OUT_PRIMARY, value);
  digitalWriteExt(PIN_CAP_OUT_CRITICAL, value);
  if(value){
    pinModeInput(PIN_CAP_IN_PRIMARY);
    pinModeInput(PIN_CAP_IN_CRITICAL);
  }else{
    pinModeOutput(PIN_CAP_IN_PRIMARY);
    pinModeOutput(PIN_CAP_IN_CRITICAL);
    digitalWriteExt(PIN_CAP_IN_PRIMARY, LOW);
    digitalWriteExt(PIN_CAP_IN_CRITICAL, LOW);
  }
}


bool is_water_level_sensors_on(){ return digitalReadExt(PIN_CAP_OUT_PRIMARY); }
void do_water_level_sensors_on(){ turn_water_level_sensors(true); }
void do_water_level_sensors_off(){ turn_water_level_sensors(false); }
const char pgmstr_water_level_sensors_on[] PROGMEM = "W.Lvl sensors: on";
const char pgmstr_water_level_sensors_off[] PROGMEM = "W.Lvl sensors: off";
MenuItemToggleCallable item_water_level_sensors_on_off(&is_water_level_sensors_on, pgmstr_water_level_sensors_on,
  pgmstr_water_level_sensors_off, &do_water_level_sensors_off, &do_water_level_sensors_on);



MenuItemToggle item_watch_for_critical_level(&storage.watch_for_critical_level, "Watch critical: on", "Watch critical: off", true);


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
    motors[i].driver.TCOOLTHRS(800);
  }
  motors[0].inactivity_timeout = 5000;

  motors[2].driver.rms_current(600);
  motors[2].driver.sgt(3);

  // touch capacitance sensor
  pinModeOutput(PIN_CAP_OUT_PRIMARY);
  pinModeOutput(PIN_CAP_OUT_CRITICAL);
  turn_water_level_sensors(false);

  pinMode(PIN_CAP_IN_PRIMARY, INPUT_PULLUP);
  pinMode(PIN_CAP_IN_CRITICAL, INPUT_PULLUP);

  // CW board trigger pin - UV led
  pinModeOutput(PIN_UV_LED);
  digitalWriteExt(PIN_UV_LED, LOW);
  delay(10);
  digitalWriteExt(PIN_UV_LED, HIGH);

  // CW board trigger pin - heater
  pinModeOutput(PIN_HEATER);
  digitalWriteExt(PIN_HEATER, LOW);
  delay(10);
  digitalWriteExt(PIN_HEATER, HIGH);

  // power output
  pinModeOutput(PIN_VALVE); // bed mosfet
  digitalWriteExt(PIN_VALVE, LOW);
  pinModeOutput(PIN_WATER_PUMP); // extruder mosfet
  digitalWriteExt(PIN_WATER_PUMP, LOW);

  menu_model_height.value = storage.model_height;

}


void beep_cycle_finished(bool show_on_lcd = true){
  if(show_on_lcd){
    lcd.clear();
    lcd.print("Finished!");
  }

  beep(40);
  _delay(400);
  beep(40);
  _delay(400);
  beep(40);
}

enum PlatformPos: uint8_t {NA, TOP, BOTTOM};

bool is_homed = false;
PlatformPos platform_pos = PlatformPos::NA;
void do_home_z_up(){
  lcd.clear();
  lcd.print("Homing platform...");

  motors[2].driver.en_pwm_mode(0);
  motors[2].driver.pwm_autoscale(0);
  motors[2].driver.intpol(0);

  processCommand(F("home z0 b0.5 f130 g130"));
  processCommand(F("dir z0"));
  processCommand(F("rpm z200"));
  processCommand(F("move_rot z0.5"));

  motors[2].driver.en_pwm_mode(1);
  motors[2].driver.pwm_autoscale(1);
  motors[2].driver.intpol(1);

  is_homed = true;
  platform_pos = PlatformPos::TOP;
}
const char pgmstr_home_z_up[] PROGMEM = "Home Z up";
MenuItemCallable item_home_z_up(pgmstr_home_z_up, &do_home_z_up, false);

void do_home_z_down(){
  lcd.clear();
  lcd.print("Homing platform...");

  motors[2].driver.en_pwm_mode(0);
  motors[2].driver.pwm_autoscale(0);
  motors[2].driver.intpol(0);

  // processCommand(F("home z1 b0.5 f180 g60"));
  processCommand(F("home z1 b0.5 f130")); // g130
  processCommand(F("dir z0"));
  processCommand(F("rpm z200"));
  // processCommand(F("move_rot z0.2"));
  processCommand(F("move_rot z0.5"));

  motors[2].driver.en_pwm_mode(1);
  motors[2].driver.pwm_autoscale(1);
  motors[2].driver.intpol(1);

  is_homed = true;
  platform_pos = PlatformPos::BOTTOM;
}
const char pgmstr_home_z_down[] PROGMEM = "Home Z down";
MenuItemCallable item_home_z_down(pgmstr_home_z_down, &do_home_z_down, false);

void ensure_homed(){
  // motors[2].driver.en_pwm_mode(1);
  // motors[2].driver.pwm_autoscale(1);
  // motors[2].driver.intpol(1);

  if(!is_homed){
    do_home_z_down();
    processCommand(F("wait_for_motor z"));
  }
}


void do_move_up(bool, bool);
void do_move_down(bool, bool);

void do_fill_tank(bool do_beep = true){
  const uint16_t stabilization_duration_ms = storage.stabilization_duration * 1000;
  const uint32_t start_time = millis();
  uint32_t stabilization_start;
  do_move_down(false, false);
  lcd.clear();
  lcd.print("Filling tank...");
  digitalWriteExt(PIN_VALVE, 0);
  digitalWriteExt(PIN_WATER_PUMP, 1);

  turn_water_level_sensors(true);

  bool prev_water_detected = false;
  bool water_detected;
  uint32_t water_detected_at = 0;
  while(1){
    const uint32_t _millis = millis();
    water_detected = digitalReadExt(PIN_CAP_IN_PRIMARY);
    if(water_detected != prev_water_detected){
      prev_water_detected = water_detected;
      if(water_detected){
        Serial.println(F("water level detected!"));
        water_detected_at = _millis;
        // digitalWriteExt(BEEPER, 1);

      }else{
        Serial.print(F("signal lost in "));
        Serial.print(_millis - water_detected_at);
        Serial.println(F("ms"));
        // digitalWriteExt(BEEPER, 0);

      }
    }

    if(water_detected && water_detected_at > 0 && _millis - water_detected_at > 150){
      digitalWriteExt(PIN_WATER_PUMP, 0);
      digitalWriteExt(PIN_VALVE, 1);
      lcd.clear();
      lcd.print("Stabilizing...");
      stabilization_start = millis();

      Serial.println(F("Stable signal reached, stabilise level now"));
      // digitalWriteExt(BEEPER, 0);
      break;

    // }else{
    //   _delay(50);
    }

    if(enc_click > 1){
      enc_click = 0;
      digitalWriteExt(PIN_WATER_PUMP, 0);
      digitalWriteExt(PIN_VALVE, 0);
      return;
    }
  }

  while(millis() < stabilization_start + stabilization_duration_ms){
    const bool water_detected = digitalReadExt(PIN_CAP_IN_PRIMARY);
    digitalWriteExt(PIN_VALVE, water_detected);
    digitalWriteExt(PIN_WATER_PUMP, !water_detected);
    _delay(200);

    if(enc_click > 1){
      enc_click = 0;
      digitalWriteExt(PIN_WATER_PUMP, 0);
      digitalWriteExt(PIN_VALVE, 0);
      return;
    }
  }
  digitalWriteExt(PIN_WATER_PUMP, 0);
  digitalWriteExt(PIN_VALVE, 1);
  // beep(30);
  _delay(700); // 1300 // 700
  // _delay(500);
  // beep(30);
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
    _delay(2000);
  }

  turn_water_level_sensors(false);
}
const char pgmstr_fill_tank[] PROGMEM = "Fill tank";
MenuItemCallable item_fill_tank(pgmstr_fill_tank, &do_fill_tank, false);


void do_empty_tank(bool do_beep = true){
  const uint32_t emptying_duration = 73UL * 1000UL;
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
    if(enc_click > 1){
      enc_click = 0;
      break;
    }
  }
  digitalWriteExt(PIN_VALVE, 0);
  lcd.clear();
  lcd.print("Tank is empty");
  if(do_beep){
    beep(50);
    _delay(2000);
  }
}
const char pgmstr_empty_tank[] PROGMEM = "Empty tank";
MenuItemCallable item_empty_tank(pgmstr_empty_tank, &do_empty_tank, false);

void do_fill_and_empty(){
  do_fill_tank(false);
  do_empty_tank();
}
const char pgmstr_fill_and_empty[] PROGMEM = "Fill & empty";
MenuItemCallable item_fill_and_empty(pgmstr_fill_and_empty, &do_fill_and_empty, false);


void do_move_up(bool do_wait = true, bool do_beep = true){
  if(platform_pos == PlatformPos::TOP) return;

  do{
    ensure_homed();
    if(do_wait){
      lcd.clear();
      lcd.print("Moving platform up");
    }
    motors[2].stallguard_triggered = false;
    motors[2].stop_on_stallguard = true;
    processCommand(F("dir z0"));
    processCommand(F("move_ramp s60 f333 a250 d250 z-42"));
    processCommand(F("start z"));
    // processCommand(F("print_info z"));
    if(do_wait){
      delay(10);
      processCommand(F("wait_for_motor z"));
      if(do_beep) beep(50);
    }
    if(motors[2].stallguard_triggered){
      Serial.println(F("STALLGUARD TRIGGERED DURING MOVEMENT"));
      is_homed = false;
    }

  }while(motors[2].stallguard_triggered);
  platform_pos = PlatformPos::TOP;
}
void _do_move_up(){ do_move_up(); }
const char pgmstr_move_up[] PROGMEM = "Move platform up";
MenuItemCallable item_move_up(pgmstr_move_up, &_do_move_up, false);

// void do_move_down(bool do_wait = true, bool do_beep = true){
//   if(!is_homed){
//     // Serial.println(F("[DO_MOVE_DOWN] is not homed..."));
//     do_home_z_down();
//     return;
//   }
//
//   if(platform_pos == PlatformPos::BOTTOM) return;
//
//   if(do_wait){
//     // Serial.println(F("[DO_MOVE_DOWN] do_wait is true, clear lcd"));
//     lcd.clear();
//     lcd.print("Moving platform down");
//   }
//   // Serial.println(F("[DO_MOVE_DOWN] move down!"));
//   processCommand(F("dir z1"));
//   processCommand(F("move_ramp s60 f333 a250 d250 z42"));
//   processCommand(F("start z"));
//   if(do_wait){
//     processCommand(F("wait_for_motor z"));
//     if(do_beep) beep(50);
//   }
//   platform_pos = PlatformPos::BOTTOM;
// }
void do_move_down(bool do_wait = true, bool do_beep = true){
  do{
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
    motors[2].stallguard_triggered = false;
    motors[2].stop_on_stallguard = true;
    processCommand(F("dir z1"));
    processCommand(F("move_ramp s60 f333 a250 d250 z42"));
    processCommand(F("start z"));
    if(do_wait){
      delay(10);
      processCommand(F("wait_for_motor z"));
      if(do_beep) beep(50);
    }
    if(motors[2].stallguard_triggered){
      Serial.println(F("STALLGUARD TRIGGERED DURING MOVEMENT"));
      is_homed = false;
    }

  }while(motors[2].stallguard_triggered);
  platform_pos = PlatformPos::BOTTOM;
}
void _do_move_down(){ do_move_down(); }
const char pgmstr_move_down[] PROGMEM = "Move platform down";
MenuItemCallable item_move_down(pgmstr_move_down, &_do_move_down, false);


void do_start_washing(bool do_beep = true){
  uint32_t duration_half = ((uint32_t)storage.washing_duration * 500UL * 60UL) - 1500UL;
  char duration_buf[64] = "wait x";
  ultoa(duration_half, &duration_buf[6], 10);

  // move down and fill tank
  processCommand(F("empty_queue x z"));
  do_fill_tank(false); // off for debug

  // wash model for some time...
  lcd.clear();
  lcd.print("Washing... <<<");

  const bool old_stop_on_stallguard = motors[0].stop_on_stallguard;
  motors[0].stop_on_stallguard = false;
  // motors[0].print_stallguard_to_serial = true;

  // processCommand(F("stop_on_stallguard x0"));
  processCommand(F("dir x1"));
  // processCommand(F("rpm x60"));
  processCommand(F("rpm x0.1"));
  processCommand(F("ramp_to x333"));
  processCommand(F("start x1"));
  processCommand(duration_buf);
  processCommand(F("stop x"));
  processCommand(F("wait x3000"));
  // processCommand(F("wait_for_motor x"));
  int32_t remaining;
  const uint32_t washing_end = millis() + ((uint32_t)storage.washing_duration * 60000UL);
  while(motors[0].is_busy()){
    remaining = washing_end - millis();
    lcd.setCursor(0, 2);
    lcd.print("Remaining ");
    lcd.print((uint16_t)(remaining / 1000));
    lcd.print("s ");
    _delay(50);
    if(enc_click > 1){
      enc_click = 0;
      beep(50);
      processCommand(F("stop x"));
    }
  }
  // processCommand(F("empty_queue x"));
  // beep(30);

  lcd.setCursor(0, 0);
  lcd.print("Washing... >>>");
  processCommand(F("dir x0"));
  // processCommand(F("rpm x60"));
  processCommand(F("rpm x0.1"));
  processCommand(F("ramp_to x333"));
  processCommand(F("start x1"));
  processCommand(duration_buf);
  processCommand(F("stop x"));
  // processCommand(F("wait x3000"));
  // processCommand(F("wait_for_motor x"));
  while(motors[0].is_busy()){
    remaining = washing_end - millis();
    lcd.setCursor(0, 2);
    lcd.print("Remaining ");
    lcd.print((uint16_t)(remaining / 1000));
    lcd.print("s ");
    _delay(50);
    if(enc_click > 1){
      enc_click = 0;
      beep(50);
      processCommand(F("stop x"));
    }
  }

  // empty tank
  do_move_up(false, false); // off for debug
  do_empty_tank(false); // off for debug

  if(do_beep) beep_cycle_finished();

  motors[0].stop_on_stallguard = old_stop_on_stallguard;
  motors[0].print_stallguard_to_serial = false;

}
const char pgmstr_start_washing[] PROGMEM = "Start washing";
MenuItemCallable item_start_washing(pgmstr_start_washing, &do_start_washing, false);


void do_start_drying(bool do_beep = true){
  const float full_cycle_rots = 38.0; // 42.0; // 1rot = 4mm
  float cycle_rots = full_cycle_rots;
  switch (menu_model_height.value) {
    case 1: { cycle_rots = full_cycle_rots*0.80; break; } // 3/4
    case 2: { cycle_rots = full_cycle_rots*0.60; break; } // half
    case 3: { cycle_rots = full_cycle_rots*0.30; break; } // 1/4
  }

  // const float rpm = cycle_rots * 2; // 84.0;
  const float rpm = cycle_rots * 6; // 84.0;
  const float delta_rots = full_cycle_rots - cycle_rots;
  const uint16_t sps = motors[2].rpm2sps(rpm);
  const uint32_t cycle_steps = motors[2].rot2usteps(cycle_rots);
  const uint32_t delta_steps = motors[2].rot2usteps(delta_rots);
  const uint32_t cycle_duration_ms = (float)cycle_steps / sps * 1000.0;
  const float cycle_duration_min = cycle_rots / rpm;
  const float cycles_per_min = 1.0 / cycle_duration_min;
  const uint8_t cycles_total = ceil(storage.drying_duration * cycles_per_min);
  const uint32_t preheat_duration = 45 * 1000UL;
  uint32_t remaining;

  char do_steps_buf[20] = "do_steps z";
  ultoa(cycle_steps, &do_steps_buf[10], 10);

  processCommand(F("empty_queue z"));
  const uint32_t preheat_end = millis() + preheat_duration;
  do_heater_on();

  // do_move_up(false, false); // replaced by following code:
  ensure_homed();
  // char move_ramp_buf[40] = "move_ramp s120 f333 a250 d250 z";

  if(platform_pos == PlatformPos::TOP){
    if(delta_steps > 0){
      // dtostrf(delta_rots, -8, 2, &move_ramp_buf[31]);
      // for (size_t i = 31; i < sizeof(move_ramp_buf); i++) if(move_ramp_buf[i] == ' '){ move_ramp_buf[i] = 0; break; }
      // processCommand(F("dir z1"));
      // processCommand(move_ramp_buf);
      motors[2].plan_ramp_move(delta_rots, 120, 333, 250, 250);
      motors[2].start();
    }

  }else{
    // dtostrf(42.0 - delta_rots, -8, 2, &move_ramp_buf[31]);
    // for (size_t i = 31; i < sizeof(move_ramp_buf); i++) if(move_ramp_buf[i] == ' '){ move_ramp_buf[i] = 0; break; }
    // processCommand(F("dir z0"));
    // processCommand(move_ramp_buf);
    motors[2].plan_ramp_move(-(42.0 - delta_rots), 120, 333, 250, 250);
    motors[2].start();

  }

  lcd.clear();
  lcd.print("Preheating...");
  // while((remaining = preheat_end - millis()) > 100){
  while(preheat_end > millis()){
    remaining = preheat_end - millis();
    lcd.setCursor(0, 2);
    lcd.print("Remaining ");
    lcd.print((uint16_t)(remaining / 1000));
    lcd.print("s ");
    _delay(50);

    if(enc_click > 1){
      enc_click = 0;
      if(!motors[2].is_busy()) break;
    }
  }

  motors[2].rpm(rpm);
  motors[2].dir(false); // false = up
  motors[2].stop_on_stallguard = false;

  // motors[2].driver.en_pwm_mode(1);
  // motors[2].driver.pwm_autoscale(1);
  // motors[2].driver.intpol(1);


  lcd.clear();
  lcd.print("Drying...");
  processCommand(F("empty_queue z"));
  // Serial.println(F("[DRYING] Start"));
  const uint32_t drying_end = millis() + (60000UL * storage.drying_duration);

  for (size_t i = 0; i < cycles_total; i++) {
    lcd.clear();
    lcd.print("Drying ");
    switch (menu_model_height.value) {
      case 0: lcd.print("full"); break;
      case 1: lcd.print("3/4 of"); break;
      case 2: lcd.print("2/4 of"); break;
      case 3: lcd.print("1/4 of"); break;
    }
    lcd.print(" model");
    lcd.setCursor(0, 1);
    lcd.print("Cycle:");
    lcd.print(i+1);
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
    processCommand(F("start z"));
    // processCommand(F("wait_for_motor z"));
    while(motors[2].is_busy()){
      remaining = drying_end - millis();
      lcd.setCursor(0, 2);
      lcd.print("Remaining ");
      lcd.print((uint16_t)(remaining / 1000));
      lcd.print("s ");
      _delay(50);

      if(enc_click > 1){
        enc_click = 0;
        lcd.clear();
        lcd.print("Finishing cycle...");
        i = cycles_total + 1; // break from loop
        while(motors[2].is_busy()) _delay(50);
        break;
      }
    }
    // beep(10);

  }

  // motors[2].driver.en_pwm_mode(0);
  // motors[2].driver.pwm_autoscale(0);
  // motors[2].driver.intpol(0);

  // Serial.print(F("[DRYING] Ended at: "));
  // Serial.println(motors[2].dir() ? "bottom" : "top");

  lcd.clear();
  lcd.print("Finishing...");
  processCommand(F("empty_queue z"));
  Serial.println(F("[[]] Finishing"));

  do_heater_off();

  if(motors[2].dir()){
    // do_move_up(true, false); // move up since we ended at bottom
    Serial.println(F("[[]] Move bot -> top"));
    // processCommand(F("move_ramp s0.1 f300 a200 d200 z38"));
    // processCommand(F("start z"));
    processCommand(F("dir z0"));
    motors[2].plan_ramp_move(-38.0, 0.1, 333, 250, 250);
    motors[2].start();
    processCommand(F("wait_for_motor z"));
  }else{
    if(delta_steps){
      // dtostrf(delta_rots, -8, 2, &move_ramp_buf[31]);
      // for (size_t i = 31; i < sizeof(move_ramp_buf); i++) if(move_ramp_buf[i] == ' '){ move_ramp_buf[i] = 0; break; }
      // Serial.println(F("[[]] Move mid -> top"));
      // Serial.print(F("[[]] using >>>"));
      // Serial.print(move_ramp_buf);
      // Serial.println(F("<<<"));
      // processCommand(F("dir z0"));
      // processCommand(move_ramp_buf);
      // processCommand(F("start z"));
      motors[2].plan_ramp_move(-delta_rots, 0.1, 333, 250, 250);
      motors[2].start();
      processCommand(F("wait_for_motor z"));
    }
  }
  platform_pos = PlatformPos::TOP;

  // is_homed = false; // force homing in next run
  if(do_beep) beep_cycle_finished();

}
const char pgmstr_start_drying[] PROGMEM = "Start drying";
MenuItemCallable item_start_drying(pgmstr_start_drying, &do_start_drying, false);


void do_start_curing(bool do_beep = true){
  const uint32_t duration = storage.curing_duration * 60000;
  int32_t remaining;
  lcd.clear();
  lcd.print("Curing...");

  processCommand(F("empty_queue z"));
  do_move_up(true, false);
  do_uv_led_on();

  const uint32_t curing_end = millis() + duration;
  while((remaining = curing_end - millis()) > 100){
    lcd.setCursor(0, 2);
    lcd.print("Remaining ");
    lcd.print((uint16_t)(remaining / 1000));
    lcd.print("s ");
    _delay(50);

    if(enc_click > 1){
      enc_click = 0;
      break;
    }
  }

  do_uv_led_off();

  if(do_beep) beep_cycle_finished();

}
const char pgmstr_start_curing[] PROGMEM = "Start curing";
MenuItemCallable item_start_curing(pgmstr_start_curing, &do_start_curing, false);


void do_wash_dry_cure(){
  do_start_washing(false);
  do_start_drying(false);
  do_start_curing(false);

  beep_cycle_finished();
}
const char pgmstr_wash_dry_cure[] PROGMEM = "Wash, dry & cure";
MenuItemCallable item_wash_dry_cure(pgmstr_wash_dry_cure, &do_wash_dry_cure, false);


void do_wash_dry(){
  do_start_washing(false);
  do_start_drying(false);

  beep_cycle_finished();
}
const char pgmstr_wash_dry[] PROGMEM = "Wash & dry";
MenuItemCallable item_wash_dry(pgmstr_wash_dry, &do_wash_dry, false);


void do_up_and_down(){
  while(1){
    do_move_up(true, false);
    do_move_down(true, false);
  }
}
const char pgmstr_up_and_down[] PROGMEM = "Up & down";
MenuItemCallable item_up_and_down(pgmstr_up_and_down, &do_up_and_down, false);


// debug menu
MenuItem* const debug_menu_items[] PROGMEM = {
  &back,
  &item_up_and_down,
  // &item_home_z_up,
  &item_home_z_down,
  &item_uv_led_on_off,
  &item_heater_on_off,
  &item_valve_on_off,
  &item_water_pump_on_off,
  &item_watch_water_level_on_off,
  &item_water_level_sensors_on_off,
  &item_watch_for_critical_level,
  &motor_x,
  &motor_z,
};
Menu debug_menu(debug_menu_items, sizeof(debug_menu_items) / 2);
const char pgmstr_debug[] PROGMEM = "!!! Debug";
MenuItem item_debug_menu(pgmstr_debug, &debug_menu);


// cycle duration menu
MenuItem* const cycle_duration_menu_items[] PROGMEM = {
  &back,
  &item_washing_duration,
  &item_drying_duration,
  &item_curing_duration,
};
Menu cycle_duration_menu(cycle_duration_menu_items, sizeof(cycle_duration_menu_items) / 2);
const char pgmstr_cycle_duration[] PROGMEM = "Cycle duration";
MenuItem item_cycle_duration_menu(pgmstr_cycle_duration, &cycle_duration_menu);


// start cycle menu
MenuItem* const start_cycle_menu_items[] PROGMEM = {
  &back,
  &item_start_washing,
  &item_start_drying,
  &item_start_curing,
};
Menu start_cycle_menu(start_cycle_menu_items, sizeof(start_cycle_menu_items) / 2);
const char pgmstr_start_cycle[] PROGMEM = "Start cycle";
MenuItem item_start_cycle_menu(pgmstr_start_cycle, &start_cycle_menu);



// main menu
MenuItem* const main_menu_items[] PROGMEM = {
  &item_wash_dry_cure,
  &item_wash_dry,
  &item_model_height,
  &item_cycle_duration_menu,
  &item_start_cycle_menu,
  &item_move_up,
  &item_move_down,
  &item_fill_tank,
  &item_empty_tank,
  &item_fill_and_empty,
  &item_stabilization_duration,
  &item_debug_menu,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);


#endif
