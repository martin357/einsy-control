#include "twin_tower_test.h"
#include <Arduino.h>
#include <Wire.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#include "../../serial.h"
#ifdef CUSTOM_TWIN_TOWER_TEST

#include "../ad7150_registers.h"
#include "../ad7150.h"
#include "../kalmanfilter.h"

const uint8_t towers[] = {ML, MR};
uint32_t uptime = 0;

// ad7150_reg_setup ch_setup;
ad7150_reg_capdac ch1_capdac, ch2_capdac;
// ad7150_reg_configuration configuration;

int32_t last_autolevel_tick = 0;
bool autolevel_enabled = false;
uint32_t autolevel_on_at = 0;
float level_tolerance = 0.3; // 0.5; // 0.3;
float autolevel_target = 0.0;
uint32_t pump_off_time_remaining = 0;
uint32_t pump_off_time_decrease_last_tick = 0;
uint32_t pump_slowdown_at = 0;

float rpm_min = 80.0;
float rpm_optimal = 220.0;
float rpm_max = 260.0;
float new_rpm = 0.0001;
float total_dist = 0.0;

KalmanFilter filter(10, 1, 0.01);
float filtered = 0.0;
float filtered_corrected = 0.0;
int8_t capsense_reading_range = 0;
uint32_t last_capsense_reading_error_beep = 0;


void writeRegister(uint8_t reg, uint16_t value, bool wait = true);
uint8_t readRegister(uint8_t reg);

// bool print_homing_homed_busy = false;
// uint32_t last_print_hhb = 0;


bool get_level_ir_gate_status(){
  return !digitalReadExt(X_MIN_PIN);
}

void start_pump_auto_off_timer(){
  pump_off_time_remaining = DEFAULT_PUMP_AUTO_OFF_TIME;
  pump_off_time_decrease_last_tick = millis();
}

void stop_pump_auto_off_timer(){
  pump_off_time_remaining = 0;
}

void enable_pump_slowdown(){
  pump_slowdown_at = millis() + 180000L;
}

void disable_pump_slowdown(){
  pump_slowdown_at = 0;
}


void pump_start(bool dir, float target_rpm = 150){
  processCommand(F("halt x"));
  processCommand(F("empty_queue x"));
  processCommand(dir ? F("dir x0") : F("dir x1"));
  // processCommand(F("accel x150"));
  processCommand(F("rpm x0.01"));
  processCommand(F("start x1"));
  // processCommand(F("ramp_to x150"));
  motors[MP].ramp_to(target_rpm);
  start_pump_auto_off_timer();
  // if(dir) disable_pump_slowdown();
  // else enable_pump_slowdown();
}

void pump_stop(bool wait = true){
  stop_pump_auto_off_timer();
  disable_pump_slowdown();
  // processCommand(F("decel x150"));
  // processCommand(F("ramp_to x0.01"));
  processCommand(F("ramp_to x0"));
  processCommand(F("start x")); // TODO check if this is still needed??
  if(wait) processCommand(F("wait_for_motor x"));
}

#define PRINT_ANALOG(pin)  Serial.print("" # pin " = "); Serial.println(analogRead(pin));



const char pgmstr_filtered[] PROGMEM = "filtered";
MenuItemDynamic<float> item__filtered(pgmstr_filtered, filtered);
const char pgmstr_corrected[] PROGMEM = "corrected";
MenuItemDynamic<float> item__filtered_corrected(pgmstr_corrected, filtered_corrected);
const char pgmstr_zero_offset[] PROGMEM = "zero_offset";
MenuItemDynamic<float> item_storage__zero_offset(pgmstr_zero_offset, storage.zero_offset);
const char pgmstr_level_min[] PROGMEM = "level_min";
MenuItemDynamic<float> item_storage__level_min(pgmstr_level_min, storage.level_min);
const char pgmstr_level_optimal[] PROGMEM = "level_optimal";
MenuItemDynamic<float> item_storage__level_optimal(pgmstr_level_optimal, storage.level_optimal);
const char pgmstr_level_fill[] PROGMEM = "level_fill";
MenuItemDynamic<float> item_storage__level_fill(pgmstr_level_fill, storage.level_fill);
const char pgmstr_level_max[] PROGMEM = "level_max";
MenuItemDynamic<float> item_storage__level_max(pgmstr_level_max, storage.level_max);
const char pgmstr_tilt_max_tmp[] PROGMEM = "tilt max tmp";
MenuItemRange<float> item_storage__tilt_max_temperature(pgmstr_tilt_max_tmp, storage.tilt_max_temperature, 40, 100, 1, true);

const char pgmstr_A_L_target[] PROGMEM = "A.L. target";
MenuItemRange<float> item__autolevel_target(pgmstr_A_L_target, autolevel_target, -1000, 1000, 0.01f, false);

MenuItemDynamicTime item_pump_off_time(pgmstr_off_time, &pump_off_time_remaining); // pgmstr_off_time defined in menu_system_derivates.cpp

const char pgmstr_uptime[] PROGMEM = "Uptime";
MenuItemDynamicTime item_uptime(pgmstr_uptime, &uptime);



const char pgmstr__manual_edit_zero_offset[] PROGMEM = "zero_offset";
MenuItemRange<float> manual_edit_item_zero_offset(pgmstr__manual_edit_zero_offset, storage.zero_offset, -1000, 1000, 0.01f, true);

const char pgmstr__manual_edit_level_min[] PROGMEM = "level_min";
MenuItemRange<float> manual_edit_item_level_min(pgmstr__manual_edit_level_min, storage.level_min, -1000, 1000, 0.01f, true);

const char pgmstr__manual_edit_level_fill[] PROGMEM = "level_fill";
MenuItemRange<float> manual_edit_item_level_fill(pgmstr__manual_edit_level_fill, storage.level_fill, -1000, 1000, 0.01f, true);

const char pgmstr__manual_edit_level_optimal[] PROGMEM = "level_optimal";
MenuItemRange<float> manual_edit_item_level_optimal(pgmstr__manual_edit_level_optimal, storage.level_optimal, -1000, 1000, 0.01f, true);

const char pgmstr__manual_edit_level_max[] PROGMEM = "level_max";
MenuItemRange<float> manual_edit_item_level_max(pgmstr__manual_edit_level_max, storage.level_max, -1000, 1000, 0.01f, true);

const char pgmstr__manual_edit_valid_min[] PROGMEM = "valid_min";
MenuItemRange<float> manual_edit_item_valid_min(pgmstr__manual_edit_valid_min, storage.valid_min, -1000, 1000, 0.1f, true);

const char pgmstr__manual_edit_valid_max[] PROGMEM = "valid_max";
MenuItemRange<float> manual_edit_item_valid_max(pgmstr__manual_edit_valid_max, storage.valid_max, -1000, 1000, 0.1f, true);



void do_calibrate_zero_offset(){
  storage.zero_offset = -filtered;
  storage.save();
  beep(10);
}
const char pgmstr_calibrate_zero_offset[] PROGMEM = "SET zero_offset";
MenuItemCallable item_calibrate_zero_offset(pgmstr_calibrate_zero_offset, &do_calibrate_zero_offset, false);

void do_calibrate_level_min(){
  storage.level_min = filtered_corrected;
  storage.save();
  beep(10);
}
const char pgmstr_calibrate_level_min[] PROGMEM = "SET level_min";
MenuItemCallable item_calibrate_level_min(pgmstr_calibrate_level_min, &do_calibrate_level_min, false);

void do_reset_level_min(){
  storage.level_min = 1.0;
  storage.save();
  beep(10);
}
const char pgmstr_reset_level_min[] PROGMEM = "RESET level_min";
MenuItemCallable item_reset_level_min(pgmstr_reset_level_min, &do_reset_level_min, false);

void do_calibrate_level_optimal(){
  storage.level_optimal = filtered_corrected;
  storage.save();
  beep(10);
}
const char pgmstr_calibrate_level_optimal[] PROGMEM = "SET level_optimal";
MenuItemCallable item_calibrate_level_optimal(pgmstr_calibrate_level_optimal, &do_calibrate_level_optimal, false);

void do_calibrate_level_fill(){
  storage.level_fill = filtered_corrected;
  storage.save();
  beep(10);
}
const char pgmstr_calibrate_level_fill[] PROGMEM = "SET level_fill";
MenuItemCallable item_calibrate_level_fill(pgmstr_calibrate_level_fill, &do_calibrate_level_fill, false);

void do_reset_level_fill(){
  storage.level_fill = storage.level_optimal / 2.0;
  storage.save();
  beep(10);
}
const char pgmstr_reset_level_fill[] PROGMEM = "RESET level_fill";
MenuItemCallable item_reset_level_fill(pgmstr_reset_level_fill, &do_reset_level_fill, false);

void do_calibrate_level_max(){
  storage.level_max = filtered_corrected;
  storage.save();
  beep(10);
}
const char pgmstr_calibrate_level_max[] PROGMEM = "SET level_max";
MenuItemCallable item_calibrate_level_max(pgmstr_calibrate_level_max, &do_calibrate_level_max, false);

void do_reset_level_max(){
  storage.level_max = storage.level_optimal + 4.5;
  storage.save();
  beep(10);
}
const char pgmstr_reset_level_max[] PROGMEM = "RESET level_max";
MenuItemCallable item_reset_level_max(pgmstr_reset_level_max, &do_reset_level_max, false);

const char pgmstr_level_ir_gate[] PROGMEM = "Level IR gate";
uint16_t get_level_ir_gate(){
  return get_level_ir_gate_status();
}
MenuItemDynamicCallable<uint16_t> item__level_ir_gate(pgmstr_level_ir_gate, &get_level_ir_gate);

MenuItem* const calibrate_menu_items[] PROGMEM = {
  &back,
  &item_calibrate_zero_offset,
  &item_calibrate_level_min,
  &item_reset_level_min,
  &item_calibrate_level_optimal,
  &item_calibrate_level_fill,
  &item_reset_level_fill,
  &item_calibrate_level_max,
  &item_reset_level_max,
  &item__filtered_corrected,
  &item__filtered,
  &separator,
  &manual_edit_item_zero_offset,
  &manual_edit_item_level_min,
  &manual_edit_item_level_fill,
  &manual_edit_item_level_optimal,
  &manual_edit_item_level_max,
  &item_storage__tilt_max_temperature,
  &manual_edit_item_valid_min,
  &manual_edit_item_valid_max,
  &item__level_ir_gate,
};
Menu calibrate_menu(calibrate_menu_items, sizeof(calibrate_menu_items) / 2);
const char pgmstr_calibrate[] PROGMEM = "calibrate";
MenuItem item_calibrate_menu(pgmstr_calibrate, &calibrate_menu);




void setupCustom(){
  // setup diag pin interrupt
  PCICR |= (1 << PCIE0);
  PCMSK0 = 0;
  PCMSK0 |= (1 << PCINT4); // Z_MIN
  // PCMSK0 |= (1 << PCINT5); // Y_MIN
  // PCMSK0 |= (1 << PCINT6); // X_MIN

  for(size_t i = 1; i < MOTORS_MAX; i++){
    motors[i].inactivity_timeout = 0;
    motors[i].rpm(30);
  }

  // tilt
  motors[MT].accel = 150;
  motors[MT].decel = 150;
  // motors[MT].accel = 500;
  // motors[MT].decel = 500;
  motors[MT].planned.accel = motors[MT].accel;
  motors[MT].planned.decel = motors[MT].decel;
  motors[MT].autohome.enabled = true;
  motors[MT].autohome.autohome_on_move = false;
  motors[MT].autohome.direction = false;
  motors[MT].autohome.initial_rpm = 220;
  motors[MT].autohome.final_rpm = 220;
  motors[MT].autohome.initial_backstep_rot = 0.0;
  motors[MT].autohome.final_backstep_rot = 0.15;
  motors[MT].autohome.ramp_from = 10;
  motors[MT].autohome.wait_duration = 50;
  motors[MT].stop_on_stallguard = true; // IR optogate functions as a stallguard
  motors[MT].stop_on_stallguard_only_when_homing = false;
  // motors[MT].driver.TCOOLTHRS(0xffff);

  // pump
  motors[MP].accel = 150;
  motors[MP].decel = 400;
  motors[MP].stop_on_stallguard = false;
  motors[MP].driver.sgt(5);
  motors[MP].inactivity_timeout = 10000;
  motors[MP].ignore_stallguard = true;

  // init ad7150
  ch1_capdac.DacAuto = false;
  // writeRegister(AD7150_REG_CH1_SENSITIVITY, CH_SENSITIVITY);
  // writeRegister(AD7150_REG_CH1_SETUP, ch_setup.reg);
  // writeRegister(AD7150_REG_CH1_CAPDAC, ch1_capdac.reg);
  //
  // writeRegister(AD7150_REG_CH2_SENSITIVITY, CH_SENSITIVITY);
  // writeRegister(AD7150_REG_CH2_SETUP, ch_setup.reg);
  // writeRegister(AD7150_REG_CH2_CAPDAC, ch2_capdac.reg);
  //
  // // writeRegister(AD7150_REG_CONFIGURATION, MODE_SINGLE_CONV);
  // // writeRegister(AD7150_REG_CONFIGURATION, MODE_CONTINUOUS_CONV);
  // writeRegister(AD7150_REG_CONFIGURATION, configuration.reg);
  // writeRegister(AD7150_REG_POWER_DOWN_TIMER, 0x00);

  writeRegister(AD7150_REG_CH1_SENSITIVITY, 0x08);
  writeRegister(AD7150_REG_CH1_TIMEOUT, 0x86);
  writeRegister(AD7150_REG_CH1_SETUP, 0x0B);
  writeRegister(AD7150_REG_CH2_SENSITIVITY, 0x08);
  writeRegister(AD7150_REG_CH2_TIMEOUT, 0x86);
  writeRegister(AD7150_REG_CH2_SETUP, 0x0B);
  writeRegister(AD7150_REG_CONFIGURATION, 0x19);
  writeRegister(AD7150_REG_POWER_DOWN_TIMER, 0x00);
  // writeRegister(AD7150_REG_CH1_CAPDAC, 0x80); // enable
  // writeRegister(AD7150_REG_CH2_CAPDAC, 0xC0); // enable + auto
  writeRegister(AD7150_REG_CH1_CAPDAC, ch1_capdac.reg); // enable
  writeRegister(AD7150_REG_CH2_CAPDAC, ch2_capdac.reg); // enable + auto

  unsigned long seed = analogRead(A4) + analogRead(A6) + analogRead(A8) + analogRead(A10);
  seed += analogRead(A11) + analogRead(A14) + analogRead(A15);

  randomSeed(seed);
  pinModeInput(X_MIN_PIN);

  // print_gcode_to_lcd = true;
  main_menu.redraw_interval = 50;
  calibrate_menu.redraw_interval = 50;

  last_entered_motor_menu = 3;
  current_menu = &menu_motor_position_speed;
  menu_motor_position_speed.offset = 1;
  menu_motor_position_speed.current_item = 3;
  menu_motor_position_speed.came_from = &main_menu;
  // processCommand(F("set_position e0"));
  // print_gcode_to_lcd = true;

  // Serial.println();
  // Serial.println(F("ready!"));

  // SETUP_PIN(1);
  // SETUP_PIN(2);
  // SETUP_PIN(3);
  // SETUP_PIN(4);
  // SETUP_PIN(5);
  // SETUP_PIN(6);
  Serial.begin(115200);
  Serial.flush();
  Serial.println();
  Serial.println(F("ready!"));
}


void loopCustom(){
  const uint32_t _millis = millis();
  Wire.requestFrom(AD7150_I2C_ADDRESS, 1);
  const ad7150_reg_status status{.reg = (uint8_t)Wire.read()};

  if(autolevel_on_at > 0 && _millis >= autolevel_on_at){
    autolevel_on_at = 0;
    custom_gcode_autofill_on();
  }

  // Serial.print("status=");
  // Serial.print(status.reg);
  // Serial.print("\t");
  // Serial.print(status.reg, BIN);
  // if(!status.RDY1 && !status.RDY2) Serial.print(" <---");
  // Serial.println();

  if(!status.DacStep2){
    Serial.print("status=");
    Serial.print(status.reg);
    Serial.print("\t");
    ch2_capdac.reg = readRegister(AD7150_REG_CH2_CAPDAC);
    ch1_capdac.DacValue = ch2_capdac.DacValue;
    writeRegister(AD7150_REG_CH1_CAPDAC, ch1_capdac.reg, false);

    Serial.print("capdac change>");
    Serial.println(ch2_capdac.DacValue);

  }else
  if(!status.RDY1 && !status.RDY2){
    uint8_t data[5] = {0};

    Wire.requestFrom(AD7150_I2C_ADDRESS, 5);
    for (int i = 0; i < 5; i++){
      while(!Wire.available());
      data[i] = Wire.read();
    }

    // if(skip_samples > 0){
    //   skip_samples--;
    //   return;
    // }

    const uint16_t ch1_val = (((uint16_t)data[1] << 4) | ((uint16_t)data[2] >> 4));
    const uint16_t ch2_val = (((uint16_t)data[3] << 4) | ((uint16_t)data[4] >> 4));
    const int32_t delta = (int32_t)ch2_val - ch1_val;
    filtered = filter.updateEstimate(delta);
    filtered_corrected = filtered + storage.zero_offset;

    // Serial.print(delta);
    // Serial.print("\t");
    // Serial.print(filtered);
    // // Serial.print("\t");
    // // Serial.print(filtered_corrected);
    // Serial.println();

    // static float max_avg = 0.0;
    // static uint32_t max_avg_last_change = 0;
    // if(_millis >= max_avg_last_change + 3000){
    //   max_avg_last_change = _millis;
    //   max_avg = 0.0;
    // }
    // if(filtered_corrected > max_avg){
    //   max_avg_last_change = _millis;
    //   max_avg = filtered_corrected;
    // }

    static uint32_t last_pump_tick = 0; // 655356540;
    if(_millis >= last_pump_tick + 100){
    // if(true){
      // const float level_delta = running_average - autolevel_target;
      last_pump_tick = _millis;

      if(filtered_corrected <= storage.valid_min){
        capsense_reading_range = -1;
      }else if(filtered_corrected >= storage.valid_max){
        capsense_reading_range = 1;
      }else{
        capsense_reading_range = 0;
      }

      if(capsense_reading_range != 0){
        if(motors[MP].is_busy() && !motors[MP].dir()) pump_stop(true);
        return;
      }

      if(abs(filtered_corrected - autolevel_target) > level_tolerance){
        if(filtered_corrected <= autolevel_target){
          // need more resin
          if(filtered_corrected <= storage.level_min){
            new_rpm = rpm_optimal;
          }else{
            const float pwr = 1.0 - ((filtered_corrected - storage.level_min) / (autolevel_target - storage.level_min));
            new_rpm = rpm_min + ((rpm_optimal - rpm_min) * pwr);
          }
          if(autolevel_enabled){
            if(motors[MP].dir()) motors[MP].dir(false);
            motors[MP].target_rpm = new_rpm;
            if(!motors[MP].is_busy()){
              motors[MP].rpm(1);
              motors[MP].start(1);
              // motors[MP].ramp_to(new_rpm);
              last_pump_tick += 500;
            }
          }

        }else{
          // too much resin
          if(filtered_corrected >= storage.level_max){
            new_rpm = rpm_max;
          }else{
            const float pwr = ((filtered_corrected - autolevel_target) / (storage.level_max - autolevel_target));
            new_rpm = rpm_min + ((rpm_max - rpm_min) * pwr);
          }
          if(autolevel_enabled){
            if(!motors[MP].dir()) motors[MP].dir(true);
            motors[MP].target_rpm = new_rpm;
            if(!motors[MP].is_busy()){
              motors[MP].rpm(1);
              motors[MP].start(1);
              // motors[MP].ramp_to(new_rpm);
              last_pump_tick += 500;
            }
          }

        }
      }else{
        if(autolevel_enabled){
          new_rpm = 0.0;
          motors[MP].target_rpm = new_rpm;
          // Serial.println(F("  set to 0 rpm"));
        }

      }

      // Serial.print(delta);
      // Serial.print("\t");
      // Serial.print(filtered);
      // Serial.print("\t");
      // Serial.print(filtered_corrected);
      // Serial.println();

    }

    if(capsense_reading_range != 0 && (autolevel_enabled || (motors[MP].is_busy() && !motors[MP].dir())) && _millis >= last_capsense_reading_error_beep + 2000){
      last_capsense_reading_error_beep = _millis;
      beep(200);
    }
  }

  static uint32_t last_total_dist_calculated_at = 0;
  if(_millis >= last_total_dist_calculated_at + 50){
    last_total_dist_calculated_at = _millis;
    total_dist = (float)motors[MP].steps_total / 200.0 / motors[MP].usteps;
  }

  if(pump_off_time_remaining){
    uint16_t delta = _millis - pump_off_time_decrease_last_tick;
    pump_off_time_decrease_last_tick = _millis;
    pump_off_time_remaining -= delta > pump_off_time_remaining ? pump_off_time_remaining : delta;
    if(pump_off_time_remaining == 0){
      pump_stop(false);
    }
  }

  static uint32_t last_tilt_overtemperature_check = 0;
  if(_millis >= last_tilt_overtemperature_check + 3000){
    last_tilt_overtemperature_check = _millis;
    if(TILT_TEMPERATURE > storage.tilt_max_temperature){
      beep(50);
    }
  }

  if(pump_slowdown_at > 0 && _millis >= pump_slowdown_at){
    pump_slowdown_at = 0;
    // motors[MP].decel = 1;
    // motors[MP].planned.decel = 1;
    motors[MP].ramp_to(40);
  }

  uptime = _millis;

  static bool last_level_ir_gate_status = false;
  bool level_ir_gate_status = get_level_ir_gate_status();
  if(last_level_ir_gate_status != level_ir_gate_status){
    last_level_ir_gate_status = level_ir_gate_status;
    // Serial.print(F("level_ir_gate_status = "));
    // Serial.println(level_ir_gate_status);
    // beep(level_ir_gate_status ? 100 : 10);
  }
}


// stallguard(pinda) pin change interrupt
ISR(PCINT0_vect){
  const bool sg[MOTORS_MAX] = {
    false, // !(PINB & (1 << PINB4)), // Z_MIN, motor X is stopped by pinda
    false, // PINB & (1 << PINB5), // Y_MIN
    false, // PINB & (1 << PINB6), // X_MIN
    !(PINB & (1 << PINB4)), // false, // E0_MIN
  };

  // Serial.print("SG Cust Int ");
  // Serial.print(sg[0]);
  // Serial.print(sg[1]);
  // Serial.print(sg[2]);
  // Serial.println(sg[3]);

  if(!sg[3]) return;

  for(size_t i = 3; i < MOTORS_MAX; i++){
    if(sg[i]){
      if(!motors[i].is_homing) continue;
      if(motors[i].ignore_stallguard_steps > 0) continue;

      motors[i].stallguard_triggered = true;
      if(motors[i].is_expecting_stallguard()){
        motors[i].running = false;
        motors[i].pause_steps = true;
        motors[i].steps_to_do = 0;
        motors[i].process_next_queue_item();
        motors[i].pause_steps = false;

      }else{
        if(motors[i].stop_on_stallguard) motors[i].stop();
        if(motors[i].reset_is_homed_on_stall){
          motors[i].is_homed = false;
          motors[i].planned.is_homed = false;
        }
      }

    }
  }
}



// main menu
// void do_menu_item_template(){
// }
// const char pgmstr_menu_item_template[] PROGMEM = "menu_item_template";
// MenuItemCallable item_menu_item_template(pgmstr_menu_item_template, &do_menu_item_template, false);


void do_z_motors_off(){
  for (size_t i = 0; i < 2; i++) motors[towers[i]].off();
}
const char pgmstr_z_motors_off[] PROGMEM = "Z motors off";
MenuItemCallable item_z_motors_off(pgmstr_z_motors_off, &do_z_motors_off, false);



void do_home_coarse(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(5); // 8
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(0);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(80);

    motors[towers[i]].start(true);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_home_coarse_nowait(){ do_home_coarse(false); }
const char pgmstr_home_coarse[] PROGMEM = "home coarse";
MenuItemCallable item_home_coarse(pgmstr_home_coarse, &do_home_coarse_nowait, false);



void do_home_fast(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(5); // 8
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(0);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(120);

    motors[towers[i]].start(true);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_home_fast_nowait(){ do_home_fast(false); }
const char pgmstr_home_fast[] PROGMEM = "home fast";
MenuItemCallable item_home_fast(pgmstr_home_fast, &do_home_fast_nowait, false);



void do_home_sensitive(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(300*CURRENT_MP);
    motors[towers[i]].driver.sgt(1); // 8
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(16);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(80);

    motors[towers[i]].start(true);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_home_sensitive_nowait(){ do_home_sensitive(false); }
const char pgmstr_home_sensitive[] PROGMEM = "home sensitive";
MenuItemCallable item_home_sensitive(pgmstr_home_sensitive, &do_home_sensitive_nowait, false);



void do_coarse_backstep(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(8);
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(16);
    motors[towers[i]].dir(true);
    motors[towers[i]].rpm(120);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(0.1);
    motors[towers[i]].start(false);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_coarse_backstep_nowait(){ do_coarse_backstep(false); }
const char pgmstr_coarse_backstep[] PROGMEM = "coarse backstep";
MenuItemCallable item_coarse_backstep(pgmstr_coarse_backstep, &do_coarse_backstep_nowait, false);


void do_big_backstep(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(8);
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(16);
    motors[towers[i]].dir(true);
    motors[towers[i]].rpm(120);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(5.0);
    motors[towers[i]].start(false);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_big_backstep_nowait(){ do_big_backstep(false); }
const char pgmstr_big_backstep[] PROGMEM = "big backstep";
MenuItemCallable item_big_backstep(pgmstr_big_backstep, &do_big_backstep_nowait, false);



void do_home_fine(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(1);
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(128);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(45);

    motors[towers[i]].start(true);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_home_fine_nowait(){ do_home_fine(false); }
const char pgmstr_home_fine[] PROGMEM = "home fine";
MenuItemCallable item_home_fine(pgmstr_home_fine, &do_home_fine_nowait, false);



void do_tram_z(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(13);
    motors[towers[i]].stop_on_stallguard = false;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(256);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(20);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(0.5);
    motors[towers[i]].start(false);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_tram_z_nowait(){ do_tram_z(false); }
const char pgmstr_tram_z[] PROGMEM = "tram z";
MenuItemCallable item_tram_z(pgmstr_tram_z, &do_tram_z_nowait, false);


void do_tram_z_weak(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(300*CURRENT_MP);
    motors[towers[i]].driver.sgt(13);
    motors[towers[i]].stop_on_stallguard = false;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(256);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(5);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(0.3);
    motors[towers[i]].start(false);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_tram_z_weak_nowait(){ do_tram_z_weak(false); }
const char pgmstr_tram_z_weak[] PROGMEM = "tram z weak";
MenuItemCallable item_tram_z_weak(pgmstr_tram_z_weak, &do_tram_z_weak_nowait, false);



void do_fine_backstep(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(5);
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(16);
    motors[towers[i]].dir(true);
    motors[towers[i]].rpm(50);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(0.3);
    motors[towers[i]].start(false);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_fine_backstep_wait(){ do_fine_backstep(true); }
const char pgmstr_fine_backstep[] PROGMEM = "fine backstep";
MenuItemCallable item_fine_backstep(pgmstr_fine_backstep, &do_fine_backstep_wait, false);



void do_home(){
  do_home_coarse(true);
  delay(500);

  // do_coarse_backstep(true);
  // delay(500);

  // do_home_fine(true);
  // delay(500);

  do_tram_z(true);
  delay(500);

  do_fine_backstep(true);

}
const char pgmstr_home[] PROGMEM = "home....";
MenuItemCallable item_home(pgmstr_home, &do_home, false);



void do_home_weak(){
  do_home_coarse(true);
  delay(500);

  // do_coarse_backstep(true);
  // delay(500);

  // do_home_fine(true);
  // delay(500);

  do_tram_z_weak(true);
  delay(500);

  do_fine_backstep(true);

}
void custom_gcode_home_weak(){ do_home_weak(); }
const char pgmstr_home_weak[] PROGMEM = "home weak....";
MenuItemCallable item_home_weak(pgmstr_home_weak, &do_home_weak, false);


void custom_gcode_home_tower(){
  do_home_coarse(true);
  delay(500);

  // do_coarse_backstep(true);
  // delay(500);

  // do_home_fine(true);
  // delay(500);

  do_tram_z_weak(true);
  delay(500);

  do_fine_backstep(true);

}



void _tilt_step_up(){
  // processCommand(F("move_rot e0.3 f80"));
  // processCommand(F("move_rot e1 f120"));
  processCommand(F("move_rot e0.1 f20"));
  processCommand(F("start e0"));
  delay(10);
  processCommand(F("wait_for_motor e"));
  // delay(50);
}
void do_home_tilt(bool do_wait = true){
  // print_homing_homed_busy = true;
  processCommand(F("set_is_homing_override e1"));
  processCommand(F("set_is_homed_override e1"));

  if(!(PINB & (1 << PINB4))){
    // Serial.println("pinda is triggered..");
    while(!(PINB & (1 << PINB4))){
      // Serial.println("pinda is still triggered..");
      _tilt_step_up();
    }
    _tilt_step_up();
  }

  processCommand(F("autohome e"));
  // processCommand(F("set_is_homing e1"));
  // processCommand(F("set_is_homed e0"));
  // processCommand(F("move e3.8 f50"));

  processCommand(F("move_rot e-0.15"));
  processCommand(F("set_position e0"));
  processCommand(F("set_is_homing_override e-1"));
  processCommand(F("set_is_homed_override e-1"));
  processCommand(F("start e0"));
  if(do_wait){
    delay(10);
    processCommand(F("wait_for_motor e"));
    // Serial.println(F(">> wait finished"));
  }

}
void do_home_tilt_wait(){ do_home_tilt(false /*true*/ ); }
void custom_gcode_home_tilt(){ do_home_tilt(false); }
const char pgmstr_home_tilt[] PROGMEM = "home tilt";
MenuItemCallable item_home_tilt(pgmstr_home_tilt, &do_home_tilt_wait, false);


void custom_gcode_is_endstop_triggered(){
  Serial.println(!(PINB & (1 << PINB4)));
}


void custom_gcode_autofill_on_delayed_short(){
  autolevel_on_at = millis() + 1000UL;
}


void custom_gcode_autofill_on_delayed(){
  autolevel_on_at = millis() + 3500UL; // 30000UL;
}


void custom_gcode_autofill_on(){
  autolevel_on_at = 0;
  autolevel_target = storage.level_optimal;
  autolevel_enabled = true;
  if(!motors[MP].is_busy()){
    motors[MP].rpm(10.0);
    motors[MP].start(true);
  }
  start_pump_auto_off_timer();
}


void custom_gcode_autofill_off(){
  autolevel_on_at = 0;
  autolevel_enabled = false;
  if(motors[MP].is_busy()){
    pump_stop(false);
  }
  stop_pump_auto_off_timer();
}


void custom_gcode_fill_wait(){
  static bool fill_wait_last_state = false;
  static uint32_t fill_wait_last_hit = 0;
  // Serial.println("fill with wait...");
  const bool old_autolevel_enabled = autolevel_enabled;
  custom_gcode_autofill_on();
  stop_pump_auto_off_timer();
  autolevel_target = storage.level_fill;
  // while(running_average_corrected <= (storage.level_optimal - level_tolerance - 0.3)){
  //   // loopCustom();
  //   loop();
  //   delay(1);
  // }

  // while(1){
  //   const uint32_t _millis = millis();
  //   const bool state = running_average_corrected <= (storage.level_optimal - level_tolerance - 1.3 /*0.3*/);
  //   if(state != fill_wait_last_state){
  //     fill_wait_last_state = state;
  //     // fill_wait_last_hit = state ? _millis : 0;
  //     fill_wait_last_hit = _millis;
  //     // Serial.print("change to ");
  //     // Serial.print(running_average_corrected);
  //     // Serial.print(" at ");
  //     // Serial.print(fill_wait_last_hit);
  //     // Serial.println(state ? " true" : " false");
  //   }
  //
  //   if(!state && fill_wait_last_hit > 0 && _millis > fill_wait_last_hit + 10000UL /*30000UL*/ /*120000*/){
  //     // Serial.print("return at ");
  //     // Serial.println(_millis);
  //
  //     return;
  //   }
  //
  //   loop();
  //   delay(1);
  // }
  // while(filtered_corrected < (storage.level_optimal - 2.3)){
  while(filtered_corrected < storage.level_fill){
    loop();
    delay(1);
  }

  // if(!old_autolevel_enabled) custom_gcode_autofill_off();
  custom_gcode_autofill_off();
}


void custom_gcode_fill_wait_stable(){
  static bool fill_wait_last_state = false;
  static uint32_t fill_wait_last_hit = 0;
  const bool old_autolevel_enabled = autolevel_enabled;
  custom_gcode_autofill_on();
  stop_pump_auto_off_timer();
  autolevel_target = storage.level_fill;
  // while(filtered <= (storage.level_optimal - level_tolerance - 0.3)){
  //   // loopCustom();
  //   loop();
  //   delay(1);
  // }

  while(1){
    const uint32_t _millis = millis();
    const bool state = filtered < storage.level_fill;
    if(state != fill_wait_last_state){
      fill_wait_last_state = state;
      // fill_wait_last_hit = state ? _millis : 0;
      fill_wait_last_hit = _millis;
      // Serial.print("change to ");
      // Serial.print(filtered);
      // Serial.print(" at ");
      // Serial.print(fill_wait_last_hit);
      // Serial.println(state ? " true" : " false");
    }

    if(!state && fill_wait_last_hit > 0 && _millis > fill_wait_last_hit + 20000UL /*10000UL*/ /*30000UL*/ /*120000*/){
      Serial.print("return at ");
      Serial.println(_millis);

      break;
    }

    loop();
    delay(1);
  }

  // while(filtered_corrected < (storage.level_optimal - 2.3)){
  //   loop();
  //   delay(1);
  // }

  // if(!old_autolevel_enabled) custom_gcode_autofill_off();
  custom_gcode_autofill_off();
}


void custom_gcode_empty_tank(){
  if(motors[MP].is_busy()){
    pump_stop(false);
    return;
  }
  pump_start(false, 140);
  enable_pump_slowdown();
}


void custom_gcode_get_level_fill(){
  Serial.println(storage.level_fill, 2);
}


void custom_gcode_get_level_optimal(){
  Serial.println(storage.level_optimal, 2);
}


void custom_gcode_get_level_tolerance(){
  Serial.println(level_tolerance, 2);
}


void custom_gcode_capsense_raw(){
  Serial.println(filtered, 2);
}


void custom_gcode_capsense(){
  Serial.println(filtered_corrected, 2);
}


void custom_gcode_set_target_fill(){
  autolevel_target = storage.level_fill;
}


void custom_gcode_set_target_optimal(){
  autolevel_target = storage.level_optimal;
}


void custom_gcode_tilt_temp(){
  Serial.println(TILT_TEMPERATURE, 2);
}


void custom_gcode_level_ir_gate(){
  Serial.println(get_level_ir_gate_status());
}



void do_pump_stop(){
  lcd.clear();
  lcd.print("Stopping...");
  pump_stop(true);
}
const char pgmstr_pump_stop[] PROGMEM = "pump_stop";
MenuItemCallable item_pump_stop(pgmstr_pump_stop, &do_pump_stop, false);


void do_pump_start_fill(){
  if(motors[MP].is_busy()){
    do_pump_stop();
    return;
  }
  pump_start(true);
}
const char pgmstr_pump_start_fill[] PROGMEM = "pump_start_fill";
MenuItemCallable item_pump_start_fill(pgmstr_pump_start_fill, &do_pump_start_fill, false);


void do_pump_start_empty(){
  if(motors[MP].is_busy()){
    do_pump_stop();
    return;
  }
  pump_start(false, 140);
  enable_pump_slowdown();
}
const char pgmstr_pump_start_empty[] PROGMEM = "pump_start_empty";
MenuItemCallable item_pump_start_empty(pgmstr_pump_start_empty, &do_pump_start_empty, false);


bool is_auto_level_on(){ return autolevel_enabled; }
void do_auto_level_on(){
  autolevel_target = storage.level_optimal;
  autolevel_enabled = true;
  if(!motors[MP].is_busy()){
    motors[MP].rpm(10.0);
    motors[MP].start(true);
  }
}
void do_auto_level_off(){
  autolevel_enabled = false;
  if(motors[MP].is_busy()){
    do_pump_stop();
  }
}
const char pgmstr_auto_level_on[] PROGMEM = "Autolevel: on  ";
const char pgmstr_auto_level_off[] PROGMEM = "Autolevel: off ";
MenuItemToggleCallable item_auto_level(&is_auto_level_on, pgmstr_auto_level_on, pgmstr_auto_level_off, &do_auto_level_off, &do_auto_level_on);



void do_random_skew(bool wait = true){
  float dist = 0.3; // + (float(random(100)) / 100.0);
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(8);
    motors[towers[i]].stop_on_stallguard = false;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(0);
    motors[towers[i]].dir(true);
    motors[towers[i]].rpm(80);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(dist);
    motors[towers[i]].start(false);
  }
  WAIT_FOR_TOWERS;
  for (size_t i = 0; i < 2; i++) {
    dist = float(random(40)) / 40.0;
    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(dist);
    Serial.print("motor ");
    Serial.print(motors[towers[i]].axis);
    Serial.print(" rot: ");
    Serial.println(dist);
  }
  for (size_t i = 0; i < 2; i++) motors[towers[i]].start(false);
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_random_skew_nowait(){ do_random_skew(false); }
const char pgmstr_random_skew[] PROGMEM = "random skew";
MenuItemCallable item_random_skew(pgmstr_random_skew, &do_random_skew_nowait, false);


bool is_invert_tower_direction_on(){ return motors[towers[0]].invert_direction; }
void do_invert_tower_direction_on(){ for (size_t i = 0; i < 2; i++) motors[towers[i]].invert_direction = true; }
void do_invert_tower_direction_off(){ for (size_t i = 0; i < 2; i++) motors[towers[i]].invert_direction = false; }
const char pgmstr_invert_tower_direction_on[] PROGMEM = "Motors invert: on";
const char pgmstr_invert_tower_direction_off[] PROGMEM = "Motors invert: off";
MenuItemToggleCallable item_invert_tower_direction_on_off(&is_invert_tower_direction_on, pgmstr_invert_tower_direction_on,
  pgmstr_invert_tower_direction_off, &do_invert_tower_direction_off, &do_invert_tower_direction_on);


bool is_print_stallguard_on(){ return motors[towers[0]].print_stallguard_to_serial; }
void do_print_stallguard_on(){ for (size_t i = 0; i < 2; i++) motors[towers[i]].print_stallguard_to_serial = true; }
void do_print_stallguard_off(){ for (size_t i = 0; i < 2; i++) motors[towers[i]].print_stallguard_to_serial = false; }
const char pgmstr_print_stallguard_on[] PROGMEM = "Print sg: on";
const char pgmstr_print_stallguard_off[] PROGMEM = "Print sg: off";
MenuItemToggleCallable item_print_stallguard_on_off(&is_print_stallguard_on, pgmstr_print_stallguard_on,
  pgmstr_print_stallguard_off, &do_print_stallguard_off, &do_print_stallguard_on);




MenuItem* const debug_menu_items[] PROGMEM = {
  &back,
  &motor_x,
  &motor_y,
  &motor_z,
  &motor_e,
  &separator,
  &item__autolevel_target,
  // &item_home_fast,
  // &item_home_coarse,
  // &item_home_sensitive,
  // &item_coarse_backstep,
  // &item_big_backstep,
  // &item_home_fine,
  // &item_tram_z,
  // &item_tram_z_weak,
  // &item_fine_backstep,
  // &item_home,
  // &item_home_weak,
  &item_home_tilt,
  &item_random_skew,
  &item_print_stallguard_on_off,
  // &item_invert_tower_direction_on_off,
  &separator,
  &item_calibrate_menu,
  &item_uptime,
};
Menu debug_menu(debug_menu_items, sizeof(debug_menu_items) / 2);
const char pgmstr_debug[] PROGMEM = "debug";
MenuItem item_debug_menu(pgmstr_debug, &debug_menu);


const char pgmstr_pump[] PROGMEM = "pump";
const char* get_pump_status(){
  if(motors[MP].is_busy()){
    if(motors[MP].dir()) return "emptying";
    else return "filling";
  }
  return "idle";
}
MenuItemDynamicCallable<const char*> item__pump_status(pgmstr_pump, &get_pump_status);


const char pgmstr_tilt_temp[] PROGMEM = "Tilt temp";
MenuItemDynamic<float> item_tilt_temp(pgmstr_tilt_temp, TILT_TEMPERATURE);

const char pgmstr_fep_temp[] PROGMEM = "FEP temp";
MenuItemDynamic<float> item_fep_temp(pgmstr_fep_temp, FEP_TEMPERATURE);



MenuItem* const main_menu_items[] PROGMEM = {
  // &item_fep_temp,
  &item__filtered_corrected,
  &item__pump_status,
  &item_auto_level,
  &item_pump_start_fill,
  &item_pump_start_empty,
  &item_pump_stop,
  &item_pump_off_time,
  // &item_z_motors_off,
  &item_tilt_temp,
  &separator,
  &item_debug_menu,
  &separator,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);




void writeRegister(uint8_t reg, uint16_t value, bool wait){
  Wire.begin();
  Wire.beginTransmission(AD7150_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  if(wait) delay(4);
}


uint8_t readRegister(uint8_t reg){
  Wire.begin();
  Wire.beginTransmission(AD7150_I2C_ADDRESS);
  Wire.write(reg);
  Wire.requestFrom(AD7150_I2C_ADDRESS, 1);
  while(!Wire.available());
  const uint8_t result = /*(uint8_t)*/Wire.read();
  Wire.endTransmission();
  return result;
}


#endif
