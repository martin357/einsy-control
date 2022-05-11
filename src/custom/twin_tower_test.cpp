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

ad7150_reg_setup ch_setup;
ad7150_reg_capdac ch1_capdac, ch2_capdac;
ad7150_reg_configuration configuration;

int32_t last_autolevel_tick = 0;
bool autolevel_enabled = false;
uint32_t autolevel_on_at = 0;
double level_tolerance = 0.5; // 0.3;

double rpm_min = 50.0;
double rpm_optimal = 180.0;
double rpm_max = 180.0;
double new_rpm = 0.0001;
double total_dist = 0.0;

KalmanFilter filter(10, 1, 0.01);
double filtered = 0.0;
double filtered_corrected = 0.0;


void writeRegister(uint8_t reg, uint16_t value, bool wait = true);
uint8_t readRegister(uint8_t reg);

// bool print_homing_homed_busy = false;
// uint32_t last_print_hhb = 0;


void pump_start(bool dir, double target_rpm = 150){
  processCommand(F("halt y"));
  processCommand(F("empty_queue y"));
  processCommand(dir ? F("dir y0") : F("dir y1"));
  // processCommand(F("accel y150"));
  processCommand(F("rpm y0.01"));
  processCommand(F("start y1"));
  // processCommand(F("ramp_to y150"));
  motors[1].ramp_to(target_rpm);
}

void pump_stop(bool wait = true){
  // processCommand(F("decel y150"));
  // processCommand(F("ramp_to y0.01"));
  processCommand(F("ramp_to y0"));
  if(wait) processCommand(F("wait_for_motor y"));
}

#define PRINT_ANALOG(pin)  Serial.print("" # pin " = "); Serial.println(analogRead(pin));



MenuItemDynamic<double> item__filtered("filtered", filtered);
MenuItemDynamic<double> item__filtered_corrected("corrected", filtered_corrected);
MenuItemDynamic<double> item_storage__zero_offset("zero_offset", storage.zero_offset);
MenuItemDynamic<double> item_storage__level_min("level_min", storage.level_min);
MenuItemDynamic<double> item_storage__level_optimal("level_optimal", storage.level_optimal);
MenuItemDynamic<double> item_storage__level_max("level_max", storage.level_max);

void do_calibrate_zero_offset(){
  storage.zero_offset = -filtered;
  storage.save();
  beep(10);
}
const char pgmstr_calibrate_zero_offset[] PROGMEM = "SET zero_offset";
MenuItemCallable item_calibrate_zero_offset(pgmstr_calibrate_zero_offset, &do_calibrate_zero_offset, false);

void do_calibrate_level_optimal(){
  storage.level_min = 1.0;
  storage.level_optimal = filtered_corrected; // 4.5;
  storage.level_max = filtered_corrected + 4.5; // 9.0;
  storage.save();
  beep(10);
}
const char pgmstr_calibrate_level_optimal[] PROGMEM = "SET level_optimal";
MenuItemCallable item_calibrate_level_optimal(pgmstr_calibrate_level_optimal, &do_calibrate_level_optimal, false);

MenuItem* const calibrate_menu_items[] PROGMEM = {
  &back,
  &item_calibrate_zero_offset,
  &item_calibrate_level_optimal,
  &item__filtered_corrected,
  &item__filtered,
  &item_storage__zero_offset,
  &item_storage__level_min,
  &item_storage__level_optimal,
  &item_storage__level_max,
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
  }
  motors[3].rpm(30);
  motors[3].on();

  // tilt
  motors[3].accel = 150;
  motors[3].decel = 150;
  // motors[3].accel = 500;
  // motors[3].decel = 500;
  motors[3].planned.accel = motors[3].accel;
  motors[3].planned.decel = motors[3].decel;
  motors[3].autohome.enabled = true;
  motors[3].autohome.direction = false;
  motors[3].autohome.initial_rpm = 220;
  motors[3].autohome.final_rpm = 10;
  motors[3].autohome.initial_backstep_rot = 0.0;
  motors[3].autohome.final_backstep_rot = 0.15;
  motors[3].autohome.ramp_from = 10;
  motors[3].autohome.wait_duration = 50;
  motors[3].stop_on_stallguard = false; // IR optogate functions as a stallguard
  motors[3].stop_on_stallguard_only_when_homing = true;
  // motors[3].driver.TCOOLTHRS(0xffff);

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
  writeRegister(AD7150_REG_CH1_CAPDAC, 0xC0);
  writeRegister(AD7150_REG_CH2_CAPDAC, 0xC0);

  unsigned long seed = analogRead(A4) + analogRead(A6) + analogRead(A8) + analogRead(A10);
  seed += analogRead(A11) + analogRead(A14) + analogRead(A15);

  randomSeed(seed);

  main_menu.redraw_interval = 50;
  calibrate_menu.redraw_interval = 50;
}


void loopCustom(){
  const uint32_t _millis = millis();
  Wire.requestFrom(AD7150_I2C_ADDRESS, 1);
  const ad7150_reg_status status{.reg = (uint8_t)Wire.read()};

  if(autolevel_on_at > 0 && _millis >= autolevel_on_at){
    autolevel_on_at = 0;
    custom_gcode_autofill_on();
  }

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

    // static double max_avg = 0.0;
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
    // if(_millis >= last_pump_tick + 100){
    if(true){
      // const double level_delta = running_average - storage.level_optimal;
      last_pump_tick = _millis;

      if(abs(filtered_corrected - storage.level_optimal) > level_tolerance){
        if(filtered_corrected <= storage.level_optimal){
          // need more resin
          if(filtered_corrected <= storage.level_min){
            new_rpm = rpm_optimal;
          }else{
            const double pwr = 1.0 - ((filtered_corrected - storage.level_min) / (storage.level_optimal - storage.level_min));
            new_rpm = rpm_min + ((rpm_optimal - rpm_min) * pwr);
          }
          if(autolevel_enabled){
            if(motors[MP].dir()) motors[MP].dir(false);
            motors[MP].target_rpm = new_rpm;
          }

        }else{
          // too much resin
          if(filtered_corrected >= storage.level_max){
            new_rpm = rpm_max;
          }else{
            const double pwr = 1.0 - ((filtered_corrected - storage.level_optimal) / (storage.level_max - storage.level_optimal));
            new_rpm = rpm_min + ((rpm_max - rpm_min) * pwr);
          }
          if(autolevel_enabled){
            if(!motors[MP].dir()) motors[MP].dir(true);
            motors[MP].target_rpm = new_rpm;
          }

        }
      }else{
        if(autolevel_enabled){
          new_rpm = 0.0001;
          motors[MP].target_rpm = new_rpm;
        }

      }

      // Serial.print(delta);
      // Serial.print("\t");
      // Serial.print(filtered);
      // Serial.print("\t");
      // Serial.print(filtered_corrected);
      // Serial.println();

    }
  }

  static uint32_t last_total_dist_calculated_at = 0;
  if(_millis >= last_total_dist_calculated_at + 50){
    last_total_dist_calculated_at = _millis;
    total_dist = (double)motors[MP].steps_total / 200.0 / motors[MP].usteps;
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

  if(!sg[0] && !sg[1] && !sg[2] && !sg[3]) return;

  for(size_t i = 0; i < MOTORS_MAX; i++){
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
    while(!(PINB & (1 << PINB4))){
      _tilt_step_up();
    }
    _tilt_step_up();
  }

  processCommand(F("autohome e"));
  // processCommand(F("set_is_homing e1"));
  // processCommand(F("set_is_homed e0"));
  // processCommand(F("move e3.8 f50"));

  processCommand(F("move_rot e-0.1"));
  processCommand(F("set_position e0"));
  processCommand(F("set_is_homing_override e-1"));
  processCommand(F("set_is_homed_override e-1"));
  processCommand(F("start e0"));
  if(do_wait){
    delay(10);
    processCommand(F("wait_for_motor e"));
    Serial.println(F(">> wait finished"));
  }

}
void do_home_tilt_wait(){ do_home_tilt(true); }
void custom_gcode_home_tilt(){ do_home_tilt(false); }
const char pgmstr_home_tilt[] PROGMEM = "home tilt";
MenuItemCallable item_home_tilt(pgmstr_home_tilt, &do_home_tilt_wait, false);


void custom_gcode_is_endstop_triggered(){
  Serial.println(!(PINB & (1 << PINB4)));
}


void custom_gcode_autofill_on_delayed(){
  autolevel_on_at = millis() + 3500UL; // 30000UL;
}


void custom_gcode_autofill_on(){
  autolevel_on_at = 0;
  autolevel_enabled = true;
  if(!motors[1].is_busy()){
    motors[1].rpm(10.0);
    motors[1].start(true);
  }
}


void custom_gcode_autofill_off(){
  autolevel_on_at = 0;
  autolevel_enabled = false;
  if(motors[1].is_busy()){
    pump_stop(false);
  }
}


void custom_gcode_fill_wait(){
  static bool fill_wait_last_state = false;
  static uint32_t fill_wait_last_hit = 0;
  // Serial.println("fill with wait...");
  const bool old_autolevel_enabled = autolevel_enabled;
  custom_gcode_autofill_on();
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
  while(filtered_corrected < (storage.level_optimal - 2.3)){
    loop();
    delay(1);
  }

  // if(!old_autolevel_enabled) custom_gcode_autofill_off();
  custom_gcode_autofill_off();
}


void custom_gcode_empty_tank(){
  if(motors[1].is_busy()){
    pump_stop(false);
    return;
  }
  pump_start(false, 140);
}



void do_pump_stop(){
  lcd.clear();
  lcd.print("Stopping...");
  pump_stop(true);
}
const char pgmstr_pump_stop[] PROGMEM = "pump_stop";
MenuItemCallable item_pump_stop(pgmstr_pump_stop, &do_pump_stop, false);


void do_pump_start_fill(){
  if(motors[1].is_busy()){
    do_pump_stop();
    return;
  }
  pump_start(true);
}
const char pgmstr_pump_start_fill[] PROGMEM = "pump_start_fill";
MenuItemCallable item_pump_start_fill(pgmstr_pump_start_fill, &do_pump_start_fill, false);


void do_pump_start_empty(){
  if(motors[1].is_busy()){
    do_pump_stop();
    return;
  }
  pump_start(false, 60);
}
const char pgmstr_pump_start_empty[] PROGMEM = "pump_start_empty";
MenuItemCallable item_pump_start_empty(pgmstr_pump_start_empty, &do_pump_start_empty, false);


bool is_auto_level_on(){ return autolevel_enabled; }
void do_auto_level_on(){
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
const char pgmstr_auto_level_on[] PROGMEM = "Autolevel: on";
const char pgmstr_auto_level_off[] PROGMEM = "Autolevel: off";
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




MenuItem* const main_menu_items[] PROGMEM = {
  &item__filtered_corrected,
  &item_auto_level,
  &item_pump_start_fill,
  &item_pump_start_empty,
  &item_pump_stop,
  // &item_z_motors_off,
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
  // &item_home_tilt,
  // &item_random_skew,
  &motor_x,
  // &motor_y,
  // &motor_z,
  &motor_e,
  // &item_print_stallguard_on_off,
  // &item_invert_tower_direction_on_off,
  &item_calibrate_menu,
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
