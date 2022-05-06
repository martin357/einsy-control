#include "cap_sense_test.h"
#include <Arduino.h>
#include <Wire.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#include "../../serial.h"
#ifdef CUSTOM_CAP_SENSE_TEST

#include "../ad7150_registers.h"
#include "../ad7150.h"
#include "../kalmanfilter.h"

uint8_t skip_samples = 64;
bool print_full_data = false;
ad7150_reg_setup ch_setup;
ad7150_reg_capdac ch1_capdac, ch2_capdac;
ad7150_reg_configuration configuration;
int32_t initial_tare = 0;
int32_t tare = 0;
uint16_t samples_total = 128; // 256; // 6; // *32;
uint16_t samples_collected = 0;
double running_average;
double running_average_corrected;
// double offset = 94.0; // moved to permanent_storage
// double offset = -8.5;

int32_t last_autolevel_tick = 0;
bool autolevel_enabled = false;
double level_tolerance = 0.5; // 0.3;
// double level_min = 1.0;
// double level_optimal = 4.5;
// double level_max = 9.0; // 7.0;

// double rpm_min = 20.0;
// double rpm_optimal = 150.0;
double rpm_min = 50.0;
double rpm_optimal = 180.0; // 120.0;
double rpm_max = 180.0; // 160.0; // 80.0; // sucking resin out
double new_rpm = 0.0001;
double total_dist = 0.0;

KalmanFilter filter(1, 1, 0.01);
uint16_t samples_collected_16 = 0;
double running_average_16;
uint16_t samples_collected_32 = 0;
double running_average_32;
uint16_t samples_collected_64 = 0;
double running_average_64;
uint16_t samples_collected_128 = 0;
double running_average_128;
uint16_t samples_collected_256 = 0;
double running_average_256;
uint16_t samples_collected_512 = 0;
double running_average_512;
uint16_t samples_collected_1024 = 0;
double running_average_1024;

uint32_t autolevel_on_at = 0;

void writeRegister(uint8_t reg, uint16_t value, bool wait = true);
uint8_t readRegister(uint8_t reg);


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

void pump_stop(bool wait = false){
  // processCommand(F("decel y150"));
  // processCommand(F("ramp_to y0.01"));
  processCommand(F("ramp_to y0"));
  if(wait) processCommand(F("wait_for_motor y"));
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

    if(skip_samples > 0){
      skip_samples--;
      return;
    }

    const uint16_t ch1_val = (((uint16_t)data[1] << 4) | ((uint16_t)data[2] >> 4));
    const uint16_t ch2_val = (((uint16_t)data[3] << 4) | ((uint16_t)data[4] >> 4));

    const int32_t delta = (int32_t)ch2_val - ch1_val - tare;

    // if(samples_collected < samples_total){
    //   running_average = ((running_average * samples_collected) + (float)delta) / (samples_collected + 1);
    //   samples_collected++;
    // }else{
    //   running_average = ((running_average * (samples_collected - 1)) + (float)delta) / samples_collected;
    // }
    // running_average = delta;
    running_average = filter.updateEstimate(delta);

    if(samples_collected_16 < 16){
      running_average_16 = ((running_average_16 * samples_collected_16) + (float)delta) / (samples_collected_16 + 1);
      samples_collected_16++;
    }else{
      running_average_16 = ((running_average_16 * (samples_collected_16 - 1)) + (float)delta) / samples_collected_16;
    }

    if(samples_collected_32 < 32){
      running_average_32 = ((running_average_32 * samples_collected_32) + (float)delta) / (samples_collected_32 + 1);
      samples_collected_32++;
    }else{
      running_average_32 = ((running_average_32 * (samples_collected_32 - 1)) + (float)delta) / samples_collected_32;
    }

    if(samples_collected_64 < 64){
      running_average_64 = ((running_average_64 * samples_collected_64) + (float)delta) / (samples_collected_64 + 1);
      samples_collected_64++;
    }else{
      running_average_64 = ((running_average_64 * (samples_collected_64 - 1)) + (float)delta) / samples_collected_64;
    }

    if(samples_collected_128 < 128){
      running_average_128 = ((running_average_128 * samples_collected_128) + (float)delta) / (samples_collected_128 + 1);
      samples_collected_128++;
    }else{
      running_average_128 = ((running_average_128 * (samples_collected_128 - 1)) + (float)delta) / samples_collected_128;
    }

    if(samples_collected_256 < 256){
      running_average_256 = ((running_average_256 * samples_collected_256) + (float)delta) / (samples_collected_256 + 1);
      samples_collected_256++;
    }else{
      running_average_256 = ((running_average_256 * (samples_collected_256 - 1)) + (float)delta) / samples_collected_256;
    }

    if(samples_collected_512 < 512){
      running_average_512 = ((running_average_512 * samples_collected_512) + (float)delta) / (samples_collected_512 + 1);
      samples_collected_512++;
    }else{
      running_average_512 = ((running_average_512 * (samples_collected_512 - 1)) + (float)delta) / samples_collected_512;
    }

    if(samples_collected_1024 < 1024){
      running_average_1024 = ((running_average_1024 * samples_collected_1024) + (float)delta) / (samples_collected_1024 + 1);
      samples_collected_1024++;
    }else{
      running_average_1024 = ((running_average_1024 * (samples_collected_1024 - 1)) + (float)delta) / samples_collected_1024;
    }

    running_average_corrected = running_average + storage.zero_offset;

    static double max_avg = 0.0;
    static uint32_t max_avg_last_change = 0;

    if(_millis >= max_avg_last_change + 3000){
      max_avg = 0.0;
    }

    if(running_average_1024 > max_avg){
      max_avg_last_change = _millis;
      max_avg = running_average_1024;
    }

    static uint32_t last_announce = 0; // 655356540;
    if(_millis >= last_announce + 100){
      // const double level_delta = running_average - storage.level_optimal;
      last_announce = _millis;

      if(abs(running_average_corrected - storage.level_optimal) > level_tolerance){
        if(running_average_corrected <= storage.level_optimal){
          // need more resin
          if(running_average_corrected <= storage.level_min){
            new_rpm = rpm_optimal;
          }else{
            const double pwr = 1.0 - ((running_average_corrected - storage.level_min) / (storage.level_optimal - storage.level_min));
            new_rpm = rpm_min + ((rpm_optimal - rpm_min) * pwr);
          }
          if(autolevel_enabled){
            if(motors[1].dir()) motors[1].dir(false);
            motors[1].target_rpm = new_rpm;
          }

        }else{
          // too much resin
          if(running_average_corrected >= storage.level_max){
            new_rpm = rpm_max;
          }else{
            const double pwr = 1.0 - ((running_average_corrected - storage.level_optimal) / (storage.level_max - storage.level_optimal));
            new_rpm = rpm_min + ((rpm_max - rpm_min) * pwr);
          }
          if(autolevel_enabled){
            if(!motors[1].dir()) motors[1].dir(true);
            motors[1].target_rpm = new_rpm;
          }

        }
      }else{
        if(autolevel_enabled){
          new_rpm = 0.0001;
          motors[1].target_rpm = new_rpm;
        }

      }

      // Serial.print(running_average_corrected); Serial.print("\t");
      // Serial.print(running_average); Serial.print("\t");
      // Serial.print(running_average_16); Serial.print("\t");
      // Serial.print(running_average_32); Serial.print("\t");
      // Serial.print(running_average_64); Serial.print("\t");
      // Serial.print(running_average_128); Serial.print("\t");
      // Serial.print(running_average_256); Serial.print("\t");
      // Serial.print(running_average_512); Serial.print("\t");
      // Serial.print(running_average_1024); Serial.print("\t");
      // Serial.print(max_avg); Serial.print("\t");
      //
      // Serial.print(running_average - storage.level_optimal); Serial.print("\t");
      // Serial.print(new_rpm); Serial.print("\t");

      // Serial.println();
    }



  }

  static uint32_t last_total_dist_calculated_at = 0;
  if(_millis >= last_total_dist_calculated_at + 50){
    last_total_dist_calculated_at = _millis;
    total_dist = (double)motors[1].steps_total / 200.0 / motors[1].usteps;
  }
}



// main menu
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


// MenuItemToggle item_auto_level(&autolevel_enabled, "Autolevel: on", "Autolevel: off", false);
bool is_auto_level_on(){ return autolevel_enabled; }
void do_auto_level_on(){
  autolevel_enabled = true;
  if(!motors[1].is_busy()){
    motors[1].rpm(10.0);
    motors[1].start(true);
  }
}
void do_auto_level_off(){
  autolevel_enabled = false;
  if(motors[1].is_busy()){
    do_pump_stop();
  }
}
const char pgmstr_auto_level_on[] PROGMEM = "Autolevel: on";
const char pgmstr_auto_level_off[] PROGMEM = "Autolevel: off";
MenuItemToggleCallable item_auto_level(&is_auto_level_on, pgmstr_auto_level_on, pgmstr_auto_level_off, &do_auto_level_off, &do_auto_level_on);

MenuItemDynamic<double> item_cap_val("Avg", running_average);
MenuItemDynamic<double> item_cap_val_corrected("AvgCor", running_average_corrected);
MenuItemDynamic<double> item_cap_val_128("Avg128", running_average_128);
MenuItemDynamic<double> item_cap_val_256("Avg256", running_average_256);
MenuItemDynamic<double> item_cap_val_512("Avg512", running_average_512);
MenuItemDynamic<double> item_cap_val_1024("Avg1024", running_average_1024);
MenuItemDynamic<double> item_new_rpm("New RPM", new_rpm);
MenuItemDynamic<double> item_distance("Dist", total_dist);





void do_calibrate_zero_offset(){
  storage.zero_offset = -running_average;
  storage.save();
  beep(10);
}
const char pgmstr_calibrate_zero_offset[] PROGMEM = "zero offset";
MenuItemCallable item_calibrate_zero_offset(pgmstr_calibrate_zero_offset, &do_calibrate_zero_offset, false);

void do_calibrate_level_optimal(){
  storage.level_min = 1.0;
  storage.level_optimal = running_average; // 4.5;
  storage.level_max = running_average + 4.5; // 9.0;
  storage.save();
  beep(10);
}
const char pgmstr_calibrate_level_optimal[] PROGMEM = "level optimal";
MenuItemCallable item_calibrate_level_optimal(pgmstr_calibrate_level_optimal, &do_calibrate_level_optimal, false);

MenuItem* const calibrate_menu_items[] PROGMEM = {
  &back,
  &item_calibrate_zero_offset,
  &item_calibrate_level_optimal,
  &item_cap_val,
  &item_cap_val_corrected,
};
Menu calibrate_menu(calibrate_menu_items, sizeof(calibrate_menu_items) / 2);
const char pgmstr_calibrate[] PROGMEM = "calibrate";
MenuItem item_calibrate_menu(pgmstr_calibrate, &calibrate_menu);


MenuItem* const main_menu_items[] PROGMEM = {
  // &item_cap_val_1024,
  // &item_cap_val_512,
  // &item_cap_val_256,
  // &item_cap_val_128,
  &item_cap_val_corrected,
  &item_new_rpm,
  &item_auto_level,
  &item_distance,
  &item_pump_start_fill,
  &item_pump_start_empty,
  &item_pump_stop,
  &motor_x,
  &motor_y,
  &item_cap_val,
  &item_calibrate_menu,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);



void setupCustom(){
  motors[1].accel = 150;
  motors[1].decel = 400;
  motors[1].stop_on_stallguard = false;
  motors[1].driver.sgt(5);
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

  main_menu.redraw_interval = 50;
  calibrate_menu.redraw_interval = 50;
}


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
  while(running_average_corrected < (storage.level_optimal - 2.3)){
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


#endif
