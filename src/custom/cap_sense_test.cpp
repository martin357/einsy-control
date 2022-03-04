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

uint8_t skip_samples = 64;
bool print_full_data = false;
ad7150_reg_setup ch_setup;
ad7150_reg_capdac ch1_capdac, ch2_capdac;
ad7150_reg_configuration configuration;
int32_t initial_tare = 0;
int32_t tare = 0;
uint8_t samples_total = 6; // *32;
uint8_t samples_collected = 0;
float running_average;

uint16_t samples_collected_16 = 0;
float running_average_16;
uint16_t samples_collected_32 = 0;
float running_average_32;
uint16_t samples_collected_64 = 0;
float running_average_64;
uint16_t samples_collected_128 = 0;
float running_average_128;
uint16_t samples_collected_256 = 0;
float running_average_256;
uint16_t samples_collected_512 = 0;
float running_average_512;
uint16_t samples_collected_1024 = 0;
float running_average_1024;

void writeRegister(uint8_t reg, uint16_t value, bool wait = true);
uint8_t readRegister(uint8_t reg);


void pump_start(bool dir){
  processCommand(F("halt y"));
  processCommand(F("empty_queue y"));
  processCommand(dir ? F("dir y0") : F("dir y1"));
  processCommand(F("accel y150"));
  processCommand(F("rpm y0.01"));
  processCommand(F("start y1"));
  processCommand(F("ramp_to y150"));
}

void pump_stop(bool wait = false){
  processCommand(F("decel y150"));
  // processCommand(F("ramp_to y0.01"));
  processCommand(F("ramp_to y0"));
  if(wait) processCommand(F("wait_for_motor y"));
}


void loopCustom(){
  const uint32_t _millis = millis();
  Wire.requestFrom(AD7150_I2C_ADDRESS, 1);
  const ad7150_reg_status status{.reg = (uint8_t)Wire.read()};

  if(!status.RDY1 && !status.RDY2){
    uint8_t data[5] = {0};

    Wire.requestFrom(AD7150_I2C_ADDRESS, 5);
    for (int i = 0; i < 5; i++){
      while(!Wire.available());
      data[i] = Wire.read();
    }
    // writeRegister(AD7150_REG_CONFIGURATION, MODE_POWER_DOWN);

    if(skip_samples > 0){
      skip_samples--;
      return;
    }

    const uint16_t ch1_val = (((uint16_t)data[1] << 4) | ((uint16_t)data[2] >> 4));
    const uint16_t ch2_val = (((uint16_t)data[3] << 4) | ((uint16_t)data[4] >> 4));
    // const uint16_t ch1_val = (((uint16_t)data[1] << 4) | data[2]) >> 4;
    // const uint16_t ch2_val = (((uint16_t)data[3] << 4) | data[4]) >> 4;
    //const uint16_t ch1_val = data[1] << 4 | data[2] >> 4;
    //const uint16_t ch2_val = data[3] << 4 | data[4] >> 4;

    // Serial.print(data[0], BIN);
    // Serial.print("\t");

    if(initial_tare == 0) initial_tare = (int32_t)ch2_val - ch1_val;
    //if(tare == 0) tare = (int32_t)ch2_val - ch1_val;
    const int32_t delta = (int32_t)ch2_val - ch1_val - tare;
    //const int32_t delta = (int32_t)ch2_val - ch1_val;
    //const int32_t delta = (int32_t)ch1_val - ch2_val;

    if(samples_collected < samples_total){
      running_average = ((running_average * samples_collected) + (float)delta) / (samples_collected + 1);
      samples_collected++;
    }else{
      running_average = ((running_average * (samples_collected - 1)) + (float)delta) / samples_collected;
    }

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

    // const int32_t abs_delta = abs((int32_t)ch2_val - ch1_val);
    // const bool is_triggered = delta > 30; // abs_delta > 60;
    // const bool is_triggered = delta > 30; // abs_delta > 60;
    // static bool old_is_triggered = !is_triggered;

    static ResinLevel old_resin_level = UNKNOWN;
    static uint32_t last_resin_level_change = 0;
    // ResinLevel resin_level = NORMAL;
    // if(running_average_1024 < 26.0) resin_level = BELOW;
    // else if(running_average_1024 > 30.0) resin_level = ABOVE;

    // #define RESIN_LEVEL_MIN 26.0
    // #define RESIN_LEVEL_MAX 30.0
    #define RESIN_LEVEL_MIN 38.0
    #define RESIN_LEVEL_MAX 40.0

    //const ResinLevel resin_level = running_average_1024 < 25.0 ? BELOW : (running_average_1024 > 32.0 ? ABOVE : NORMAL);
    const ResinLevel resin_level = running_average_1024 < RESIN_LEVEL_MIN ? BELOW : (running_average_1024 > RESIN_LEVEL_MAX ? ABOVE : NORMAL);

    static double max_avg = 0.0;
    static uint32_t max_avg_last_change = 0;

    if(_millis >= max_avg_last_change + 3000){
      max_avg = 0.0;
    }

    // if(running_average > max_avg){
    //   max_avg_last_change = _millis;
    //   max_avg = running_average;
    // }

    if(running_average_1024 > max_avg){
      max_avg_last_change = _millis;
      max_avg = running_average_1024;
    }

    static uint32_t last_announce = 0; // 655356540;
    if(_millis >= last_announce + 10){
      last_announce = _millis;
      // Serial.print(running_average); Serial.print("\t");
      // Serial.print(running_average_16); Serial.print("\t");
      // Serial.print(running_average_32); Serial.print("\t");
      // Serial.print(running_average_64); Serial.print("\t");
      // Serial.print(running_average_128); Serial.print("\t");
      Serial.print(running_average_256); Serial.print("\t");
      Serial.print(running_average_512); Serial.print("\t");
      Serial.print(running_average_1024); Serial.print("\t");

      Serial.print(max_avg);
      Serial.println();
    }

    if(resin_level != old_resin_level){
      last_resin_level_change = _millis;
      old_resin_level = resin_level;
      // if(resin_level == BELOW){
      //   Serial.println("resin level BELOW threshold!");
      // }else if(resin_level == ABOVE){
      //   Serial.println("resin level ABOVE threshold!");
      // }else if(resin_level == NORMAL){
      //   Serial.println("resin level normal");
      // }else if(resin_level == UNKNOWN){
      //   Serial.println("resin level UNKNOWN!!");
      // }
    }

    if(false && last_resin_level_change > 0 && _millis >= last_resin_level_change + /* 300 */ 700){
      last_resin_level_change = 0;
      // Serial.println("reacting to resin level change...");
      if(resin_level == BELOW){
        // Serial.println("    resin level BELOW threshold!");
        pump_start(true);

      }else if(resin_level == ABOVE){
        // Serial.println("    resin level ABOVE threshold!");
        pump_start(false);

      }else if(resin_level == NORMAL){
        // Serial.println("    resin level normal");
        pump_stop();

      }
    }




    // if(resin_level != old_resin_level){
    //   old_resin_level = resin_level;
    //   if(resin_level == BELOW){
    //     Serial.println("resin level BELOW threshold!");
    //     pump_start(true);
    //
    //   }else if(resin_level == ABOVE){
    //     Serial.println("resin level ABOVE threshold!");
    //     pump_start(false);
    //
    //   }else if(resin_level == NORMAL){
    //     Serial.println("resin level normal");
    //     pump_stop();
    //
    //   }
    // }

    // if(is_triggered != old_is_triggered){
    //   old_is_triggered = is_triggered;
    //   Serial.print("is_triggered=");
    //   Serial.println(is_triggered ? "true" : "false");
    //
    //   if(is_triggered){
    //     pump_stop(true);
    //     pump_start(false);
    //
    //   }else{
    //     pump_stop(true);
    //     pump_start(true);
    //
    //   }
    // }

//    digitalWrite(LED_BUILTIN, is_triggered);

//    Serial.print("0\t255\t");
//
//    if(print_full_data){
     // Serial.print(running_average);
//      Serial.print("\t");
//
//      Serial.print(ch2_val);
//      Serial.print("\t");
//
//      Serial.print(ch1_val);
//      Serial.print("\t");
//
//      //Serial.print((int32_t)ch2_val - ch1_val);
//      Serial.print(delta);
//      Serial.print("\t");
//      // Serial.print((int32_t)max(ch1_val, ch2_val) - min(ch1_val, ch2_val));
//
////      Serial.print(abs_delta);
////      Serial.print("\t");
//
//      Serial.print(is_triggered ? "\t1024" : "\t0");
//
     // Serial.println();
//
//    }else{
//      //Serial.println((int32_t)ch2_val - ch1_val);
//      Serial.print(running_average);
//      //Serial.print(delta);
//      Serial.println(is_triggered ? "\t128" : "\t0");
//
//    }

    // if(!status.DacStep2){
    //   ch2_capdac.reg = readRegister(AD7150_REG_CH2_CAPDAC);
    //   ch1_capdac.DacValue = ch2_capdac.DacValue;
    //   writeRegister(AD7150_REG_CH1_CAPDAC, ch1_capdac.reg, false);
    //
    //   // Serial.print(ch2_capdac.DacValue);
    //   // Serial.print("\t");
    //   //
    //   // Serial.print(!status.DacStep1 ? "TRUE  " : "false ");
    //   // Serial.print(!status.DacStep2 ? "TRUE " : "false");
    //   // Serial.print("\t");
    // }

    // delay(10);
    // writeRegister(AD7150_REG_CONFIGURATION, MODE_SINGLE_CONV);
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
  pump_start(false);
}
const char pgmstr_pump_start_empty[] PROGMEM = "pump_start_empty";
MenuItemCallable item_pump_start_empty(pgmstr_pump_start_empty, &do_pump_start_empty, false);


MenuItem* const main_menu_items[] PROGMEM = {
  &item_pump_start_fill,
  &item_pump_start_empty,
  &item_pump_stop,
  &motor_x,
  &motor_y,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);



void setupCustom(){
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


#endif
