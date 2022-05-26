#include "fan_control.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#include "../../serial.h"
#ifdef CUSTOM_FAN_CONTROL

#include "../kalmanfilter.h"

KalmanFilter filter(1, 1, 0.01);
float filtered = 0.0;



void setupCustom(){
  // enable heater_0 pwm
  TCCR3A = (1 << COM3C1) | (1 << WGM30);
  TCCR3B = (1 << CS31) | (1 << CS30);
  OCR3C = 0; // 40;

  pinMode(A8, INPUT);
}


uint32_t last_pwm_update = 0;
void loopCustom(){
  const uint32_t _millis = millis();

  const uint16_t pot_value = analogRead(A8);
  filtered = filter.updateEstimate(pot_value);
  const uint16_t pwm_value = map(round(filtered), 0, 1023, 0, 128);

  if(_millis >= last_pwm_update + 100){
    last_pwm_update = _millis;
    OCR3C = pwm_value;
  }
}



bool heater_fan_enabled = false;
bool is_heater_fan_on(){ return heater_fan_enabled; }
void do_heater_fan_on(){
  OCR3C = 40;
  heater_fan_enabled = true;
}
void do_heater_fan_off(){
  OCR3C = 0;
  heater_fan_enabled = false;
}
const char pgmstr_heater_fan_on[] PROGMEM = "Heater fan: on";
const char pgmstr_heater_fan_off[] PROGMEM = "Heater fan: off";
MenuItemToggleCallable item_heater_fan(&is_heater_fan_on, pgmstr_heater_fan_on, pgmstr_heater_fan_off, &do_heater_fan_off, &do_heater_fan_on);



bool bed_fan_enabled = false;
bool is_bed_fan_on(){ return bed_fan_enabled; }
void do_bed_fan_on(){
  bed_fan_enabled = true;
  analogWrite(HEATER_0_PIN, 40);
}
void do_bed_fan_off(){
  bed_fan_enabled = false;
  digitalWriteExt(HEATER_0_PIN, LOW);
}
const char pgmstr_bed_fan_on[] PROGMEM = "Bed fan: on";
const char pgmstr_bed_fan_off[] PROGMEM = "Bed fan: off";
MenuItemToggleCallable item_bed_fan(&is_bed_fan_on, pgmstr_bed_fan_on, pgmstr_bed_fan_off, &do_bed_fan_off, &do_bed_fan_on);



const char pgmstr_timer3c[] PROGMEM = "Speed";
uint16_t* timer_ptr3c = &OCR3C;
MenuRange<uint16_t> menu_timer3c("Speed", *timer_ptr3c, 0, 128);
MenuItem timer3c(pgmstr_timer3c, &menu_timer3c);



MenuItem* const main_menu_items[] PROGMEM = {
  &item_heater_fan,
  &item_bed_fan,
  &separator,
  &timer3c,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);



#endif
