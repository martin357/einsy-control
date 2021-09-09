// #include <avr/pgmspace.h>
#include <Arduino.h>
// #include "src/LiquidCrystal_Prusa.h"
#include "menu_system_derivates.h"
#include "hardware.h"



/*
  menu motor
*/
MenuMotor::MenuMotor(uint8_t index, MenuItem* const* items, uint8_t items_count):
  Menu(items, items_count),
  index(index){}


void MenuMotor::on_enter(){
  last_entered_motor_menu = index;
}



/*
  menu list motor microstepping
*/
uint16_t motor_microstepping_items[] = {0, 2, 4, 8, 16, 32, 64, 128, 256};
MenuListMotorMicrostepping::MenuListMotorMicrostepping():
  MenuList("Microstepping", &value, motor_microstepping_items, sizeof(motor_microstepping_items) / sizeof(motor_microstepping_items[0])),
  value(64){}


void MenuListMotorMicrostepping::on_enter(){
  value = motors[last_entered_motor_menu].usteps;
  MenuList::on_enter();
}


void MenuListMotorMicrostepping::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    if(value != motors[last_entered_motor_menu].usteps) motors[last_entered_motor_menu].microsteps(value);
    last_loop = _millis;
  }
}



/*
  menu list motor blank time
*/
uint8_t motor_blank_time_items[] = {16, 24, 36, 54};
MenuListMotorBlankTime::MenuListMotorBlankTime():
  MenuList("Blank time", &value, motor_blank_time_items, sizeof(motor_blank_time_items) / sizeof(motor_blank_time_items[0])),
  value(24){}


void MenuListMotorBlankTime::on_enter(){
  value = motors[last_entered_motor_menu].driver.blank_time();
  MenuList::on_enter();
}


void MenuListMotorBlankTime::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    if(value != motors[last_entered_motor_menu].driver.blank_time()) motors[last_entered_motor_menu].driver.blank_time(value);
    last_loop = _millis;
  }
}



/*
  menu range motor off time
*/
MenuRangeMotorOffTime::MenuRangeMotorOffTime():
  MenuRange("Off time", value, 1, 15),
  value(1){}


void MenuRangeMotorOffTime::on_enter(){
  MenuRange::on_enter();
  value = motors[last_entered_motor_menu].driver.toff();
}


void MenuRangeMotorOffTime::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    if(value != motors[last_entered_motor_menu].driver.toff()) motors[last_entered_motor_menu].driver.toff(value);
    last_loop = _millis;
  }
}



/*
  menu range motor rpm
*/
MenuRangeMotorRPM::MenuRangeMotorRPM():
  MenuRange("Speed [RPM]", value, 1, 460),
  value(1){}


void MenuRangeMotorRPM::on_enter(){
  MenuRange::on_enter();
  value = motors[last_entered_motor_menu].rpm();
}


void MenuRangeMotorRPM::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    if(value != motors[last_entered_motor_menu].rpm()){
      motors[last_entered_motor_menu].target_rpm = -1.0;
      motors[last_entered_motor_menu].rpm(value);
    }
    last_loop = _millis;
  }
}
