#include <Arduino.h>
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
  menu list motor current
*/
uint16_t motor_current_items[32] = {0};
MenuListMotorCurrent::MenuListMotorCurrent():
  MenuList("Current", &value, motor_current_items, sizeof(motor_current_items) / sizeof(motor_current_items[0])),
  value(1){}


void MenuListMotorCurrent::on_enter(){
  const size_t items_cnt = sizeof(motor_current_items) / sizeof(motor_current_items[0]);
  for (size_t i = 0; i < items_cnt; i++) items[i] = motors[last_entered_motor_menu].driver.cs2rms(i);
  value = motors[last_entered_motor_menu].driver.rms_current();
  MenuList::on_enter();
}


void MenuListMotorCurrent::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    // if(value != motors[last_entered_motor_menu].driver.rms_current()) motors[last_entered_motor_menu].driver.rms_current(value);
    if(value != motors[0].driver.rms_current()){
      for (size_t i = 0; i < MOTORS_MAX; i++) motors[i].driver.rms_current(value);
    }
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
  // value = motors[last_entered_motor_menu].driver.toff();
  value = motors[0].driver.toff();
}


void MenuRangeMotorOffTime::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    // if(value != motors[last_entered_motor_menu].driver.toff()) motors[last_entered_motor_menu].driver.toff(value);
    if(value != motors[0].driver.toff()){
      for (size_t i = 0; i < MOTORS_MAX; i++) motors[i].driver.toff(value);
    }
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
  // value = motors[last_entered_motor_menu].rpm();
  value = motors[0].rpm();
}


void MenuRangeMotorRPM::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    // if(value != motors[last_entered_motor_menu].rpm()){
    if(value != motors[0].rpm()){
      // motors[last_entered_motor_menu].target_rpm = -1.0;
      // motors[last_entered_motor_menu].rpm(value);
      for (size_t i = 0; i < MOTORS_MAX; i++) {
        motors[i].target_rpm = -1.0;
        motors[i].rpm(value);
      }
    }
    last_loop = _millis;
  }
}



/*
  menu range motor sg threshold
*/
MenuRangeMotorSgThreshold::MenuRangeMotorSgThreshold():
  MenuRange("Sg thres", value, -63, 63),
  value(2){}


void MenuRangeMotorSgThreshold::on_enter(){
  MenuRange::on_enter();
  value = motors[last_entered_motor_menu].driver.sgt();
}


void MenuRangeMotorSgThreshold::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    // if(value != motors[last_entered_motor_menu].driver.sgt()) motors[last_entered_motor_menu].driver.sgt(value);
    if(value != motors[0].driver.sgt()){
      for (size_t i = 0; i < MOTORS_MAX; i++) motors[i].driver.sgt(value);
    }
    last_loop = _millis;
  }
}



/*
  menu range motor accel
*/
MenuRangeMotorAccel::MenuRangeMotorAccel():
  MenuRange("Accel [RPMS]", value, 1, 1200),
  value(1){}


void MenuRangeMotorAccel::on_enter(){
  MenuRange::on_enter();
  value = motors[last_entered_motor_menu].accel;
}


void MenuRangeMotorAccel::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    if(value != motors[last_entered_motor_menu].accel) motors[last_entered_motor_menu].accel = value;
    last_loop = _millis;
  }
}



/*
  menu range motor decel
*/
MenuRangeMotorDecel::MenuRangeMotorDecel():
  MenuRange("Decel [RPMS]", value, 1, 1200),
  value(1){}


void MenuRangeMotorDecel::on_enter(){
  MenuRange::on_enter();
  value = motors[last_entered_motor_menu].decel;
}


void MenuRangeMotorDecel::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    if(value != motors[last_entered_motor_menu].decel) motors[last_entered_motor_menu].decel = value;
    last_loop = _millis;
  }
}



/*
  menu range motor semin
*/
MenuRangeMotorSEMIN::MenuRangeMotorSEMIN():
  MenuRange("SmartEnergy Min", value, 0, 15),
  value(1){}


void MenuRangeMotorSEMIN::on_enter(){
  MenuRange::on_enter();
  value = motors[last_entered_motor_menu].driver.semin();
}


void MenuRangeMotorSEMIN::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    if(value != motors[last_entered_motor_menu].driver.semin()) motors[last_entered_motor_menu].driver.semin(value);
    last_loop = _millis;
  }
}



/*
  menu range motor semax
*/
MenuRangeMotorSEMAX::MenuRangeMotorSEMAX():
  MenuRange("SmartEnergy Max", value, 0, 15),
  value(1){}


void MenuRangeMotorSEMAX::on_enter(){
  MenuRange::on_enter();
  value = motors[last_entered_motor_menu].driver.semax();
}


void MenuRangeMotorSEMAX::loop(){
  static uint32_t last_loop = 0;
  uint32_t _millis = millis();
  if(_millis > last_loop + 50){
    if(value != motors[last_entered_motor_menu].driver.semax()) motors[last_entered_motor_menu].driver.semax(value);
    last_loop = _millis;
  }
}
