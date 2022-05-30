#pragma once
#include "menu_system_basics.h"



class MenuMotor: public Menu{
public:
  MenuMotor(uint8_t, MenuItem* const*, uint8_t);
  void on_enter();
  uint8_t index;
};


class MenuListMotorMicrostepping: public MenuList<uint16_t>{
public:
  MenuListMotorMicrostepping();
  void on_enter();
  void loop();
  uint16_t value;
};


class MenuListMotorBlankTime: public MenuList<uint8_t>{
public:
  MenuListMotorBlankTime();
  void on_enter();
  void loop();
  uint8_t value;
};


class MenuListMotorCurrent: public MenuList<uint16_t>{
public:
  MenuListMotorCurrent();
  void on_enter();
  void loop();
  uint16_t value;
};


class MenuRangeMotorOffTime: public MenuRange<uint8_t>{
public:
  MenuRangeMotorOffTime();
  void on_enter();
  void loop();
  uint8_t value;
};


class MenuRangeMotorRPM: public MenuRange<uint16_t>{
public:
  MenuRangeMotorRPM();
  void on_enter();
  void loop();
  uint16_t value;
};


class MenuRangeMotorSgThreshold: public MenuRange<int8_t>{
public:
  MenuRangeMotorSgThreshold();
  void on_enter();
  void loop();
  int8_t value;
};


class MenuRangeMotorAccel: public MenuRange<uint16_t>{
public:
  MenuRangeMotorAccel();
  void on_enter();
  void loop();
  uint16_t value;
};


class MenuRangeMotorDecel: public MenuRange<uint16_t>{
public:
  MenuRangeMotorDecel();
  void on_enter();
  void loop();
  uint16_t value;
};


class MenuRangeMotorSEMIN: public MenuRange<int8_t>{
public:
  MenuRangeMotorSEMIN();
  void on_enter();
  void loop();
  int8_t value;
};


class MenuRangeMotorSEMAX: public MenuRange<int8_t>{
public:
  MenuRangeMotorSEMAX();
  void on_enter();
  void loop();
  int8_t value;
};


class MenuMotorManualSteps: public Menu{
public:
  MenuMotorManualSteps();
  void on_enter();
  void on_press(uint16_t);
  void draw(bool = true);
  void move(int8_t);
};



extern const char pgmstr_microstepping[]; // "Microstepping"
extern const char pgmstr_blank_time[]; // "Blank time"
extern const char pgmstr_current[]; // "Current"
extern const char pgmstr_off_time[]; // "Off time"
extern const char pgmstr_speed_rpm[]; // "Speed [RPM]"
extern const char pgmstr_sg_thres[]; // "Sg thres"
extern const char pgmstr_accel_rpms[]; // "Accel [RPMS]"
extern const char pgmstr_decel_rpms[]; // "Decel [RPMS]"
extern const char pgmstr_smartenergy_min[]; // "SmartEnergy Min"
extern const char pgmstr_smartenergy_max[]; // "SmartEnergy Max"
