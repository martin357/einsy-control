#ifndef _menu_system_derivates_h_
#define _menu_system_derivates_h_

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


#endif
