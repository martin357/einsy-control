#ifndef _hardware_h_
#define _hardware_h_

#include "src/TMCStepper.h"
// #include "pins.h"

#define MOTORS_MAX  4
#define MOTOR_X 0
#define MOTOR_Y 1
#define MOTOR_Z 2
#define MOTOR_E 3

void setupPins();
void setupLcd();
void setupMotors();
void readEncoder();
void beep(uint16_t = 200);


class Motor{
public:
  Motor(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
  void on();
  void off();
  bool is_on();
  void start();
  void stop();
  bool dir();
  void dir(bool);
  uint16_t sg_value();
  uint8_t step_pin;
  uint8_t dir_pin;
  uint8_t enable_pin;
  uint8_t cs_pin;
  uint8_t diag_pin;
  TMC2130Stepper driver;
  bool running;
};



extern int8_t enc_diff;
extern uint8_t enc_click; // 0=no_press, 1=short, 2=long
extern uint32_t beeper_off_at;
extern Motor motors[];

#endif
