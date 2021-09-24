#ifndef _hardware_h_
#define _hardware_h_

#include "src/LiquidCrystal_Prusa.h"
#include "src/TMCStepper.h"


// #define DEBUG_PRINT
#ifdef DEBUG_PRINT
  #define SERIAL_PRINT(x) Serial.print(x);
  #define SERIAL_PRINTLN(x) Serial.println(x);
#else
  #define SERIAL_PRINT(x) ;
  #define SERIAL_PRINTLN(x) ;
#endif

#define FSTEPS_PER_REVOLUTION 200
#define MOTORS_PRESCALER  8
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

float rpm2rps(float);
uint16_t rps2sps(float, uint16_t);
uint16_t sps2ocr(uint16_t);
uint16_t rpm2ocr(float, uint16_t);
uint16_t rps2ocr(float, uint16_t);
float ocr2rpm(uint16_t, uint16_t);
float ocr2rps(uint16_t, uint16_t);


struct MotorStallguardInfo {
  uint16_t sg_result;
  bool fsactive;
  uint8_t cs_actual;
  uint16_t rms;
};


class Motor{
public:
  Motor(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t*, uint8_t, uint16_t*, uint16_t*, uint8_t*, uint8_t);
  void on();
  void off();
  bool is_on();
  void start(bool = false);
  void stop();
  void step();
  bool dir();
  void dir(bool);
  uint16_t sg_value();
  void microsteps(uint16_t);
  void rpm(float);
  float rpm();
  void ramp_to(float, bool = false);
  void ramp_off();
  MotorStallguardInfo get_stallguard_info();
  uint8_t step_pin;
  uint8_t dir_pin;
  uint8_t enable_pin;
  uint8_t cs_pin;
  uint8_t diag_pin;
  uint8_t* step_port;
  uint8_t step_bit;
  TMC2130Stepper driver;
  uint16_t usteps;
  bool stop_on_stallguard;
  bool print_stallguard_to_serial;
  volatile bool running;
  volatile bool stallguard_triggered;
  volatile uint32_t steps_to_do;
  volatile uint32_t steps_total;

  volatile float target_rpm;
  float accel;
  float decel;

  volatile uint32_t last_speed_change;
private:
  volatile float _rpm;
  uint16_t* timer_compare_port;
  uint16_t* timer_counter_port;
  uint8_t* timer_enable_port;
  uint8_t timer_enable_bit;
};



extern int8_t enc_diff;
extern uint8_t enc_click; // 0=no_press, 1=short, 2=long
extern uint32_t beeper_off_at;
extern Motor motors[];

#endif
