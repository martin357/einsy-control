#pragma once
#include "src/LiquidCrystal_Prusa.h"
#include "motor.h"


// #define DEBUG_PRINT
#ifdef DEBUG_PRINT
  #define SERIAL_PRINT(x) Serial.print(x);
  #define SERIAL_PRINTLN(x) Serial.println(x);
#else
  #define SERIAL_PRINT(x) ;
  #define SERIAL_PRINTLN(x) ;
#endif


void setupPins();
void setupLcd();
void setupMotors();
void readEncoder();
void beep(uint16_t = 200);



extern int8_t enc_diff;
extern uint8_t enc_click; // 0=no_press, 1=short, 2=long
extern uint32_t beeper_off_at;
