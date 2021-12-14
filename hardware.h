#pragma once
#include "src/LiquidCrystal_Prusa.h"
#include "motor.h"
#include "pins.h"

#define _PRINT_VAR(v) #v
#define PRINT_VAR(v) Serial.print(F(_PRINT_VAR(v) "=")); Serial.println(v);

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
uint32_t float_as_uint32(float);
float uint32_as_float(uint32_t);
void store_float_to_uint32(uint32_t*, const float);
float read_float_from_uint32(uint32_t*);



extern int8_t enc_diff;
extern uint8_t enc_click; // 0=no_press, 1=short, 2=long
extern uint32_t beeper_off_at;
extern const bool lcd_present;
