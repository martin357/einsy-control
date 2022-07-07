#pragma once
#include "src/LiquidCrystal_Prusa.h"
#include "motor.h"
#include "pins.h"
#include "temperature_table.h"

#define THERMISTOR_CNT  5
#define VOLTAGE_ADC_CNT 3
#define FLOAT_DECIMALS  6

#define _PRINT_VAR(v) #v
#define PRINT_VAR(v) Serial.print(F(_PRINT_VAR(v) "=")); Serial.println(v);

#define _PRINT_VARF(v) #v
#define PRINT_VARF(v) Serial.print(F(_PRINT_VARF(v) "=")); Serial.println(v, FLOAT_DECIMALS);

// #define DEBUG_SERIAL
// #define DEBUG_PRINT
#ifdef DEBUG_PRINT
  #define SERIAL_PRINT(x) Serial2.print(x);
  #define SERIAL_PRINTLN(x) Serial2.println(x);
#else
  #define SERIAL_PRINT(x) ;
  #define SERIAL_PRINTLN(x) ;
#endif


void setupPins();
void setupLcd();
void setupMotors();
void readEncoder();
void readThermistors();
void readVoltages();
void beep(uint16_t = 200);
uint32_t float_as_uint32(float);
float uint32_as_float(uint32_t);
void store_float_to_uint32(uint32_t*, const float);
float read_float_from_uint32(uint32_t*);
const char* read_pgm_string(const char*);
uint8_t read_pgm_string(const char*, char*, uint8_t);



extern int8_t enc_diff;
extern uint8_t enc_click; // 0=no_press, 1=short, 2=long
extern volatile uint32_t beeper_off_at;
extern bool read_temperature;
extern float temperature[THERMISTOR_CNT];
extern float temperature_raw[THERMISTOR_CNT];
extern bool read_voltage;
extern float voltage[VOLTAGE_ADC_CNT];
extern uint32_t last_lcd_reinit;
extern float voltage_raw[VOLTAGE_ADC_CNT];
extern const bool lcd_present;
