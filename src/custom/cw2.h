#pragma once
#include "../../custom.h"

#ifdef CUSTOM_CW2
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "cw2"
  #define CUSTOM_MENU
  #define CUSTOM_SETUP
  #define CUSTOM_LOOP

  #define PIN_OUT_1 X_MIN_PIN
  #define PIN_OUT_2 Y_MIN_PIN
  #define PIN_OUT_3 Z_MIN_PIN
  #define PIN_OUT_4 PIN_FAN_0 // 3.6V?
  #define PIN_OUT_5 PIN_FAN_1 // 3.6V?
  #define PIN_OUT_6 76 // PJ5
  #define PIN_OUT_7 A8 // PK0
  #define PIN_OUT_8 SDA
  #define PIN_OUT_9 SCL
  #define PIN_OUT_10 73 // PJ3
  #define PIN_OUT_11 53 // PROC_nCS
  #define PIN_OUT_12 2 // nAC_FAULT



  #define SET_PIN(n)   PIN_##n##_PORT |= PIN_##n##_BIT
  #define RESET_PIN(n) PIN_##n##_PORT ^= PIN_##n##_BIT

  #define PIN_1_PORT  PORTB // PB6 .. X_MIN_PIN
  #define PIN_2_PORT  PORTB // PB5 .. Y_MIN_PIN
  #define PIN_3_PORT  PORTB // PB4 .. Z_MIN_PIN
  #define PIN_4_PORT  PORTH // PH5 .. PIN_FAN_0 .. 3.6V?
  #define PIN_5_PORT  PORTH // PH3 .. PIN_FAN_1 .. 3.6V?
  #define PIN_6_PORT  PORTJ // PJ5
  #define PIN_7_PORT  PORTK // PK0 .. A8
  #define PIN_8_PORT  PORTD // PD1 .. SDA
  #define PIN_9_PORT  PORTD // PD0 .. SCL
  #define PIN_10_PORT PORTJ // PJ3
  #define PIN_11_PORT PORTB // PB0 .. PROC_nCS
  #define PIN_12_PORT PORTE // PE4 .. nAC_FAULT

  #define PIN_1_BIT  _BV(6)
  #define PIN_2_BIT  _BV(5)
  #define PIN_3_BIT  _BV(4)
  #define PIN_4_BIT  _BV(5)
  #define PIN_5_BIT  _BV(3)
  #define PIN_6_BIT  _BV(5)
  #define PIN_7_BIT  _BV(0)
  #define PIN_8_BIT  _BV(1)
  #define PIN_9_BIT  _BV(0)
  #define PIN_10_BIT _BV(3)
  #define PIN_11_BIT _BV(0)
  #define PIN_12_BIT _BV(4)

#endif
