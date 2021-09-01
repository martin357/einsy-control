#ifndef _hardware_h_
#define _hardware_h_

#include "pins.h"

void setupPins();
void setupLcd();
void readEncoder();
void beep(uint16_t = 200);

extern int8_t enc_diff;
extern uint8_t enc_click; // 0=no_press, 1=short, 2=long
extern uint32_t beeper_off_at;

#endif
