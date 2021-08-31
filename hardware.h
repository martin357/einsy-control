#ifndef _hardware_h_
#define _hardware_h_

#include "pins.h"

void setupPins();
void readEncoder();

extern int8_t enc_diff;

#endif
