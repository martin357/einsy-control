#include	<inttypes.h>
#include	<Arduino.h>
#include "hardware.h"


int8_t enc_diff = 0;


void setupPins(){
  pinModeOutput(BEEPER);
  pinModeInput(BTN_EN1, true);
  pinModeInput(BTN_EN2, true);
  pinModeInput(BTN_ENC, true);
}


// void setupTimer0() {
// 	OCR0A = 0xAF;
// 	TIMSK0 |= _BV(OCIE0A);
// }
//
// ISR(TIMER0_COMPA_vect) {
// 	encoderRead();
// }

// shamelessly stolen from CW1 FW
void readEncoder() {
  static int8_t rotary_diff = 0;
  static uint8_t lcd_encoder_bits = 0;
	uint8_t enc = 0;
	if (digitalReadExt(BTN_EN1) == HIGH) {
		enc |= B01;
	}
	if (digitalReadExt(BTN_EN2) == HIGH) {
		enc |= B10;
	}
	if (enc != lcd_encoder_bits) {
		switch (enc) {
			case 0:
				if (lcd_encoder_bits == 1) {
					rotary_diff++;
				} else if (lcd_encoder_bits == 2) {
					rotary_diff--;
				}
				break;
			case 2:
				if (lcd_encoder_bits == 0) {
					rotary_diff++;
				} else if (lcd_encoder_bits == 3) {
					rotary_diff--;
				}
				break;
			case 3:
				if (lcd_encoder_bits == 2) {
					rotary_diff++;
				} else if (lcd_encoder_bits == 1) {
					rotary_diff--;
				}
				break;
			case 1:
				if (lcd_encoder_bits == 3) {
					rotary_diff++;
				} else if (lcd_encoder_bits == 0) {
					rotary_diff--;
				}
				break;
		}
		lcd_encoder_bits = enc;
		if (rotary_diff > 124)
			rotary_diff = 124;
		else if (rotary_diff < -124)
			rotary_diff = -124;

    if (rotary_diff > 3) {
  		rotary_diff -= 4;
  		enc_diff += 1;
  	} else if (rotary_diff < -3) {
  		rotary_diff += 4;
  		enc_diff -= 1;
  	}
	}
}
