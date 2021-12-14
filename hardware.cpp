#include "hardware.h"


// extern
bool _is_lcd_present(){
  pinModeOutput(LCD_PINS_RS);
  digitalWriteExt(LCD_PINS_RS, LOW);
  pinModeInput(LCD_PINS_RS);
  bool _lcd_present = digitalReadExt(LCD_PINS_RS);
  pinModeOutput(LCD_PINS_RS);
  digitalWriteExt(LCD_PINS_RS, LOW);
  return _lcd_present;
}

int8_t enc_diff = 0;
uint8_t enc_click = 0;
uint32_t beeper_off_at = 0;
const bool lcd_present = _is_lcd_present();
#define INIT_MOTOR(i, d, n, a) Motor(d##_STEP_PIN, d##_DIR_PIN, d##_ENABLE_PIN, d##_TMC2130_CS, d##_TMC2130_DIAG, &PORTC, PINC##i, &OCR##n##A, &TCNT##n, &TIMSK##n, OCIE##n##A, a)
Motor motors[] = {
  INIT_MOTOR(0, X, 1, 'x'),
  INIT_MOTOR(1, Y, 3, 'y'),
  INIT_MOTOR(2, Z, 4, 'z'),
  INIT_MOTOR(3, E0, 5, 'e'),
};
#undef INIT_MOTOR


const uint8_t Back[8] PROGMEM = {
	B00100,
	B01110,
	B11111,
	B00100,
	B11100,
	B00000,
	B00000,
	B00000
};

const uint8_t Right[8] PROGMEM = {
	B00000,
	B00100,
	B00010,
	B11111,
	B00010,
	B00100,
	B00000,
	B00000
};

const uint8_t Backslash[8] PROGMEM = {
	B00000,
	B10000,
	B01000,
	B00100,
	B00010,
	B00001,
	B00000,
	B00000
};

const uint8_t Play[8] PROGMEM = {
	B00000,
	B01000,
	B01100,
	B01110,
	B01100,
	B01000,
	B00000,
	B00000
};

const uint8_t Stop[8] PROGMEM = {
	B00000,
	B10001,
	B01010,
	B00100,
	B01010,
	B10001,
	B00000,
	B00000
};



void setupPins(){
  // encoder
  pinModeInput(BTN_EN1, true);
  pinModeInput(BTN_EN2, true);
  pinModeInput(BTN_ENC, true);

  // misc
  pinModeInput(MISO, true);
  pinModeOutput(BEEPER);
}


void setupLcd(){
  lcd.setBrightness(128);
  lcd.clear();

  lcd.createChar(0, Backslash);
	lcd.createChar(1, Back);
	lcd.createChar(2, Right);
	lcd.createChar(3, Play);
	lcd.createChar(4, Stop);
}


void setupMotors(){
  for(size_t i = 0; i < MOTORS_MAX; i++) {
    motors[i].driver.begin();
    motors[i].driver.toff(2);
    motors[i].driver.blank_time(24);
    motors[i].driver.rms_current(300); // mA

    // motors[i].driver.TCOOLTHRS(0xFFFFF); // 20bit max
    motors[i].driver.THIGH(0);
    motors[i].driver.semin(5);
    motors[i].driver.semax(2);
    motors[i].driver.sedn(0b01);
    motors[i].driver.sgt(2);

    motors[i].driver.en_pwm_mode(1);      // Enable extremely quiet stepping
    motors[i].driver.pwm_autoscale(1);
    motors[i].driver.intpol(true);

    motors[i].driver.diag0_stall(true);
    motors[i].driver.diag1_stall(true);
    motors[i].driver.diag0_int_pushpull(true);
    motors[i].driver.diag1_pushpull(true);
    // motors[i].driver.en_pwm_mode(false);
    motors[i].driver.TCOOLTHRS(460);

    motors[i].microsteps(motors[i].usteps);
    motors[i].dir(false);
    motors[i].rpm(60.0);
  }

  setupMotorTimers();

}



// shamelessly stolen from CW1 FW
void readEncoder() {
  static int8_t rotary_diff = 0;
  static uint8_t lcd_encoder_bits = 0;
  static uint8_t prev_click_state = 0;
  static uint32_t last_click = 0;
	uint8_t enc = 0;
	uint8_t click = !digitalReadExt(BTN_ENC);
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
  if(click != prev_click_state){
    uint32_t _millis = millis();
    if(!click) enc_click = _millis - last_click > 350 ? 2 : 1;
    last_click = _millis;
    prev_click_state = click;
  }
}



void beep(uint16_t duration){
  digitalWriteExt(BEEPER, HIGH);
  beeper_off_at = millis() + duration;
}



uint32_t float_as_uint32(float value){
  return *reinterpret_cast<uint32_t*>(&value);
}



float uint32_as_float(uint32_t value){
  return *reinterpret_cast<float*>(&value);
}



void store_float_to_uint32(uint32_t *target, const float value){
  float* view = reinterpret_cast<float*>(*&target);
  *view = value;
}



float read_float_from_uint32(uint32_t *source){
  float* view = reinterpret_cast<float*>(*&source);
  return *view;
}
