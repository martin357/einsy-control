#include <inttypes.h>
#include <Arduino.h>
#include "src/LiquidCrystal_Prusa.h"
#include "hardware.h"


// extern
int8_t enc_diff = 0;
uint8_t enc_click = 0;
uint32_t beeper_off_at = 0;
Motor motors[MOTORS_MAX] = {
  Motor(X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_TMC2130_CS, X_TMC2130_DIAG, &PORTC, PINC0, &OCR1A, &TCNT1, &TIMSK1, OCIE1A),
  Motor(Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_TMC2130_CS, Y_TMC2130_DIAG, &PORTC, PINC1, &OCR3A, &TCNT3, &TIMSK3, OCIE3A),
  Motor(Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_TMC2130_CS, Z_TMC2130_DIAG, &PORTC, PINC2, &OCR4A, &TCNT4, &TIMSK4, OCIE4A),
  Motor(E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, E0_TMC2130_CS, E0_TMC2130_DIAG, &PORTC, PINC3, &OCR5A, &TCNT5, &TIMSK5, OCIE5A),
};


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

    motors[i].driver.TCOOLTHRS(0xFFFFF); // 20bit max
    motors[i].driver.THIGH(0);
    motors[i].driver.semin(5);
    motors[i].driver.semax(2);
    motors[i].driver.sedn(0b01);
    motors[i].driver.sgt(2);

    motors[i].driver.en_pwm_mode(1);      // Enable extremely quiet stepping
    motors[i].driver.pwm_autoscale(1);

    motors[i].microsteps(motors[i].usteps);
  }

  #if MOTORS_PRESCALER != 8
    #error MOTORS_PRESCALER is not set to 8 !!!
  #endif

  #define SETUP_TIMER(t) \
    TCCR##t##A = 0; \
    TCCR##t##B = 0; \
    TCNT##t  = 0; \
    OCR##t##A = F_CPU / MOTORS_PRESCALER / 1000; \
    TCCR##t##B |= (1 << WGM##t##2); \
    TCCR##t##B |= (1 << CS##t##1);  \
    // TIMSK##t |= (1 << OCIE##t##A); \
    TIMSK##t = 0;

  cli();
  SETUP_TIMER(1);
  SETUP_TIMER(3);
  SETUP_TIMER(4);
  SETUP_TIMER(5);
  sei();

  #undef SETUP_TIMER

  for(size_t i = 0; i < MOTORS_MAX; i++) motors[i].rpm(motors[i].rpm());

}


// #define MOTOR_STEP_ISR(t, m) ISR(TIMER##t##_COMPA_vect){  \
//   PORTC ^= 1 << PINC##m;  \
//   counter##t += 1; \
// }
//
// MOTOR_STEP_ISR(1, 0)
// MOTOR_STEP_ISR(3, 1)
// MOTOR_STEP_ISR(4, 2)
// MOTOR_STEP_ISR(5, 3)
//
// #undef MOTOR_STEP_ISR


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



float rpm2rps(float rpm){
  return rpm / 60;
}


uint16_t rps2sps(float rps, uint16_t usteps){
  return uint16_t(FSTEPS_PER_REVOLUTION * usteps * rps);
}


uint16_t sps2ocr(uint16_t sps){
  return F_CPU / MOTORS_PRESCALER / sps;
}


uint16_t rpm2ocr(float rpm, uint16_t usteps){
  return rps2ocr(rpm / 60, usteps);
}


uint16_t rps2ocr(float rps, uint16_t usteps){
  return F_CPU / MOTORS_PRESCALER / (FSTEPS_PER_REVOLUTION * usteps * rps);
}


float ocr2rpm(uint16_t ocr, uint16_t usteps){
  return ocr2rps(ocr, usteps) * 60;
}


float ocr2rps(uint16_t ocr, uint16_t usteps){
  return F_CPU / MOTORS_PRESCALER / ocr / usteps / FSTEPS_PER_REVOLUTION;
}



Motor::Motor(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, uint8_t cs_pin, uint8_t diag_pin,
    uint8_t* step_port, uint8_t step_bit, uint16_t* timer_compare_port, uint16_t* timer_counter_port,
    uint8_t* timer_enable_port, uint8_t timer_enable_bit):
  step_pin(step_pin),
  dir_pin(dir_pin),
  enable_pin(enable_pin),
  cs_pin(cs_pin),
  diag_pin(diag_pin),
  step_port(step_port),
  step_bit(step_bit),
  timer_compare_port(timer_compare_port),
  timer_counter_port(timer_counter_port),
  timer_enable_port(timer_enable_port),
  timer_enable_bit(timer_enable_bit),
  driver(cs_pin, 0.2f),
  usteps(32),
  running(false),
  steps_to_do(0),
  steps_total(0),

  target_rpm(-1.0),
  // accel(400.0),
  // decel(800.0),
  accel(40.0),
  decel(120.0),
  ramp_interval(30),

  do_delay(true),
  do_toggle(true),

  last_speed_change(last_speed_change),

  _rpm(0.0){
    pinModeOutput(enable_pin);
    pinModeOutput(dir_pin);
    pinModeOutput(step_pin);
    pinModeOutput(cs_pin);
    digitalWriteExt(enable_pin, HIGH);
    digitalWriteExt(dir_pin, LOW);
    digitalWriteExt(step_pin, LOW);
    digitalWriteExt(cs_pin, HIGH);
    ramp_off();
    rpm(0.0);
  }


void Motor::on(){
  digitalWriteExt(enable_pin, LOW);
  if(millis() > 300) SERIAL_PRINTLN("on");
}


void Motor::off(){
  digitalWriteExt(enable_pin, HIGH);
  if(millis() > 300) SERIAL_PRINTLN("off");
}


bool Motor::is_on(){
  return digitalReadExt(enable_pin) == LOW;
}


void Motor::start(){
  if(!is_on()) on();
  running = true;
  *timer_enable_port |= (1 << timer_enable_bit);
  if(millis() > 300) SERIAL_PRINTLN("start");
}


void Motor::stop(){
  *timer_enable_port = 0;
  running = false;
  ramp_off();
  if(millis() > 300) SERIAL_PRINTLN("stop");
}


void Motor::step(){
  // *step_port ^= 1 << step_bit;
  // *step_port ^= 1 << step_bit;
  *step_port ^= 1 << step_bit;
  if(do_delay) delayMicroseconds(2);
  if(do_toggle) *step_port ^= 1 << step_bit;
  steps_total++;
}


bool Motor::dir(){
  return digitalReadExt(dir_pin);
}


void Motor::dir(bool direction){
  digitalWriteExt(dir_pin, direction);
}


uint16_t Motor::sg_value(){
  TMC2130_n::DRV_STATUS_t drv_status{0};
  drv_status.sr = driver.DRV_STATUS();
  return drv_status.sg_result;
}


void Motor::microsteps(uint16_t microstepping){
  driver.microsteps(microstepping);
  usteps = driver.microsteps();
  // rpm(_rpm); // update timer

  if(millis() > 300){
    SERIAL_PRINT("new usteps: ");
    SERIAL_PRINTLN(usteps);
  }
}


void Motor::rpm(float value){
  _rpm = value;
  uint32_t ocr = rpm2ocr(value, usteps);
  if(ocr < 70) ocr = 70;
  if(ocr > 65535) ocr = 65535;

  if(value <= 0.0){
    stop();
    if(millis() > 300) SERIAL_PRINTLN("low rpm - stop!");
  }

  if(*timer_compare_port != ocr ){
    // SERIAL_PRINT("ocrD=");
    // SERIAL_PRINT(*timer_compare_port - ocr);
    // SERIAL_PRINT("\t");

    // *timer_counter_port -= *timer_compare_port - ocr;
    *timer_counter_port = 0;
    *timer_compare_port = ocr;

    // float new_rpm = ocr2rpm(ocr, usteps);
    // _rpm = new_rpm;

    if(millis() > 300){
      SERIAL_PRINT("new ocr=");
      SERIAL_PRINT(ocr);
      // SERIAL_PRINT("\tnew rpm=");
      // SERIAL_PRINT(new_rpm);
      SERIAL_PRINTLN();
    }
  }
}


float Motor::rpm(){
  return _rpm;
}


void Motor::ramp_to(float value){
  target_rpm = value;
  // target_rpm = ocr2rpm(rpm2ocr(value, usteps), usteps);
  last_speed_change = millis();
  if(!running) start();
}


void Motor::ramp_off(){
  target_rpm = -1.0;
  last_speed_change = millis();
}
