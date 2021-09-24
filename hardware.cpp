#include <inttypes.h>
#include <Arduino.h>
#include "hardware.h"


// extern
int8_t enc_diff = 0;
uint8_t enc_click = 0;
uint32_t beeper_off_at = 0;
#define INIT_MOTOR(i, d, n) Motor(d##_STEP_PIN, d##_DIR_PIN, d##_ENABLE_PIN, d##_TMC2130_CS, d##_TMC2130_DIAG, &PORTC, PINC##i, &OCR##n##A, &TCNT##n, &TIMSK##n, OCIE##n##A)
Motor motors[] = {
  INIT_MOTOR(0, X, 1),
  INIT_MOTOR(1, Y, 3),
  INIT_MOTOR(2, Z, 4),
  INIT_MOTOR(3, E0, 5),
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

    motors[i].driver.diag0_stall(true);
    motors[i].driver.diag1_stall(true);
    motors[i].driver.diag0_int_pushpull(true);
    motors[i].driver.diag1_pushpull(true);
    // motors[i].driver.en_pwm_mode(false);
    motors[i].driver.TCOOLTHRS(460);

    motors[i].microsteps(motors[i].usteps);
    motors[i].rpm(60.0);
  }

  #if MOTORS_PRESCALER != 8
    #error MOTORS_PRESCALER is not set to 8 !!!
  #endif

  // OCR##t##A = F_CPU / MOTORS_PRESCALER / 1000;
  // TIMSK##t |= (1 << OCIE##t##A);
  // TIMSK##t = 0;
  #define SETUP_TIMER(t) \
    TCCR##t##A = 0; \
    TCCR##t##B = 0; \
    TCNT##t  = 0; \
    TCCR##t##B |= (1 << WGM##t##2); \
    TCCR##t##B |= (1 << CS##t##1);  \
    TIMSK##t = 0;

  cli();
  // setup acceleration timer
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 0xFF;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 |= (1 << OCIE2A);

  SETUP_TIMER(1);
  SETUP_TIMER(3);
  SETUP_TIMER(4);
  SETUP_TIMER(5);

  // setup diag pin interrupt
  PCICR |= (1 << PCIE2);
  PCMSK2 = 0;
  PCMSK2 |= (1 << PCINT18); // X_DIAG
  PCMSK2 |= (1 << PCINT19); // E0_DIAG
  PCMSK2 |= (1 << PCINT22); // Z_DIAG
  PCMSK2 |= (1 << PCINT23); // Y_DIAG
  sei();

  #undef SETUP_TIMER

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
  driver(cs_pin, 0.2f),
  usteps(16),
  stop_on_stallguard(true),
  print_stallguard_to_serial(false),
  running(false),
  stallguard_triggered(false),
  steps_to_do(0),
  steps_total(0),

  target_rpm(-1.0),
  // accel(400.0),
  // decel(800.0),
  accel(40.0),
  decel(120.0),
  // accel(450.0),
  // decel(1000.0),
  // ramp_interval(30),

  last_speed_change(last_speed_change),

  _rpm(0.0),
  timer_compare_port(timer_compare_port),
  timer_counter_port(timer_counter_port),
  timer_enable_port(timer_enable_port),
  timer_enable_bit(timer_enable_bit){
    pinModeOutput(enable_pin);
    pinModeOutput(dir_pin);
    pinModeOutput(step_pin);
    pinModeOutput(cs_pin);
    pinModeInput(diag_pin, true);
    digitalWriteExt(enable_pin, HIGH);
    digitalWriteExt(dir_pin, LOW);
    digitalWriteExt(step_pin, LOW);
    digitalWriteExt(cs_pin, HIGH);
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


void Motor::start(bool start_running){
  if(!is_on()) on();
  running = start_running;
  *timer_counter_port = 0;
  *timer_enable_port |= (1 << timer_enable_bit);
  if(millis() > 300) SERIAL_PRINTLN("start");
}


void Motor::stop(){
  *timer_enable_port = 0;
  running = false;
  steps_to_do = 0;
  ramp_off();
  if(millis() > 300) SERIAL_PRINTLN("stop");
}


void Motor::step(){
  *step_port ^= 1 << step_bit;
  // delayMicroseconds(2);
  *step_port ^= 1 << step_bit;
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
    cli();
    // *timer_counter_port = 0;
    *timer_compare_port = ocr;
    sei();

    static uint32_t last_millis = 0;
    uint32_t _millis = millis();
    if(_millis > 300){
    // if(1){
      SERIAL_PRINT("new ocr=");
      SERIAL_PRINT(ocr);
      last_millis = _millis;
      // SERIAL_PRINT("\tnew rpm=");
      // SERIAL_PRINT(new_rpm);
      SERIAL_PRINTLN();
    }
  }
}


float Motor::rpm(){
  return _rpm;
}


void Motor::ramp_to(float value, bool keep_running){
  last_speed_change = millis();
  target_rpm = value;
  // target_rpm = ocr2rpm(rpm2ocr(value, usteps), usteps);
  if(!running){
    rpm(rpm()); // update timer
    start(keep_running);
  }
}


void Motor::ramp_off(){
  target_rpm = -1.0;
  last_speed_change = millis();
}


MotorStallguardInfo Motor::get_stallguard_info(){
  TMC2130_n::DRV_STATUS_t drv_status{0};
  drv_status.sr = driver.DRV_STATUS();

  MotorStallguardInfo result{0};
  result.sg_result = drv_status.sg_result;
  result.fsactive = drv_status.fsactive;
  result.cs_actual = drv_status.cs_actual;
  result.rms = driver.cs2rms(drv_status.cs_actual);
  return result;
}


// stepping timer
#define TIMER_ISR(t, m) ISR(TIMER##t##_COMPA_vect){ \
  if(motors[m].running){  \
    motors[m].step(); \
  }else if(motors[m].steps_to_do){  \
    motors[m].steps_to_do--;  \
    motors[m].step(); \
  } \
}
TIMER_ISR(1, 0)
TIMER_ISR(3, 1)
TIMER_ISR(4, 2)
TIMER_ISR(5, 3)
#undef TIMER_ISR
