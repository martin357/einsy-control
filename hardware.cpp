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
volatile uint32_t beeper_off_at = 0;
bool read_temperature = true;
uint8_t temperature_samples_collected = 0;
float temperature_raw[THERMISTOR_CNT] = {0};
float temperature[THERMISTOR_CNT] = {0};
bool read_voltage = true;
uint8_t voltage_samples_collected = 0;
float voltage_raw[VOLTAGE_ADC_CNT] = {0};
float voltage[VOLTAGE_ADC_CNT] = {0};
uint32_t last_lcd_reinit = 0;
const bool lcd_present = _is_lcd_present();
const uint8_t temperature_samples_total = 64;
const uint8_t voltage_samples_total = 16;
#define INIT_MOTOR(i, d, n, j, a) Motor(d##_STEP_PIN, d##_DIR_PIN, d##_ENABLE_PIN, d##_TMC2130_CS, d##_TMC2130_DIAG, &PORTC, PINC##i, &PORTL, PINL##j, &OCR##n##A, &TCNT##n, &TIMSK##n, OCIE##n##A, a)
Motor motors[] = {
  INIT_MOTOR(0, X, 1, 0, 'x'),
  INIT_MOTOR(1, Y, 3, 1, 'y'),
  INIT_MOTOR(2, Z, 4, 2, 'z'),
  INIT_MOTOR(3, E0, 5, 6, 'e'),
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

  // thermistors
  pinModeInput(TEMP_0_PIN);
  pinModeInput(TEMP_1_PIN);
  pinModeInput(TEMP_2_PIN);
  pinModeInput(TEMP_PINDA_PIN);
  pinModeInput(TEMP_AMBIENT_PIN);

  // power
  pinModeOutput(HEATER_BED_PIN);
  digitalWriteExt(HEATER_BED_PIN, LOW);
  pinModeOutput(HEATER_0_PIN);
  digitalWriteExt(HEATER_0_PIN, LOW);

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


float _analog_to_temp(int raw){
  float celsius = 0;
  byte i;

  for (i=1; i<temperature_table_einsy_len; i++){
    if ((short)pgm_read_word(&temperature_table_einsy[i][0]) > raw){
      celsius = (short)pgm_read_word(&temperature_table_einsy[i-1][1]) +
        (raw - (short)pgm_read_word(&temperature_table_einsy[i-1][0])) *
        (float)((short)pgm_read_word(&temperature_table_einsy[i][1]) - (short)pgm_read_word(&temperature_table_einsy[i-1][1])) /
        (float)((short)pgm_read_word(&temperature_table_einsy[i][0]) - (short)pgm_read_word(&temperature_table_einsy[i-1][0]));
      break;
    }
  }

  if (i == temperature_table_einsy_len) celsius = (short)pgm_read_word(&temperature_table_einsy[i-1][1]);

	const float _offset = 10;
	const float _offset_center = 50;
	const float _offset_start = 40;
	const float _first_koef = (_offset / 2) / (_offset_center - _offset_start);
	const float _second_koef = (_offset / 2) / (100 - _offset_center);

	if (celsius >= _offset_start && celsius <= _offset_center){
		celsius = celsius + (_first_koef * (celsius - _offset_start));
	}else if (celsius > _offset_center && celsius <= 100){
		celsius = celsius + (_first_koef * (_offset_center - _offset_start)) + (_second_koef * (celsius - (100 - _offset_center)));
	}else if (celsius > 100){
		celsius = celsius + _offset;
	}

  return celsius;
}


float _analog_to_temp_ambient(int raw){
  float celsius = 0;
  byte i;

  for (i=1; i<temperature_table_einsy_ambient_len; i++){
    if ((short)pgm_read_word(&temperature_table_einsy_ambient[i][0]) > raw){
      celsius = (short)pgm_read_word(&temperature_table_einsy_ambient[i-1][1]) +
        (raw - (short)pgm_read_word(&temperature_table_einsy_ambient[i-1][0])) *
        (float)((short)pgm_read_word(&temperature_table_einsy_ambient[i][1]) - (short)pgm_read_word(&temperature_table_einsy_ambient[i-1][1])) /
        (float)((short)pgm_read_word(&temperature_table_einsy_ambient[i][0]) - (short)pgm_read_word(&temperature_table_einsy_ambient[i-1][0]));
      break;
    }
  }

  if (i == temperature_table_einsy_ambient_len) celsius = (short)pgm_read_word(&temperature_table_einsy_ambient[i-1][1]);

  return celsius;
}



void readThermistors(){
  for (size_t i = 0; i < THERMISTOR_CNT; i++) {
    uint16_t uval;
    switch (i) {
      case 0: uval = analogRead(TEMP_0_PIN); break;
      case 1: uval = analogRead(TEMP_1_PIN); break;
      case 2: uval = analogRead(TEMP_2_PIN); break;
      case 3: uval = analogRead(TEMP_PINDA_PIN); break;
      case 4: uval = analogRead(TEMP_AMBIENT_PIN); break;
    }

    if(temperature_samples_collected < temperature_samples_total){
      temperature_raw[i] = ((temperature_raw[i] * temperature_samples_collected) + (float)uval) / (temperature_samples_collected + 1);
    }else{
      temperature_raw[i] = ((temperature_raw[i] * (temperature_samples_collected - 1)) + (float)uval) / temperature_samples_collected;
    }
    if(i == 4) temperature[i] = _analog_to_temp_ambient(temperature_raw[i]);
    else temperature[i] = _analog_to_temp(temperature_raw[i]);
  }
  if(temperature_samples_collected < temperature_samples_total) temperature_samples_collected++;
}


#define VOLT_DIV_REF 5
#define VOLT_DIV_R1 10000
#define VOLT_DIV_R2 2370
#define VOLT_DIV_FAC ((float)VOLT_DIV_R2 / (VOLT_DIV_R2 + VOLT_DIV_R1))

void readVoltages(){
  for (size_t i = 0; i < VOLTAGE_ADC_CNT; i++) {
    uint16_t uval;
    switch (i) {
      case 0: uval = analogRead(VOLT_PWR_PIN); break;
      case 1: uval = analogRead(VOLT_BED_PIN); break;
      case 2: uval = analogRead(VOLT_IR_PIN); break;
    }

    if(voltage_samples_collected < voltage_samples_total){
      voltage_raw[i] = ((voltage_raw[i] * voltage_samples_collected) + (float)uval) / (voltage_samples_collected + 1);
    }else{
      voltage_raw[i] = ((voltage_raw[i] * (voltage_samples_collected - 1)) + (float)uval) / voltage_samples_collected;
    }
    if(i == 2) voltage[i] = VOLT_DIV_REF * voltage_raw[i] / 1023;
    else voltage[i] = VOLT_DIV_REF * voltage_raw[i] / 1023 / VOLT_DIV_FAC;

  }
  if(voltage_samples_collected < voltage_samples_total) voltage_samples_collected++;
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



const char* read_pgm_string(const char* ptr){
  const uint8_t buf_len = 21;
  static char buffer[buf_len];
  size_t len = 0;

  memset(buffer, 0, buf_len);
  while(1){
    if((buffer[len++] = pgm_read_byte(ptr++)) == 0) break;
    else if(len >= buf_len - 1) break;
  }
  return buffer;
}



uint8_t read_pgm_string(const char* ptr, char* buffer, uint8_t buf_len){
  size_t len = 0;

  memset(buffer, 0, buf_len);
  while(1){
    if((buffer[len++] = pgm_read_byte(ptr++)) == 0) break;
    else if(len >= buf_len - 1) break;
  }
  return len;
}
