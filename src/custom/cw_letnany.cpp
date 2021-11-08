#include "cw_letnany.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#ifdef CUSTOM_CW_LETNANY



// custom CW_LETNANY stuff
bool is_washing_pump_on(){ return digitalReadExt(PIN_WASHING_PUMP); }
void do_washing_pump_on(){ digitalWriteExt(PIN_WASHING_PUMP, HIGH); }
void do_washing_pump_off(){ digitalWriteExt(PIN_WASHING_PUMP, LOW); }
const char pgmstr_washing_pump_on[] PROGMEM = "Washing: on";
const char pgmstr_washing_pump_off[] PROGMEM = "Washing: off";
MenuItemToggleCallable item_washing_pump_on_off(&is_washing_pump_on, pgmstr_washing_pump_on, pgmstr_washing_pump_off, &do_washing_pump_off, &do_washing_pump_on);


bool heating_is_on = false;
bool is_heater_on(){ return /*digitalReadExt(PIN_HEATER);*/ heating_is_on; }
void do_heater_on(){ /*digitalWriteExt(PIN_HEATER, HIGH);*/ heating_is_on = true; }
void do_heater_off(){ digitalWriteExt(PIN_HEATER, LOW); heating_is_on = false; }
const char pgmstr_heater_on[] PROGMEM = "Heater: on";
const char pgmstr_heater_off[] PROGMEM = "Heater: off";
MenuItemToggleCallable item_heater_on_off(&is_heater_on, pgmstr_heater_on, pgmstr_heater_off, &do_heater_off, &do_heater_on);


const char pgmstr_temperature[] PROGMEM = "Temperature";
MenuRange<uint8_t> menu_target_temperature("Temperature:", storage.target_temperature, 20, 45, true);
MenuItem item_target_temperature(pgmstr_temperature, &menu_target_temperature);



static float analog2tempBed(int raw) {
  float celsius = 0;
  byte i;

  for (i=1; i<TEMPERATURE_TABLE_EINSY_LEN; i++)
  {
    if (PGM_RD_W(temperature_table_einsy[i][0]) > raw)
    {
      celsius  = PGM_RD_W(temperature_table_einsy[i-1][1]) +
        (raw - PGM_RD_W(temperature_table_einsy[i-1][0])) *
        (float)(PGM_RD_W(temperature_table_einsy[i][1]) - PGM_RD_W(temperature_table_einsy[i-1][1])) /
        (float)(PGM_RD_W(temperature_table_einsy[i][0]) - PGM_RD_W(temperature_table_einsy[i-1][0]));
      break;
    }
  }

  // Overflow: Set to last value in the table
  if (i == TEMPERATURE_TABLE_EINSY_LEN) celsius = PGM_RD_W(temperature_table_einsy[i-1][1]);

	// temperature offset adjustment
#ifdef BED_OFFSET
	float _offset = BED_OFFSET;
	float _offset_center = BED_OFFSET_CENTER;
	float _offset_start = BED_OFFSET_START;
	float _first_koef = (_offset / 2) / (_offset_center - _offset_start);
	float _second_koef = (_offset / 2) / (100 - _offset_center);

	if (celsius >= _offset_start && celsius <= _offset_center){
		celsius = celsius + (_first_koef * (celsius - _offset_start));
	}
	else if (celsius > _offset_center && celsius <= 100){
		celsius = celsius + (_first_koef * (_offset_center - _offset_start)) + ( _second_koef * ( celsius - ( 100 - _offset_center ) )) ;
	}
	else if (celsius > 100){
		celsius = celsius + _offset;
	}
#endif

  return celsius;
}


#define THERMISTOR_CNT 3

const uint8_t samples_total = 64;
uint8_t samples_collected = 0;
double temperature_raw[THERMISTOR_CNT] = {0};
double temperature[THERMISTOR_CNT] = {0};
uint32_t last_temp_change = 0;


void loopCustom(){
  const uint32_t _millis = millis();
  for (size_t i = 0; i < THERMISTOR_CNT; i++) {
    const uint16_t uval = analogRead(A0 + i);

    if(samples_collected < samples_total){
      temperature_raw[i] = ((temperature_raw[i] * samples_collected) + (double)uval) / (samples_collected + 1);
    }else{
      temperature_raw[i] = ((temperature_raw[i] * (samples_collected - 1)) + (double)uval) / samples_collected;
    }
    temperature[i] = analog2tempBed(temperature_raw[i]);
  }
  if(samples_collected < samples_total) samples_collected++;

  if(heating_is_on && _millis > last_temp_change + 333){
    const bool heater_pin = digitalReadExt(PIN_HEATER);
    if(heater_pin){
      if(temperature[HEATER_THERM] + 2.0 >= storage.target_temperature) digitalWriteExt(PIN_HEATER, LOW);

    }else{
      if(temperature[HEATER_THERM] + 1.0 <= storage.target_temperature) digitalWriteExt(PIN_HEATER, HIGH);

    }
    last_temp_change = _millis;
  }

}


float get_temp0(){ return temperature[0]; }
MenuItemDynamicCallable<float> item_temp0("Temp0", &get_temp0);

float get_temp1(){ return temperature[1]; }
MenuItemDynamicCallable<float> item_temp1("Temp1", &get_temp1);

float get_temp2(){ return temperature[2]; }
MenuItemDynamicCallable<float> item_temp2("Temp2", &get_temp2);


// debug menu
MenuItem* const debug_menu_items[] PROGMEM = {
  &back,
  &motor_x,
  &motor_z,
};
Menu debug_menu(debug_menu_items, sizeof(debug_menu_items) / 2);
const char pgmstr_debug[] PROGMEM = "!!! Debug";
MenuItem item_debug_menu(pgmstr_debug, &debug_menu);


// main menu
MenuItem* const main_menu_items[] PROGMEM = {
  &item_washing_pump_on_off,
  &item_heater_on_off,
  &item_temp2,
  &item_target_temperature,
  &item_temp1,
  &item_temp0,
  &item_debug_menu,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);




void setupCustom(){
  for (size_t i = 0; i < MOTORS_MAX; i++) {
    motors[i].driver.rms_current(500);
    motors[i].driver.sgt(2);

    motors[i].driver.en_pwm_mode(1);
    motors[i].driver.pwm_autoscale(1);
    motors[i].driver.intpol(1);

    motors[i].rpm(120);
  }

  // power output
  pinModeOutput(PIN_HEATER);
  digitalWriteExt(PIN_HEATER, LOW);
  pinModeOutput(PIN_WASHING_PUMP);
  digitalWriteExt(PIN_WASHING_PUMP, LOW);

  pinModeInput(A0);
  pinModeInput(A1);
  pinModeInput(A2);

  main_menu.redraw_interval = 50;

}


#endif
