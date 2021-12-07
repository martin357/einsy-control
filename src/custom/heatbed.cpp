#include "heatbed.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#ifdef CUSTOM_HEATBED



// custom HEATBED stuff
bool heating_is_on = false;
bool is_heater_on(){ return heating_is_on; }
void do_heater_on(){ heating_is_on = true; }
void do_heater_off(){ digitalWriteExt(PIN_HEATER, LOW); heating_is_on = false; }
const char pgmstr_heater_on[] PROGMEM = "Heater: on";
const char pgmstr_heater_off[] PROGMEM = "Heater: off";
MenuItemToggleCallable item_heater_on_off(&is_heater_on, pgmstr_heater_on, pgmstr_heater_off, &do_heater_off, &do_heater_on);


const char pgmstr_temperature[] PROGMEM = "Cilova teplota";
MenuRange<uint8_t> menu_target_temperature("Teplota: [C]", storage.target_temperature, 20, 100, true);
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


MenuItemDynamic<double> item_temp0("Temp0", temperature[0]);
MenuItemDynamic<double> item_temp1("Temp1", temperature[1]);
MenuItemDynamic<double> item_temp2("Temp2", temperature[2]);



void _delay(uint32_t ms){
  const uint32_t finish = millis() + ms;
  while(millis() < finish){
    loopCustom();
    delay(1);
  }
}



// main menu
MenuItem* const main_menu_items[] PROGMEM = {
  &item_heater_on_off,
  &item_target_temperature,
  &item_temp0,
  &item_temp1,
  &item_temp2,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);




void setupCustom(){
  // power output
  pinModeOutput(PIN_HEATER);
  digitalWriteExt(PIN_HEATER, LOW);

  pinModeInput(A0);
  pinModeInput(A1);
  pinModeInput(A2);

  main_menu.redraw_interval = 50;
}


#endif
