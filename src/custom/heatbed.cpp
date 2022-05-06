#include "heatbed.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#ifdef CUSTOM_HEATBED



// custom HEATBED stuff
uint32_t last_on_time_sec_increment_at = 0;

bool heating_is_on = false;
bool is_heater_on(){ return heating_is_on; }
void do_heater_on(){ last_on_time_sec_increment_at = millis(); heating_is_on = true; }
void do_heater_off(){ digitalWriteExt(PIN_HEATER, LOW); heating_is_on = false; }
const char pgmstr_heater_on[] PROGMEM = "Heater: on";
const char pgmstr_heater_off[] PROGMEM = "Heater: off";
MenuItemToggleCallable item_heater_on_off(&is_heater_on, pgmstr_heater_on, pgmstr_heater_off, &do_heater_off, &do_heater_on);


const char pgmstr_temperature[] PROGMEM = "Cilova teplota";
MenuRange<uint8_t> menu_target_temperature("Teplota: [C]", storage.target_temperature, 20, 100, true);
MenuItem item_target_temperature(pgmstr_temperature, &menu_target_temperature);



uint32_t last_temp_change = 0;
uint32_t last_no_therm_beep = 0;
uint16_t on_time_sec = 0;


void loopCustom(){
  const uint32_t _millis = millis();
  uint8_t active_therm = 2;
  uint8_t target_temp = storage.target_temperature;
  if(temperature[0] != 0.0){
    active_therm = 0;
    if(!lcd_present) target_temp = 40;
  }
  else if(temperature[1] != 0.0){
    active_therm = 1;
    if(!lcd_present) target_temp = 60;
  }
  else if(temperature[2] != 0.0){
    active_therm = 2;
    if(!lcd_present) target_temp = 80;
  }
  else{
    target_temp = 0;
    if(heating_is_on && _millis >= last_no_therm_beep + 5000){
      last_no_therm_beep = _millis;
      beep(1);
    }
  }

  if((!lcd_present || heating_is_on) && _millis > last_temp_change + 600){
    const bool heater_pin = digitalReadExt(PIN_HEATER);

    if(target_temp > 0){
      if(heater_pin){
        if(temperature[active_therm] + 2.0 >= target_temp) digitalWriteExt(PIN_HEATER, LOW);

      }else{
        if(temperature[active_therm] + 1.0 <= target_temp) digitalWriteExt(PIN_HEATER, HIGH);

      }
    }else{
      digitalWriteExt(PIN_HEATER, LOW);

    }
    last_temp_change = _millis;
  }

  if(heating_is_on && _millis >= last_on_time_sec_increment_at + 1000){
    last_on_time_sec_increment_at = _millis;
    on_time_sec++;
  }
}


MenuItemDynamic<double> item_temp0("Temp0", temperature[0]);
MenuItemDynamic<double> item_temp1("Temp1", temperature[1]);
MenuItemDynamic<double> item_temp2("Temp2", temperature[2]);
MenuItemDynamic<double> item_temp_pinda("Pinda temp", temperature[3]);
MenuItemDynamic<double> item_temp_ambient("Ambient t.", temperature[4]);

MenuItemDynamic<double> item_volt_pwr("Pwr voltage", voltage[0]);
MenuItemDynamic<double> item_volt_bed("Bed voltage", voltage[1]);
MenuItemDynamic<double> item_volt_ir("IR voltage", voltage[2]);

MenuItemDynamic<uint16_t> item_on_time_sec("On duration:", on_time_sec);
void do_reset_on_time_sec(){ on_time_sec = 0; }
const char pgmstr_reset_on_time_sec[] PROGMEM = "Reset on duration";
MenuItemCallable item_reset_on_time_sec(pgmstr_reset_on_time_sec, &do_reset_on_time_sec, false);


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
  &item_temp_pinda,
  &item_temp_ambient,
  &item_volt_pwr,
  &item_volt_bed,
  &item_volt_ir,
  &item_on_time_sec,
  &item_reset_on_time_sec,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);



void setupCustom(){
  // power output
  pinModeOutput(PIN_HEATER);
  digitalWriteExt(PIN_HEATER, LOW);

  main_menu.redraw_interval = 50;
}


#endif
