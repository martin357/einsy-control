#include "cw_letnany.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#ifdef CUSTOM_CW_LETNANY



// custom CW_LETNANY stuff
uint8_t water_pump_mode = 0;
void do_water_pump_off(){ water_pump_mode = 0; }
void do_water_pump_half(){ water_pump_mode = 1; }
void do_water_pump_full(){ water_pump_mode = 2; }

bool is_washing_pump_on(){ return digitalReadExt(PIN_WASHING_PUMP); }
void do_washing_pump_on(){ digitalWriteExt(PIN_WASHING_PUMP, HIGH); }
void do_washing_pump_off(){ digitalWriteExt(PIN_WASHING_PUMP, LOW); }
const char pgmstr_washing_pump_on[] PROGMEM = "Washing: on";
const char pgmstr_washing_pump_off[] PROGMEM = "Washing: off";
MenuItemToggleCallable item_washing_pump_on_off(&is_washing_pump_on, pgmstr_washing_pump_on, pgmstr_washing_pump_off, &do_washing_pump_off, &do_washing_pump_on);


bool heating_is_on = false;
bool is_heater_on(){ return heating_is_on; }
void do_heater_on(){ heating_is_on = true; }
void do_heater_off(){ digitalWriteExt(PIN_HEATER, LOW); heating_is_on = false; }
const char pgmstr_heater_on[] PROGMEM = "Heater: on";
const char pgmstr_heater_off[] PROGMEM = "Heater: off";
MenuItemToggleCallable item_heater_on_off(&is_heater_on, pgmstr_heater_on, pgmstr_heater_off, &do_heater_off, &do_heater_on);


const char pgmstr_temperature[] PROGMEM = "Cilova teplota";
MenuRange<uint8_t> menu_target_temperature("Teplota: [C]", storage.target_temperature, 20, 65, true);
MenuItem item_target_temperature(pgmstr_temperature, &menu_target_temperature);


const char pgmstr_washing_duration[] PROGMEM = "Delka myciho cyklu";
MenuRange<uint8_t> menu_washing_duration("Doba myti: [min]", storage.washing_duration, 1, 240, true);
MenuItem item_washing_duration(pgmstr_washing_duration, &menu_washing_duration);


const char pgmstr_water_pump_intensity[] PROGMEM = "Vykon cerpadla";
MenuRange<uint8_t> menu_water_pump_intensity("Vykon cerpadla", storage.water_pump_intensity, 100, 255, true);
MenuItem item_water_pump_intensity(pgmstr_water_pump_intensity, &menu_water_pump_intensity);



uint32_t last_temp_change = 0;


void loopCustom(){
  const uint32_t _millis = millis();
  if(heating_is_on && _millis > last_temp_change + 333){
    const bool heater_pin = digitalReadExt(PIN_HEATER);
    if(heater_pin){
      if(temperature[HEATER_THERM] + 2.0 >= storage.target_temperature) digitalWriteExt(PIN_HEATER, LOW);

    }else{
      if(temperature[HEATER_THERM] + 1.0 <= storage.target_temperature) digitalWriteExt(PIN_HEATER, HIGH);

    }
    last_temp_change = _millis;
  }

  switch (water_pump_mode) {
    case 0: if(OCR3C != 0) OCR3C = 0; break;
    case 1: if(OCR3C != 3) OCR3C = 3; break;
    case 2: if(OCR3C != storage.water_pump_intensity) OCR3C = storage.water_pump_intensity; break;
  }
}


MenuItemDynamic<double> item_temp0("Temp0", temperature[0]);
MenuItemDynamic<double> item_temp1("T. zasobnik", temperature[1]);
MenuItemDynamic<double> item_temp2("T. trubka", temperature[2]);


// OCR3C
const char pgmstr_timer3c[] PROGMEM = "OCR3C";
uint16_t* timer_ptr3c = &OCR3C;
MenuRange<uint16_t> menu_timer3c("OCR3C", *timer_ptr3c, 0, 255);
MenuItem timer3c(pgmstr_timer3c, &menu_timer3c);



void _delay(uint32_t ms){
  const uint32_t finish = millis() + ms;
  while(millis() < finish){
    loopCustom();
    delay(1);
  }
}

void beep_cycle_finished(bool show_on_lcd = true){
  if(show_on_lcd){
    lcd.clear();
    lcd.print("Finished!");
  }

  beep(40);
  _delay(400);
  beep(40);
  _delay(400);
  beep(40);
}


void do_preheat(bool do_beep = true){
  lcd.clear();
  lcd.print("Preheating...");

  do_water_pump_half();
  do_heater_on();

  const uint32_t preheat_start = millis();
  const double start_temp = temperature[2];

  while(1){
    const double temp_delta = storage.target_temperature - temperature[2];
    lcd.setCursor(0, 2);
    lcd.print((float)temperature[2]);
    lcd.print(" /");
    lcd.print(storage.target_temperature, 10);
    lcd.print(".00");
    loopCustom();
    delay(1);

    if(enc_diff != 0){
      storage.target_temperature += enc_diff;
      if(storage.target_temperature < 20) storage.target_temperature = 20;
      if(storage.target_temperature > 45) storage.target_temperature = 45;
      // storage.save();
      enc_diff = 0;
    }

    if(temp_delta < 1.0) break;
  }

  const uint32_t duration_ms = millis() - preheat_start;
  Serial.print(F("Preheat "));
  Serial.print(start_temp);
  Serial.print(F(" -> "));
  Serial.print(storage.target_temperature);
  Serial.print(F(" done in "));
  Serial.print(duration_ms / 1000);
  Serial.println(F("sec."));

  do_water_pump_off();
  do_heater_off();
  if(do_beep) beep();

}
const char pgmstr_preheat[] PROGMEM = "Predehrev";
void _do_preheat(){ do_preheat(); }
MenuItemCallable item_preheat(pgmstr_preheat, &_do_preheat, false);


void do_wash_cycle(bool do_beep = true){
  lcd.clear();
  lcd.print("Washing...");

  do_water_pump_full();
  do_heater_on();

  int32_t remaining;
  uint32_t washing_end = millis() + (60000UL * storage.washing_duration);

  while((remaining = (washing_end - millis()) / 1000L) > 0){
    loopCustom();
    lcd.setCursor(0, 2);
    lcd.print("Remaining ");
    lcd.print((uint16_t)remaining);
    lcd.print("s ");

    if(enc_click > 1){
      enc_click = 0;
      break;
    }
    if(enc_diff != 0){
      washing_end += enc_diff * 5000L;
      enc_diff = 0;
    }
    if(washing_end < millis()) break;
    delay(10);
  }

  do_water_pump_off();
  do_heater_off();
  // beep();

}
const char pgmstr_wash_cycle[] PROGMEM = "Myci cyklus";
void _do_wash_cycle(){ do_wash_cycle(); }
MenuItemCallable item_wash_cycle(pgmstr_wash_cycle, &_do_wash_cycle, false);


void do_preheat_and_wash(){
  do_preheat(false);
  do_wash_cycle(false);
  beep_cycle_finished();

}
const char pgmstr_preheat_and_wash[] PROGMEM = "Predehrev + myti";
MenuItemCallable item_preheat_and_wash(pgmstr_preheat_and_wash, &do_preheat_and_wash, false);



const char pgmstr_water_pump_off[] PROGMEM = "Water Pump: off";
MenuItemCallable item_water_pump_off(pgmstr_water_pump_off, &do_water_pump_off, false);

const char pgmstr_water_pump_half[] PROGMEM = "Water Pump: half";
MenuItemCallable item_water_pump_half(pgmstr_water_pump_half, &do_water_pump_half, false);

const char pgmstr_water_pump_full[] PROGMEM = "Water Pump: full";
MenuItemCallable item_water_pump_full(pgmstr_water_pump_full, &do_water_pump_full, false);

// debug menu
MenuItem* const debug_menu_items[] PROGMEM = {
  &back,
  &timer3c,
  // &item_washing_pump_on_off,
  &item_heater_on_off,
  &item_temp2,
  &item_water_pump_off,
  &item_water_pump_half,
  &item_water_pump_full,
  &item_water_pump_intensity,
  &motor_x,
  &motor_z,
};
Menu debug_menu(debug_menu_items, sizeof(debug_menu_items) / 2);
const char pgmstr_debug[] PROGMEM = "!!! Debug";
MenuItem item_debug_menu(pgmstr_debug, &debug_menu);


// main menu
MenuItem* const main_menu_items[] PROGMEM = {
  &item_preheat_and_wash,
  &item_wash_cycle,
  &item_preheat,
  &item_washing_duration,
  &item_target_temperature,
  &item_temp2,
  // &timer3c,
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

  // setup water pump pwm timer (overriding motor Y timer!!)
  TCCR3A = (1 << COM3C1) | (1 << WGM30);
  TCCR3B = (1 << CS31) | (1 << CS30);
  OCR3C = 0;

}


#endif
