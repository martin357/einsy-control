#include "resin_mixer.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#include "../../serial.h"
#ifdef CUSTOM_RESIN_MIXER



// custom stuff
void beep_cycle_finished(bool show_on_lcd = true);
void _delay(uint32_t ms);


bool heating_is_on = false;
bool is_heater_on(){ return heating_is_on; }
void do_heater_on(){ heating_is_on = true; }
void do_heater_off(){ digitalWriteExt(PIN_HEATER, LOW); heating_is_on = false; }
const char pgmstr_heater_on[] PROGMEM = "Heater: on";
const char pgmstr_heater_off[] PROGMEM = "Heater: off";
MenuItemToggleCallable item_heater_on_off(&is_heater_on, pgmstr_heater_on, pgmstr_heater_off, &do_heater_off, &do_heater_on);


uint16_t mixing_remaining = 0;
uint32_t last_deduction = 0;
bool cycle_running = false;
bool is_mixing_on(){ return cycle_running; }
void do_mixing_on(){
  cycle_running = true;
  heating_is_on = true;
  mixing_remaining = storage.mixing_duration * 60;
  motors[0].rpm(1);
  motors[0].start(true);
  motors[0].ramp_to(storage.mixing_speed);
  last_deduction = millis();
}
void do_mixing_off(){
  digitalWriteExt(PIN_HEATER, LOW);
  lcd.clear();
  lcd.print("Stopping...");
  heating_is_on = false;
  cycle_running = false;
  mixing_remaining = 0;
  motors[0].ramp_to(0.0);
  processCommand(F("wait_for_motor x"));
  // beep(1);
  // beep_cycle_finished();
}
const char pgmstr_mixing_on[] PROGMEM = "Michani: on";
const char pgmstr_mixing_off[] PROGMEM = "Michani: off";
MenuItemToggleCallable item_mixing_on_off(&is_mixing_on, pgmstr_mixing_on, pgmstr_mixing_off, &do_mixing_off, &do_mixing_on);


const char pgmstr_temperature[] PROGMEM = "Cilova teplota";
MenuRange<uint8_t> menu_target_temperature("Teplota: [C]", storage.target_temperature, 20, 45, 1, true);
MenuItem item_target_temperature(pgmstr_temperature, &menu_target_temperature);


const char pgmstr_mixing_duration[] PROGMEM = "Delka cyklu";
MenuRange<uint8_t> menu_mixing_duration("Celka cyklu: [min]", storage.mixing_duration, 1, 240, 1, true);
MenuItem item_mixing_duration(pgmstr_mixing_duration, &menu_mixing_duration);


uint32_t last_temp_change = 0;



void loopCustom(){
  const uint32_t _millis = millis();
  if(heating_is_on && _millis > last_temp_change + 33){
    const bool heater_pin = digitalReadExt(PIN_HEATER);
    if(heater_pin){
      if(temperature[HEATER_THERM] + 2.0 >= storage.target_temperature) digitalWriteExt(PIN_HEATER, LOW);

    }else{
      if(temperature[HEATER_THERM] + 1.0 <= storage.target_temperature) digitalWriteExt(PIN_HEATER, HIGH);

    }
    last_temp_change = _millis;
  }

  if(mixing_remaining > 0 && _millis >= last_deduction + 1000){
    last_deduction = _millis;
    if(--mixing_remaining == 0){
      do_mixing_off();
      beep_cycle_finished();
    }
  }

}

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

  // beep(40);
  // _delay(400);
  // beep(40);
  // _delay(400);
  // beep(40);
}

MenuItemDynamic<uint16_t> item_mixing_remaining("Zbyvajici cas", mixing_remaining);


MenuItemDynamic<float> item_temp0("Temp0", temperature[0]);
MenuItemDynamic<float> item_temp1("Temp1", temperature[1]);
MenuItemDynamic<float> item_temp2("T. heater", temperature[2]);



class MenuMotorSpeed: public Menu{
public:
  MenuMotorSpeed(): Menu(nullptr, 0){ redraw_interval = 50; };
  void on_enter(){ lcd.clear(); }
  void on_press(uint16_t){ go_back(); };
  void draw(bool = true){
    lcd.print("\3", 0, 0); lcd.print("Speed"); lcd.print(" \1");
    lcd.print("<", 0, 2); lcd.setCursor(8, 2); lcd.print((uint16_t)storage.mixing_speed); lcd.print(">", 19, 2);
  };
  void move(int8_t amount){
    storage.mixing_speed += amount;
    if(storage.mixing_speed < 60) storage.mixing_speed = 60;
    if(storage.mixing_speed > 460) storage.mixing_speed = 460;
    storage.save();
    motors[0].rpm(storage.mixing_speed);
  };
};


MenuMotorSpeed menu_mixing_speed;
const char pgmstr_mixing_speed[] PROGMEM = "Rychlost";
MenuItem item_mixing_speed(pgmstr_mixing_speed, &menu_mixing_speed);



void do_preheat(bool do_beep = true){
  lcd.clear();
  lcd.print("Preheating...");

  do_heater_on();

  const uint32_t preheat_start = millis();
  const float start_temp = temperature[2];

  while(1){
    const float temp_delta = storage.target_temperature - temperature[2];
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

  do_heater_off();
  if(do_beep) beep();

}
const char pgmstr_preheat[] PROGMEM = "Predehrev";
void _do_preheat(){ do_preheat(); }
MenuItemCallable item_preheat(pgmstr_preheat, &_do_preheat, false);


// debug menu
MenuItem* const debug_menu_items[] PROGMEM = {
  &back,
  // &item_washing_pump_on_off,
  &item_heater_on_off,
  &item_temp2,
  &item_temp1,
  &item_temp0,
  &motor_x,
};
Menu debug_menu(debug_menu_items, sizeof(debug_menu_items) / 2);
const char pgmstr_debug[] PROGMEM = "!!! Debug";
MenuItem item_debug_menu(pgmstr_debug, &debug_menu);


// main menu
MenuItem* const main_menu_items[] PROGMEM = {
  &item_preheat,
  &item_mixing_on_off,
  &item_mixing_remaining,
  &item_mixing_duration,
  &item_target_temperature,
  &item_mixing_speed,
  &item_temp2,
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

  pinModeInput(A0);
  pinModeInput(A1);
  pinModeInput(A2);

  main_menu.redraw_interval = 50;

}


#endif
