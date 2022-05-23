#include "tilt_test.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#include "../../serial.h"
#ifdef CUSTOM_TILT_TEST



// custom stuff
#define STEP_MP 32.0f

void do_move_positive(){
  const uint32_t _millis = millis();
  motors[0].plan_rotations((double)storage.steps / STEP_MP, storage.speed);
  last_menu_redraw = _millis;
  last_lcd_reinit = _millis;
  lcd.clear();
	lcd.print("Moving up...");
  processCommand(F("start x"));
  processCommand(F("wait_for_motor x"));
}
const char pgmstr_do_move_positive[] PROGMEM = "Nahoru";
MenuItemCallable item_do_move_positive(pgmstr_do_move_positive, &do_move_positive, false);


void do_move_negative(){
  const uint32_t _millis = millis();
  motors[0].plan_rotations((double)storage.steps / -STEP_MP, storage.speed);
  last_menu_redraw = _millis;
  last_lcd_reinit = _millis;
  lcd.clear();
	lcd.print("Moving down...");
  processCommand(F("start x"));
  processCommand(F("wait_for_motor x"));
}
const char pgmstr_do_move_negative[] PROGMEM = "Dolu";
MenuItemCallable item_do_move_negative(pgmstr_do_move_negative, &do_move_negative, false);


void do_move_positive_negative(){
  const uint32_t _millis = millis();
  motors[0].plan_rotations((double)storage.steps / STEP_MP, storage.speed);
  lcd.clear();
	lcd.print("Moving up...");
  processCommand(F("start x"));
  processCommand(F("wait_for_motor x"));

  motors[0].plan_rotations((double)storage.steps / -STEP_MP, storage.speed);
  last_menu_redraw = _millis;
  last_lcd_reinit = _millis;
  lcd.clear();
	lcd.print("Moving down...");
  processCommand(F("start x"));
  processCommand(F("wait_for_motor x"));
}
const char pgmstr_do_move_positive_negative[] PROGMEM = "Nahoru a dolu";
MenuItemCallable item_do_move_positive_negative(pgmstr_do_move_positive_negative, &do_move_positive_negative, false);



const char pgmstr_steps[] PROGMEM = "Pocet 'kroku'";
MenuRange<uint8_t> menu_steps("Pocet 'kroku'", storage.steps, 1, 255, 1, true);
MenuItem item_steps(pgmstr_steps, &menu_steps);


const char pgmstr_speed[] PROGMEM = "Rychlost";
MenuRange<uint8_t> menu_speed("Rychlost [RPM]", storage.speed, 1, 255, 1, true);
MenuItem item_speed(pgmstr_speed, &menu_speed);



// main menu
MenuItem* const main_menu_items[] PROGMEM = {
  &item_do_move_positive_negative,
  &item_do_move_positive,
  &item_do_move_negative,
  &item_steps,
  &item_speed,
  &motor_x,
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

  motors[0].invert_direction = true;

}


#endif
