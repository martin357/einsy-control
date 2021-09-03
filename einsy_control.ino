// #include <stdint.h>
#include "src/LiquidCrystal_Prusa.h"
#include "pins.h"
#include "hardware.h"
#include "menu_system.h"
#include "menus.h"


ISR(TIMER1_COMPA_vect){
  if(motors[0].running) PORTC ^= 1 << PINC0;
  if(motors[1].running) PORTC ^= 1 << PINC1;
  if(motors[2].running) PORTC ^= 1 << PINC2;
  if(motors[3].running) PORTC ^= 1 << PINC3;

}

void setup() {
  Serial.begin(115200);

  setupPins();
  SPI.begin();

  setupLcd();
  setupMotors();

  current_menu = &main_menu;
  current_menu->draw();


  OCR1A = 100;

  menu_motor_stallguard_value.redraw_interval = 50;

}


void loop() {
  uint32_t _millis = millis();

  readEncoder();

  if(enc_diff){
    current_menu->move(enc_diff);
    enc_diff = 0;

    current_menu->draw();
  }

  if(enc_click){
    current_menu->on_press(enc_click);
    enc_click = 0;

  }

  if(current_menu->redraw_interval > 0 && _millis > last_menu_redraw + current_menu->redraw_interval) current_menu->draw(false);

  if(beeper_off_at && _millis > beeper_off_at){
    beeper_off_at = 0;
    digitalWriteExt(BEEPER, LOW);
  }

}
