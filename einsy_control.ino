#include "pins.h"
#include "hardware.h"
#include "serial.h"
#include "permanent_storage.h"
#include "menu_system_derivates.h"
#include "menus.h"
#include "custom.h"


uint16_t print_stallguard_to_serial_interval = 30;
uint32_t last_stallguard_print_to_serial = 0;


void printBinary(byte inByte) { for (int b = 7; b >= 0; b--) Serial.print(bitRead(inByte, b)); Serial.println(); }
#define _PRINT_BIN(v) #v
#define PRINT_BIN(v) Serial.print(F(#v "\t= ")); printBinary(v);

void setup() {
  Serial.begin(74880);
  Serial.setTimeout(100);
  Serial.println(F("init..."));
  #ifdef CUSTOM
    Serial.println(F("custom_variant: " CUSTOM));
  #endif

  setupPins();
  SPI.begin();

  storage.load();
  setupLcd();
  setupMotors();

  current_menu = &main_menu;
  current_menu->draw();
  menu_motor_stallguard_value.redraw_interval = 50;

  #ifdef CUSTOM_SETUP
    setupCustom();
  #endif

  Serial.println(F("ready!"));

}


void loop() {
  uint32_t _millis = millis();

  current_menu->loop();

  if(enc_diff){
    current_menu->move(enc_diff);
    enc_diff = 0;

  }

  if(enc_click){
    current_menu->on_press(enc_click);
    enc_click = 0;

  }

  if(current_menu->redraw_interval > 0 && _millis > last_menu_redraw + current_menu->redraw_interval) current_menu->draw(false);

  handleSerial();

  if(_millis >= last_stallguard_print_to_serial + print_stallguard_to_serial_interval){
    bool printed_something = false;
    for(size_t i = 0; i < MOTORS_MAX; i++){
      if(motors[i].print_stallguard_to_serial){
        printed_something = true;
        MotorStallguardInfo stallguard_info = motors[i].get_stallguard_info();
        // Serial.print("M");
        // Serial.print(i);
        // Serial.print("\t");
        Serial.print(stallguard_info.sg_result);
        Serial.print("\t");
        // Serial.print(stallguard_info.fsactive);
        // Serial.print("\t");
        // Serial.print(stallguard_info.cs_actual);
        // Serial.print("\t");
        Serial.print(stallguard_info.rms);
        Serial.print("mA\t");
        // Serial.println();
      }
    }
    if(printed_something) Serial.println();
    last_stallguard_print_to_serial = _millis;
  }


  #ifndef DISABLE_STALLGUARD_TRIGGERED_PRINT_TO_SERIAL
  for(size_t i = 0; i < MOTORS_MAX; i++){
    if(motors[i].stallguard_triggered){
      motors[i].stallguard_triggered = false;
      const float rpm = motors[i].rpm();
      const uint32_t lost_steps = motors[i].driver.LOST_STEPS();
      Serial.print(F("Motor "));
      Serial.print("XYZE"[i]);
      Serial.print(F(" stalled at "));
      Serial.print(rpm);
      Serial.print(F(" RPM with "));
      Serial.print(lost_steps);
      Serial.println(F(" lost steps!"));
    }
  }
  #endif // DISABLE_STALLGUARD_TRIGGERED_PRINT_TO_SERIAL

  if(_millis > last_lcd_reinit + 5000){
    lcd.reinit();
    current_menu->draw();
    last_lcd_reinit = _millis;
  }

  if(read_temperature) readThermistors();
  if(read_voltage) readVoltages();

  #ifdef CUSTOM_LOOP
    loopCustom();
  #endif

}
