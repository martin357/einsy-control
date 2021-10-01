#include "pins.h"
#include "hardware.h"
#include "serial.h"
#include "menu_system_derivates.h"
#include "menus.h"

uint16_t print_stallguard_to_serial_interval = 50;
uint32_t last_stallguard_print_to_serial = 0;


uint16_t counter = 0;


void setup() {
  Serial.begin(250000);
  Serial.setTimeout(100);
  Serial.println(F("init..."));

  setupPins();
  SPI.begin();

  setupLcd();
  setupMotors();

  current_menu = &main_menu;
  current_menu->draw();

  menu_motor_stallguard_value.redraw_interval = 50;
  Serial.println(F("ready!"));

  /// custom stuff
  motors[0].driver.rms_current(800);
  motors[1].driver.rms_current(800);

  motors[0].driver.sgt(12);
  motors[1].driver.sgt(12);

  // motors[0].accel = 800;
  // motors[1].accel = 800;
  motors[0].accel = 40;
  motors[1].accel = 40;

}


void loop() {
  uint32_t _millis = millis();

  static uint32_t last_report = 0;
  if(_millis > last_report + 1000){
    uint16_t cnt = counter;
    counter = 0;
    if(cnt){
      Serial.print(cnt);
      Serial.print("\t");
      Serial.println(1000.0 / cnt);
    }
    last_report = _millis;
  }

  readEncoder();

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

  if(beeper_off_at && _millis > beeper_off_at){
    beeper_off_at = 0;
    digitalWriteExt(BEEPER, LOW);
  }

  handleSerial();

  if(_millis >= last_stallguard_print_to_serial + print_stallguard_to_serial_interval){
    // bool printed_something = false;
    for(size_t i = 0; i < MOTORS_MAX; i++){
      if(motors[i].print_stallguard_to_serial){
        // printed_something = true;
        MotorStallguardInfo stallguard_info = motors[i].get_stallguard_info();
        Serial.print("M");
        Serial.print(i);
        Serial.print("\t");
        Serial.print(stallguard_info.sg_result);
        Serial.print("\t");
        Serial.print(stallguard_info.fsactive);
        Serial.print("\t");
        Serial.print(stallguard_info.cs_actual);
        Serial.print("\t");
        Serial.print(stallguard_info.rms);
        Serial.print("mA");
        Serial.println();
      }
    }
    // if(printed_something) Serial.println();
    last_stallguard_print_to_serial = _millis;
  }


  // if(Serial.available()){
  if(0){
    char ch = Serial.read();
    if(ch == 'r'){
      motors[0].on();

      motors[0].steps_to_do += 200ul * motors[0].usteps;
      uint32_t steps_to_do = motors[0].steps_to_do;
      motors[0].start();

      // test motor queue
      uint8_t next = motors[0].next_queue_index();
      motors[0].set_queue_item(next, MotorQueueItemType::DO_STEPS, 0, 1);
      motors[0].set_queue_item(next + 1, MotorQueueItemType::DO_STEPS_WITH_RAMP, 12800, 60);
      motors[0].set_queue_item(next + 2, MotorQueueItemType::DO_STEPS, 1200, 30);
      motors[0].set_queue_item(next + 3, MotorQueueItemType::DO_STEPS_WITH_RAMP, 12800, 120);
      motors[0].set_queue_item(next + 4, MotorQueueItemType::DO_STEPS_WITH_RAMP, 12800, 60);
      motors[0].set_queue_item(next + 5, MotorQueueItemType::DO_STEPS_WITH_RAMP, 12800, 120);

      motors[0].debugPrintQueue();

      Serial.print("Motor X do ");
      Serial.print(steps_to_do);
      Serial.println(" steps.");

    }else if(ch == 't'){
      float val = Serial.parseFloat();
      if(val > 0.0){
        for (size_t i = 0; i < MOTORS_MAX; i++) {
          // motors[i].rpm(motors[i].rpm());
          motors[i].ramp_to(val);
        }
        Serial.print("ramp to ");
        Serial.println(motors[0].target_rpm);
      }

    }else if(ch == 's'){
      Serial.println("stop");
      for (size_t i = 0; i < MOTORS_MAX; i++) {
        motors[i].ramp_to(0.0);
      }

    }else if(ch == 'm'){
      Serial.println("start motor0");
      Serial.print("queue index = ");
      Serial.println(motors[0].queue_index);
      motors[0].start();

    }else if(ch == 'd'){
      motors[0].debugPrintQueue();

    }

  }

  static bool motor_moving = false;
  static uint32_t last_motor_change = 0;
  bool new_motor_moving = motors[0].steps_to_do > 0;

  if(new_motor_moving != motor_moving){
    uint32_t delta = _millis - last_motor_change;

    if(!new_motor_moving){
      SERIAL_PRINT("Motor stopped after ");
      SERIAL_PRINT(delta);
      SERIAL_PRINT("ms (");
      SERIAL_PRINT(1000.0 / delta);
      SERIAL_PRINT(" RPS)");
      SERIAL_PRINTLN();

    }

    last_motor_change = _millis;
    motor_moving = new_motor_moving;
  }

  for(size_t i = 0; i < MOTORS_MAX; i++){
    if(motors[i].stallguard_triggered){
      motors[i].stallguard_triggered = false;
      const float rpm = motors[i].rpm();
      Serial.print("Motor ");
      Serial.print("XYZE"[i]);
      Serial.print(" stalled at ");
      Serial.print(rpm);
      Serial.print(" RPM!");
      Serial.println();

    }

  }

}
