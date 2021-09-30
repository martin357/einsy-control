// #include <stdint.h>
#include "pins.h"
#include "hardware.h"
#include "menu_system_derivates.h"
#include "menus.h"

uint16_t print_stallguard_to_serial_interval = 50;
uint32_t last_stallguard_print_to_serial = 0;


uint16_t counter = 0;


// motors queue handling
ISR(TIMER2_COMPB_vect){
  // counter++;

  for(size_t i = 0; i < MOTORS_MAX; i++){
    if(motors[i].enabled && !motors[i].running && motors[i].steps_to_do == 0){
      // cli();
      if(motors[i].process_next_queue_item()){
        Serial.println("[e]");
        Serial.print("s2d=");
        Serial.println(motors[i].steps_to_do);
        // Serial.flush();

      }
      // sei();
    }
  }
}


// acceleration handling
ISR(TIMER2_COMPA_vect){
  static uint8_t cnt = 0;
  static uint32_t last_tick = 0;
  if(cnt++ >= 5){ // 5=100ms interval
    // counter++;
    uint32_t _millis = millis();
    // Serial.print("acc:");
    // uint32_t delta_t = _millis - last_tick;

    // Serial.print("tD:");
    // Serial.println(delta_t);

    cli();
    for(size_t i = 0; i < MOTORS_MAX; i++){
      // Serial.print(i);
      if((motors[i].target_rpm >= 0.0) && (motors[i].running || motors[i].steps_to_do)){
        SERIAL_PRINT("M");
        SERIAL_PRINT(i);
        SERIAL_PRINT(": ");
        // SERIAL_PRINT(bool(motors[i].target_rpm >= 0.0) ? '1' : '0');
        // SERIAL_PRINT(bool(motors[i].running || motors[i].steps_to_do) ? '1' : '0');
        // SERIAL_PRINT(bool(_millis >= motors[i].last_speed_change + motors[i].ramp_interval) ? '1' : '0');
        // SERIAL_PRINT(" ");
        SERIAL_PRINT(motors[i].target_rpm);
        SERIAL_PRINT("\t");

        float rpm = motors[i].rpm();
        float rpm_delta = motors[i].target_rpm - rpm;
        // if(abs(rpm_delta) > 1.0){
          // SERIAL_PRINTLN("ramp");
          uint16_t delta_t = _millis - motors[i].last_speed_change;
          float change_fraction;

          if(rpm_delta > 0.0){
            // accelerating
            change_fraction = motors[i].accel / 1000.0 * delta_t;
            rpm += change_fraction;
            if(rpm >= motors[i].target_rpm){
              rpm = motors[i].target_rpm;
              motors[i].target_rpm = -1.0;  // stop ramping since we reached set
            }

          }else{
            // decelerating
            change_fraction = motors[i].decel / 1000.0 * delta_t;
            rpm -= change_fraction;
            if(rpm <= motors[i].target_rpm){
              rpm = motors[i].target_rpm;
              motors[i].target_rpm = -1.0;  // stop ramping since we reached set
            }

          }

          SERIAL_PRINT("rpmD ");
          SERIAL_PRINT(rpm_delta);
          SERIAL_PRINT("\ttD ");
          SERIAL_PRINT(delta_t);
          SERIAL_PRINT("\tchF ");
          SERIAL_PRINT(change_fraction);
          SERIAL_PRINT("\trpmN ");
          SERIAL_PRINT(rpm);
          SERIAL_PRINTLN();

          motors[i].rpm(rpm);

          // }

          motors[i].last_speed_change = _millis;
        }

    }
    sei();
    // Serial.print(" ");
    // Serial.print(_millis - last_tick);
    // Serial.println();


    last_tick = _millis;
    cnt = 0;
  }
  // counter2++;
}


// stallguard pin change interrupt
ISR(PCINT2_vect){
  const bool sg[MOTORS_MAX] = {
    PINK & (1 << PINK2), // X_DIAG
    PINK & (1 << PINK7), // Y_DIAG
    PINK & (1 << PINK6), // Z_DIAG
    PINK & (1 << PINK3), // E0_DIAG
  };

  for(size_t i = 0; i < MOTORS_MAX; i++){
    motors[i].stallguard_triggered = sg[i];
    if(motors[i].stop_on_stallguard && sg[i]){
      const float rpm = motors[i].rpm();
      motors[i].stop();
      Serial.print("Motor ");
      Serial.print(i);
      Serial.print(" stalled at ");
      Serial.print(rpm);
      Serial.print(" RPM!");
      Serial.println();
      beep(30);

    }

  }

}


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial.println("init...");

  setupPins();
  SPI.begin();

  setupLcd();
  setupMotors();

  current_menu = &main_menu;
  current_menu->draw();

  menu_motor_stallguard_value.redraw_interval = 50;
  Serial.println("ready!");

  /// custom stuff
  motors[0].driver.rms_current(800);
  motors[1].driver.rms_current(800);

  motors[0].driver.sgt(12);
  motors[1].driver.sgt(12);

  motors[0].accel = 800;
  motors[1].accel = 800;
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


  if(Serial.available()){
    char ch = Serial.read();
    if(ch == 'r'){
      motors[0].on();

      motors[0].steps_to_do += 200ul * motors[0].usteps;
      uint32_t steps_to_do = motors[0].steps_to_do;
      motors[0].start();

      // test motor queue
      uint8_t next = motors[0].next_queue_index();
      motors[0].set_queue_item(next, MotorQueueItemType::DO_STEPS, 6400, 60);
      motors[0].set_queue_item(next + 1, MotorQueueItemType::DO_STEPS, 1200, 30);
      motors[0].set_queue_item(next + 2, MotorQueueItemType::DO_STEPS, 12800, 120);
      motors[0].set_queue_item(next + 3, MotorQueueItemType::DO_STEPS, 12800, 120);
      motors[0].set_queue_item(next + 4, MotorQueueItemType::DO_STEPS, 12800, 120);

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

}
