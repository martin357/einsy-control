// #include <stdint.h>
#include "src/LiquidCrystal_Prusa.h"
#include "pins.h"
#include "hardware.h"
#include "menu_system_derivates.h"
#include "menus.h"

volatile uint32_t counter1 = 0;
volatile uint32_t counter3 = 0;
volatile uint32_t counter4 = 0;
volatile uint32_t counter5 = 0;
// ISR(TIMER1_COMPA_vect){
// // ISR(TIMER3_COMPA_vect){
//   // if(motors[0].running) PORTC ^= 1 << PINC0;
//   // if(motors[1].running) PORTC ^= 1 << PINC1;
//   // if(motors[2].running) PORTC ^= 1 << PINC2;
//   // if(motors[3].running) PORTC ^= 1 << PINC3;
//
//   // for(size_t i = 0; i < MOTORS_MAX; i++){
//   //   if(motors[i].running){
//   //     motors[i].step();
//   //
//   //   }else if(motors[i].steps_to_do){
//   //     motors[i].steps_to_do--;
//   //     motors[i].step();
//   //
//   //   }
//   // }
//   if(motors[0].running){
//     motors[0].step();
//   }else if(motors[0].steps_to_do){
//     motors[0].steps_to_do--;
//     motors[0].step();
//   }
//   counter1++;
// }


ISR(TIMER1_COMPA_vect){
  uint32_t _millis = millis();
  // SERIAL_PRINT(bool(motors[0].target_rpm) ? "1" : "0");
  // SERIAL_PRINT(bool(motors[0].running || motors[0].steps_to_do) ? "1" : "0");
  // SERIAL_PRINT(bool(_millis >= motors[0].last_speed_change + motors[0].ramp_interval) ? "1" : "0");
  // SERIAL_PRINTLN();

  if((motors[0].target_rpm >= 0.0) && (motors[0].running || motors[0].steps_to_do) && (_millis >= motors[0].last_speed_change + motors[0].ramp_interval)){
    SERIAL_PRINT(bool(motors[0].target_rpm >= 0.0) ? '1' : '0');
    SERIAL_PRINT(bool(motors[0].running || motors[0].steps_to_do) ? '1' : '0');
    SERIAL_PRINT(bool(_millis >= motors[0].last_speed_change + motors[0].ramp_interval) ? '1' : '0');
    SERIAL_PRINT(" ");
    SERIAL_PRINT(motors[0].target_rpm);
    SERIAL_PRINT("\t");

    float rpm = motors[0].rpm();
    float rpm_delta = motors[0].target_rpm - rpm;
    // if(abs(rpm_delta) > 1.0){
      // SERIAL_PRINTLN("ramp");
      uint16_t delta_t = _millis - motors[0].last_speed_change;
      float change_fraction;

      if(rpm_delta > 0.0){
        // accelerating
        change_fraction = motors[0].accel / 1000 * delta_t;
        rpm += change_fraction;
        if(rpm >= motors[0].target_rpm){
          rpm = motors[0].target_rpm;
          motors[0].target_rpm = -1.0;  // stop ramping since we reached set
        }

      }else{
        // decelerating
        change_fraction = motors[0].decel / 1000 * delta_t;
        rpm -= change_fraction;
        if(rpm <= motors[0].target_rpm){
          rpm = motors[0].target_rpm;
          motors[0].target_rpm = -1.0;  // stop ramping since we reached set
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

      motors[0].rpm(rpm);

    // }

    motors[0].last_speed_change = _millis;
  }

  if(motors[0].running){
    motors[0].step();

  }else if(motors[0].steps_to_do){
    motors[0].steps_to_do--;
    motors[0].step();

  }
  counter1++;

}

// mock timer cmp vectors
// ISR(TIMER1_COMPA_vect){ PORTC ^= 1 << PINC0; counter1++; }
ISR(TIMER3_COMPA_vect){ counter3++; }
ISR(TIMER4_COMPA_vect){ counter4++; }
ISR(TIMER5_COMPA_vect){ counter5++; }

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  setupPins();
  SPI.begin();

  setupLcd();
  setupMotors();

  current_menu = &main_menu;
  current_menu->draw();


  // OCR1A = 60;
  // OCR1A = 77;

  // OCR1A = F_CPU / 8 / 1000;
  // OCR3A = OCR1A;
  // OCR4A = OCR1A;
  // OCR5A = OCR1A;

  // OCR1A = F_CPU / 8 / 20000;

  menu_motor_stallguard_value.redraw_interval = 50;
  SERIAL_PRINTLN("start");

  // uint16_t sps = rps2sps(1.0, motors[0].usteps);
  // uint16_t ocr = sps2ocr(sps);
  //
  // SERIAL_PRINT("1.0rps is ");
  // SERIAL_PRINT(sps);
  // SERIAL_PRINT("sps which is ");
  // SERIAL_PRINT(ocr);
  // SERIAL_PRINT(" OCR");
  // SERIAL_PRINTLN();
  //
  // uint16_t ocr2 = rps2ocr(1.0, motors[0].usteps);
  // SERIAL_PRINT("rps2ocr(1.0) = ");
  // SERIAL_PRINT(ocr2);
  // SERIAL_PRINT(" OCR");
  // SERIAL_PRINTLN();
  //
  // uint16_t ocr3 = rpm2ocr(60.0, motors[0].usteps);
  // SERIAL_PRINT("rpm2ocr(60.0) = ");
  // SERIAL_PRINT(ocr3);
  // SERIAL_PRINT(" OCR");
  // SERIAL_PRINTLN();
  //
  // float rps = ocr2rps(ocr3, motors[0].usteps);
  // SERIAL_PRINT("ocr2rps(");
  // SERIAL_PRINT(ocr3);
  // SERIAL_PRINT(") = ");
  // SERIAL_PRINT(rps);
  // SERIAL_PRINT("rps");
  // SERIAL_PRINTLN();
  //
  // float rpm = ocr2rpm(ocr3, motors[0].usteps);
  // SERIAL_PRINT("ocr2rpm(");
  // SERIAL_PRINT(ocr3);
  // SERIAL_PRINT(") = ");
  // SERIAL_PRINT(rpm);
  // SERIAL_PRINT("rpm");
  // SERIAL_PRINTLN();
  //
  // SERIAL_PRINT("Max RPM with fullstepping:\t"); SERIAL_PRINTLN(ocr2rpm(70, 1));
  // SERIAL_PRINT("Max RPM with 2 microstepping:\t"); SERIAL_PRINTLN(ocr2rpm(70, 2));
  // SERIAL_PRINT("Max RPM with 4 microstepping:\t"); SERIAL_PRINTLN(ocr2rpm(70, 4));
  // SERIAL_PRINT("Max RPM with 8 microstepping:\t"); SERIAL_PRINTLN(ocr2rpm(70, 8));
  // SERIAL_PRINT("Max RPM with 16 microstepping:\t"); SERIAL_PRINTLN(ocr2rpm(70, 16));
  // SERIAL_PRINT("Max RPM with 32 microstepping:\t"); SERIAL_PRINTLN(ocr2rpm(70, 32));
  // SERIAL_PRINT("Max RPM with 64 microstepping:\t"); SERIAL_PRINTLN(ocr2rpm(70, 64));
  // SERIAL_PRINT("Max RPM with 128 microstepping:\t"); SERIAL_PRINTLN(ocr2rpm(70, 128));
  // SERIAL_PRINT("Max RPM with 256 microstepping:\t"); SERIAL_PRINTLN(ocr2rpm(70, 256));
  //
  // for (uint16_t rpm = 320; rpm < 460; rpm++) {
  //   static uint16_t last_ocr = 0;
  //   uint16_t ocr = rpm2ocr(rpm, 16);
  //   if(ocr != last_ocr){
  //     SERIAL_PRINT("rpm");
  //     SERIAL_PRINT(rpm);
  //     SERIAL_PRINT("\tocr ");
  //     SERIAL_PRINT(ocr);
  //     SERIAL_PRINTLN();
  //     last_ocr = ocr;
  //   }
  // }

  // while(1){}

  // for (size_t i = 0; i < 4; i++) {
  //   motors[i].on();
  // }
  // delay(200);

}


void loop() {
  uint32_t _millis = millis();

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


  if(Serial.available()){
    char ch = Serial.read();
    if(ch == 'r'){
      motors[0].on();

      motors[0].steps_to_do += 200ul * motors[0].usteps;
      uint32_t steps_to_do = motors[0].steps_to_do;

      SERIAL_PRINT("Motor X do ");
      SERIAL_PRINT(steps_to_do);
      SERIAL_PRINTLN(" steps.");

    }else if(ch == 't'){
      float val = Serial.parseFloat();
      if(val > 0.0){
        motors[0].ramp_to(val);
        SERIAL_PRINT("ramp to ");
        SERIAL_PRINTLN(motors[0].target_rpm);
      }

    }else if(ch == 's'){
      motors[0].ramp_to(0.0);
      SERIAL_PRINT("stop to ");
      SERIAL_PRINTLN(motors[0].target_rpm);

    }else if(ch == 'd'){
      motors[0].do_delay = !motors[0].do_delay;
      SERIAL_PRINT("do delay: ");
      SERIAL_PRINTLN(motors[0].do_delay ? "true" : "false");

    }else if(ch == 'b'){
      motors[0].do_toggle = !motors[0].do_toggle;
      SERIAL_PRINT("do toggle: ");
      SERIAL_PRINTLN(motors[0].do_toggle ? "true" : "false");

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

  static uint32_t last_tick = 0;
  uint32_t tick_d = _millis - last_tick;
  // if(tick_d >= 1000){
  if(0){
    // cli();
    SERIAL_PRINT("c1:");
    SERIAL_PRINT(counter1);
    SERIAL_PRINT("\t");
    SERIAL_PRINT((float)counter1 / tick_d);
    SERIAL_PRINT("\t");

    SERIAL_PRINT("c3:");
    SERIAL_PRINT(counter3);
    SERIAL_PRINT("\t");
    SERIAL_PRINT((float)counter3 / tick_d);
    SERIAL_PRINT("\t");

    SERIAL_PRINT("c4:");
    SERIAL_PRINT(counter4);
    SERIAL_PRINT("\t");
    SERIAL_PRINT((float)counter4 / tick_d);
    SERIAL_PRINT("\t");

    SERIAL_PRINT("c5:");
    SERIAL_PRINT(counter5);
    SERIAL_PRINT("\t");
    SERIAL_PRINT((float)counter5 / tick_d);
    SERIAL_PRINT("\t");

    SERIAL_PRINT(motors[0].steps_to_do);
    SERIAL_PRINT("\t");
    SERIAL_PRINT(motors[0].steps_total);

    SERIAL_PRINTLN();
    counter1 = 0;
    counter3 = 0;
    counter4 = 0;
    counter5 = 0;
    // sei();
    last_tick = _millis;
  }

}
