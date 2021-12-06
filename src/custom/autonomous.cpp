#include "autonomous.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#include "../../serial.h"
#ifdef CUSTOM_AUTONOMOUS


/// custom stuff
uint8_t half_rot_no = 20;
uint8_t rpm_start = 60;
uint8_t rpm_target = 160;
bool half_rot_dir = true;
// bool engage_pinda = false;

MenuItemToggle item_half_rot_dir(&half_rot_dir, "Dir: up", "Dir: down");

MenuRange<uint8_t> menu_half_rot_no("Half rot. no.:", half_rot_no, 1, 255);
const char pgmstr_half_rot_no[] PROGMEM = "Half rot. no.";
MenuItem item_half_rot_no(pgmstr_half_rot_no, &menu_half_rot_no);

MenuRange<uint8_t> menu_rpm_start("RPM start:", rpm_start, 1, 255);
const char pgmstr_rpm_start[] PROGMEM = "RPM start";
MenuItem item_rpm_start(pgmstr_rpm_start, &menu_rpm_start);

MenuRange<uint8_t> menu_rpm_target("RPM target:", rpm_target, 1, 255);
const char pgmstr_rpm_target[] PROGMEM = "RPM target";
MenuItem item_rpm_target(pgmstr_rpm_target, &menu_rpm_target);


void setupCustom(){
  for (size_t i = 0; i < MOTORS_MAX; i++) {
    motors[i].driver.semax(0);
    motors[i].driver.semin(0);
    motors[i].driver.rms_current(1000);
    motors[i].driver.sgt(12);

    motors[i].driver.en_pwm_mode(false);
    motors[i].driver.pwm_autoscale(false);
    motors[i].driver.intpol(false);

    motors[i].driver.TCOOLTHRS(0);

    motors[i].microsteps(8);

    // motors[i].driver.rms_current(400);
    // motors[i].driver.sgt(6);
    // motors[i].rpm(120);
  }

  motors[0].driver.semax(5);
  motors[0].driver.semin(2);
  motors[0].driver.en_pwm_mode(true);
  motors[0].driver.pwm_autoscale(true);
  motors[0].driver.intpol(true);
  motors[0].driver.rms_current(400);
  motors[0].driver.sgt(4);
  motors[0].driver.TCOOLTHRS(460);

  motors[2].driver.semax(5);
  motors[2].driver.semin(2);
  motors[2].driver.en_pwm_mode(true);
  motors[2].driver.pwm_autoscale(true);
  motors[2].driver.intpol(true);
  motors[2].driver.rms_current(400);
  motors[2].driver.sgt(4);
  motors[2].driver.TCOOLTHRS(460);

  motors[3].driver.semax(5);
  motors[3].driver.semin(2);
  motors[3].driver.en_pwm_mode(true);
  motors[3].driver.pwm_autoscale(true);
  motors[3].driver.intpol(true);
  motors[3].driver.rms_current(400);
  motors[3].driver.sgt(4);
  motors[3].driver.TCOOLTHRS(460);
  // motors[3].print_stallguard_to_serial = true;
  // motors[0].print_stallguard_to_serial = true;

  // power output
  pinModeOutput(PIN_WASHING);
  digitalWriteExt(PIN_WASHING, LOW);

  pinModeOutput(PIN_HEATING);
  digitalWriteExt(PIN_HEATING, LOW);

  pinModeOutput(PIN_VALVE_0);
  digitalWriteExt(PIN_VALVE_0, LOW);

  pinModeOutput(PIN_VALVE_1);
  digitalWriteExt(PIN_VALVE_1, LOW);


  // setup diag pin interrupt
  PCICR |= (1 << PCIE0);
  PCMSK0 = 0;
  PCMSK0 |= (1 << PCINT4); // Z_MIN
  // PCMSK0 |= (1 << PCINT5); // Y_MIN
  PCMSK0 |= (1 << PCINT6); // X_MIN


  // normal mode
  // for (size_t i = 0; i < MOTORS_MAX; i++) {
  motors[0].inactivity_timeout = 0;

  motors[2].driver.semax(0);
  motors[2].driver.semin(0);
  motors[2].driver.rms_current(1000);
  motors[2].driver.sgt(12);

  motors[2].driver.en_pwm_mode(false);
  motors[2].driver.pwm_autoscale(false);
  motors[2].driver.intpol(false);

  motors[2].driver.TCOOLTHRS(0);

  motors[2].microsteps(8);
  // }
  motors[2].invert_direction = true;
  motors[2].inactivity_timeout = 0;
  motors[2].on();

  motors[3].inactivity_timeout = 0;

  lcd.clear();
  lcd.print("Homing:");

  // prepare linear (x)
  lcd.setCursor(0, 1);
  lcd.print("Platform prepare    ");

  processCommand(F("move_rot z0.08 f30")); // move arms out of the way
  processCommand(F("stop_on_stallguard e1"));
  processCommand(F("dir e0"));
  processCommand(F("move_rot e3.5 f80"));
  processCommand(F("start e"));
  delay(20); processCommand(F("wait_for_motor e"));
  processCommand(F("stop_on_stallguard e0"));


  // home platform rotation (x)
  lcd.setCursor(0, 1);
  lcd.print("Platform rotation   ");

  processCommand(F("stop_on_stallguard x0"));
  // processCommand(F("home x1 f60 g10 b0.1 w100"));
  processCommand(F("home x1 f1 g4 b0.1 w100"));
  // processCommand(F("move_rot x0.01"));
  processCommand(F("dir x1"));
  processCommand(F("do_steps x20"));
  processCommand(F("set_position x0"));
  processCommand(F("start x"));
  delay(20); processCommand(F("wait_for_motor x"));


  // home arms (z)
  lcd.setCursor(0, 1);
  lcd.print("Collector arms      ");

  processCommand(F("dir z0"));
  processCommand(F("home z0 f60 b0.5"));
  processCommand(F("stop_on_stallguard z0"));
  processCommand(F("move z0.15"));
  processCommand(F("set_position z0"));
  processCommand(F("stop_on_stallguard z1"));
  processCommand(F("move z0.8"));
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));
  motors[2].stallguard_triggered = false;


  // home linear (e)
  lcd.setCursor(0, 1);
  lcd.print("Platform rail       ");

  processCommand(F("home e f80 g60 b0.1 w100"));
  processCommand(F("move_rot e0.25"));
  processCommand(F("set_position e0"));
  processCommand(F("start e"));
  delay(20); processCommand(F("wait_for_motor e"));


  // park arms
  lcd.clear();
  lcd.print("Park arms");

  processCommand(F("move z0"));
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));

  beep();
  // // Serial.println("[XX] pre wait");
  //
  // processCommand(F("wait_for_motor z"));
  // // processCommand(F("wait_for_motor x z e"));
  //
  // // Serial.println("[XX] post wait");
  // processCommand(F("set_position z0"));
  // // Serial.println("[XX] Z position reset!");
  // motors[2].stallguard_triggered = false;
  // // engage_pinda = false;

}



// stallguard(pinda) pin change interrupt
ISR(PCINT0_vect){
  const bool sg[MOTORS_MAX] = {
    !(PINB & (1 << PINB4)), // Z_MIN, motor X is stopped by pinda
    false, // PINB & (1 << PINB5), // Y_MIN
    PINB & (1 << PINB6), // X_MIN, motor Z is stopped by microswitch
    false, // E0_MIN
  };

  // Serial.print("SG Cust Int ");
  // Serial.print(sg[0]);
  // Serial.print(sg[1]);
  // Serial.print(sg[2]);
  // Serial.println(sg[3]);

  if(!sg[0] && !sg[1] && !sg[2] && !sg[3]) return;

  // beep(30);
  for(size_t i = 0; i < MOTORS_MAX; i++){
    if(sg[i]){
      Serial.print(F("[sg] index "));
      Serial.println(i);

      if(!motors[i].is_homing){
        Serial.println(F("[sg] motor not homing, ignoring"));
        continue;
      }

      if(motors[i].ignore_stallguard_steps > 0){
        Serial.print(F("[sg] ignore sg steps "));
        Serial.println(motors[i].ignore_stallguard_steps);
        continue;
      }

      motors[i].stallguard_triggered = true;
      if(motors[i].is_expecting_stallguard()){
        Serial.println("[sg] is expected");

        motors[i].running = false;
        motors[i].pause_steps = true;
        motors[i].steps_to_do = 0;
        Serial.print("[sg] ");
        motors[i].process_next_queue_item();
        Serial.print("s2d=");
        Serial.println(motors[i].steps_to_do);
        motors[i].pause_steps = false;

      }else{
        Serial.println("[sg] unexpected");

        // motors[i].stallguard_triggered = true;
        // if(motors[i].stop_on_stallguard) motors[i].stop();
        if(motors[i].stop_on_stallguard){
          motors[i].stop();
          Serial.println("[sg] stop motor");
        }
        if(motors[i].reset_is_homed_on_stall){
          motors[i].is_homed = false;
          motors[i].planned.is_homed = false;
        }

      }

      Serial.print("[sg] is_triggered=");
      Serial.println((uint8_t)motors[i].stallguard_triggered);
    }
  }

}




void do_run_rotations(){
  if(motors[0].steps_to_do || motors[1].steps_to_do || motors[2].steps_to_do){
    beep(30);
    return;
  }
  char f_buf[32] = {0};
  char cmd_buff[64] = {0};
  uint8_t i = 0;
  char* end = nullptr;

  i = dtostrf(half_rot_no / 2, -8, 2, f_buf);
  for (size_t i = 0; i < sizeof(f_buf); i++) if(f_buf[i] == ' '){
    f_buf[i] = 0;
    break;
  }

  // direction
  end = strcpy(cmd_buff, "dir x"); end += strlen(cmd_buff);
  strcpy(end, half_rot_dir ? "1 " : "0 "); end += 2;
  strcpy(end, "y"); end += strlen("y");
  strcpy(end, half_rot_dir ? "0" : "1"); end += 1;
  strcpy(end, " z"); end += strlen(" z");
  strcpy(end, half_rot_dir ? "0" : "1"); end += 1;
  processCommand(cmd_buff);

  // rpm
  processCommand(F("rpm x0.1 y0.1 z0.1"));

  // move_ramp
  end = cmd_buff;
  memset(cmd_buff, 0, sizeof(cmd_buff));
  if(half_rot_dir){
    strcpy(end, "move_ramp a300 x"); end += strlen("move_ramp a300 x");
  }else{
    strcpy(end, "move_ramp a40 x"); end += strlen("move_ramp a40 x");
  }
  strcpy(end, f_buf); end += strlen(f_buf);
  strcpy(end, " y"); end += strlen(" y");
  strcpy(end, f_buf); end += strlen(f_buf);
  strcpy(end, " z"); end += strlen(" z");
  strcpy(end, f_buf); end += strlen(f_buf);
  processCommand(cmd_buff);

  // start motors!
  processCommand(F("start x y z"));
  half_rot_dir = !half_rot_dir;

}
const char pgmstr_do_rotations[] PROGMEM = "Do rotations!";
MenuItemCallable run_rotations(pgmstr_do_rotations, &do_run_rotations, false);



void do_run_cycle(){
  const float arms_rpm = 20;
  const float arms_rpm_to = 120;
  const float arms_accel = 0;
  const float arms_decel = 0;
  const float linear_rpm = 0;

  processCommand(F("set_position x0 y0 z0 e0"));
  processCommand(F("empty_queue x y z e"));

  processCommand(F("move_ramp_to z10.28 s20 f120 a100 d50")); // arms up
  processCommand(F("move_rot_to e-0.1 f0")); // linear above top
  processCommand(F("start z e"));
  delay(20); processCommand(F("wait_for_motor z e"));

  delay(500); // 800

  processCommand(F("move_rot_to e0.18 f0")); // linear to ready position
  // processCommand(F("move z9 f30")); // arms slowly away
  processCommand(F("move_ramp_to z-0.15 s20 f120 a50 d100")); // arms down
  processCommand(F("start z e"));
  delay(20); processCommand(F("wait_for_motor z e"));

  // do_tower_to_locked_position(control, printer)

  // beep(); delay(500);

  processCommand(F("move_rot_to e-0.2 f0"));
  processCommand(F("start e"));
  delay(20); processCommand(F("wait_for_motor e"));

  // beep(); delay(500);

  processCommand(F("move_ramp_to z1.0 s20 f120 a100 d100"));
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));

  // beep(); delay(500);

  processCommand(F("move_rot_to e1.8 f0"));
  processCommand(F("start e"));
  delay(20); processCommand(F("wait_for_motor e"));

  // beep(); delay(500);

  processCommand(F("move_ramp_to z0 s20 f120 a100 d100"));
  processCommand(F("move x-0.008"));

  processCommand(F("move_rot_to e8.32 f0"));
  processCommand(F("start z e"));
  delay(20); processCommand(F("wait_for_motor z e"));


  // back to beginning...
  delay(500);
  beep();

  processCommand(F("move_ramp_to z0.0 s20 f120 a100 d100"));
  processCommand(F("move_rot_to e0.0 f0"));
  processCommand(F("start z e"));
  delay(20); processCommand(F("wait_for_motor z e"));

  beep();
  return;

  //
  // // move up
  // processCommand(F("empty_queue z e"));
  // motors[2].plan_ramp_move_to(10.25, arms_rpm, arms_rpm_to, 100, 50); // arms up
  // motors[3].plan_rotations_to(0, linear_rpm); // linear up
  // motors[2].start();
  // motors[3].start();
  // delay(10);
  // processCommand(F("wait_for_motor z e"));
  //
  // delay(500);
  //
  // motors[3].plan_rotations_to(0.2, linear_rpm); // ready linear
  // motors[2].plan_ramp_move_to(0, arms_rpm, arms_rpm_to, 50, 100); // arms down
  // motors[2].start();
  // motors[3].start();
  // delay(10);
  // processCommand(F("wait_for_motor z e"));
  //
  // motors[3].plan_rotations_to(0, linear_rpm); // linear up
  // motors[3].start();
  // delay(10);
  // processCommand(F("wait_for_motor e"));
  //
  // motors[2].plan_ramp_move_to(1, arms_rpm, arms_rpm_to, arms_accel, arms_decel); // arms away
  // motors[2].start();
  // delay(10);
  // processCommand(F("wait_for_motor z"));
  //
  //
  // motors[3].plan_rotations_to(4, linear_rpm); // linear half way down
  // motors[3].start();
  // delay(10);
  // processCommand(F("wait_for_motor e"));
  //
  // motors[2].plan_ramp_move_to(0, arms_rpm, arms_rpm_to, arms_accel, arms_decel); // arms down
  // motors[3].plan_rotations_to(8, linear_rpm); // linear all the way down
  // motors[2].start();
  // motors[3].start();
  // delay(10);
  // processCommand(F("wait_for_motor z e"));
  //
  // beep();

}
const char pgmstr_run_cycle[] PROGMEM = "Run cycle";
MenuItemCallable run_cycle(pgmstr_run_cycle, &do_run_cycle, false);



void do_home_washer_linear(){
  // rpm e180;home e1 f180 g60 b0.25;start e;wait_for_motor e;dir e0;move_rot e0.5
  processCommand(F("empty_queue e"));
  processCommand(F("rpm e180"));
  processCommand(F("home e1 f120 g50 b0.25"));
  processCommand(F("start e"));
  processCommand(F("wait_for_motor e"));
  processCommand(F("dir e0"));
  processCommand(F("move_rot e0.25"));
  processCommand(F("wait_for_motor e"));
  processCommand(F("stop e"));
  processCommand(F("off e"));
  beep(30);
}
const char pgmstr_home_washer_linear[] PROGMEM = "Home washer linear";
MenuItemCallable item_home_washer_linear(pgmstr_home_washer_linear, &do_home_washer_linear, false);



void do_move_linear_step(){
  // rpm e180;home e1 f180 g60 b0.25;start e;wait_for_motor e;dir e0;move_rot e0.5
  // processCommand(F("move_rot x0.125 f15"));
  processCommand(F("move_ramp x0.125 s1 f10 a30 d30"));
  // processCommand(F("start x"));
  // beep(30);
}
const char pgmstr_move_linear_step[] PROGMEM = "Rotate by 1/8";
MenuItemCallable item_move_linear_step(pgmstr_move_linear_step, &do_move_linear_step, false);


bool is_washing_on(){ return digitalReadExt(PIN_WASHING); }
void do_washing_on(){ digitalWriteExt(PIN_WASHING, HIGH); }
void do_washing_off(){ digitalWriteExt(PIN_WASHING, LOW); }
const char pgmstr_washing_on[] PROGMEM = "Washing: on";
const char pgmstr_washing_off[] PROGMEM = "Washing: off";
MenuItemToggleCallable item_washing_on_off(&is_washing_on, pgmstr_washing_on, pgmstr_washing_off, &do_washing_off, &do_washing_on);

bool is_heating_on(){ return digitalReadExt(PIN_HEATING); }
void do_heating_on(){ digitalWriteExt(PIN_HEATING, HIGH); }
void do_heating_off(){ digitalWriteExt(PIN_HEATING, LOW); }
const char pgmstr_heating_on[] PROGMEM = "Heating: on";
const char pgmstr_heating_off[] PROGMEM = "Heating: off";
MenuItemToggleCallable item_heating_on_off(&is_heating_on, pgmstr_heating_on, pgmstr_heating_off, &do_heating_off, &do_heating_on);

bool is_valve_0_on(){ return digitalReadExt(PIN_VALVE_0); }
void do_valve_0_on(){ digitalWriteExt(PIN_VALVE_0, HIGH); }
void do_valve_0_off(){ digitalWriteExt(PIN_VALVE_0, LOW); }
const char pgmstr_valve_0_on[] PROGMEM = "Ventil VYPUST: on";
const char pgmstr_valve_0_off[] PROGMEM = "Ventil VYPUST: off";
MenuItemToggleCallable item_valve_0_on_off(&is_valve_0_on, pgmstr_valve_0_on, pgmstr_valve_0_off, &do_valve_0_off, &do_valve_0_on);

bool is_valve_1_on(){ return digitalReadExt(PIN_VALVE_1); }
void do_valve_1_on(){ digitalWriteExt(PIN_VALVE_1, HIGH); }
void do_valve_1_off(){ digitalWriteExt(PIN_VALVE_1, LOW); }
const char pgmstr_valve_1_on[] PROGMEM = "Ventil NAPUST: on";
const char pgmstr_valve_1_off[] PROGMEM = "Ventil NAPUST: off";
MenuItemToggleCallable item_valve_1_on_off(&is_valve_1_on, pgmstr_valve_1_on, pgmstr_valve_1_off, &do_valve_1_off, &do_valve_1_on);


void do_mode_stealth(){
  for (size_t i = 0; i < MOTORS_MAX; i++) {
    motors[i].driver.semax(0);
    motors[i].driver.semin(0);
    motors[i].driver.rms_current(800);
    motors[i].driver.sgt(6);

    motors[i].driver.en_pwm_mode(true);
    motors[i].driver.pwm_autoscale(true);
    motors[i].driver.intpol(true);

    motors[i].driver.TCOOLTHRS(0);

    motors[i].microsteps(8);
  }
}
const char pgmstr_mode_stealth[] PROGMEM = "Mode: stealth";
MenuItemCallable item_mode_stealth(pgmstr_mode_stealth, &do_mode_stealth, false);

void do_mode_normal(){
  for (size_t i = 0; i < MOTORS_MAX; i++) {
    motors[i].driver.semax(0);
    motors[i].driver.semin(0);
    motors[i].driver.rms_current(1000);
    motors[i].driver.sgt(12);

    motors[i].driver.en_pwm_mode(false);
    motors[i].driver.pwm_autoscale(false);
    motors[i].driver.intpol(false);

    motors[i].driver.TCOOLTHRS(0);

    motors[i].microsteps(8);
  }
}
const char pgmstr_mode_normal[] PROGMEM = "Mode: normal";
MenuItemCallable item_mode_normal(pgmstr_mode_normal, &do_mode_normal, false);


void do_debug_rotation_x(){
  do_mode_stealth();
  processCommand(F("rpm x0.1"));
  processCommand(F("move_ramp a300 x10"));
  processCommand(F("start x"));
  processCommand(F("wait_for_motor x"));
  beep(10);
  processCommand(F("stop x"));
}
MenuItemCallable debug_rotation_x("rpm;move_ramp x", &do_debug_rotation_x, false);


void do_debug_seq(){
  do_mode_stealth();
  processCommand(F("rpm x20"));
  processCommand(F("move_rot x0.5"));
  processCommand(F("rpm x120"));
  processCommand(F("move_rot x2"));
  // processCommand(F("start x"));
  processCommand(F("wait_for_motor x"));
  beep(10);
  processCommand(F("stop x"));
}
MenuItemCallable debug_seq("debug seq", &do_debug_seq, false);


void do_debug_wait(){
  do_mode_stealth();
  processCommand(F("rpm x120"));
  processCommand(F("start x1"));
  processCommand(F("wait x1000"));
  processCommand(F("stop x"));
  // processCommand(F("wait_for_motor x"));
  // beep(10);
}
MenuItemCallable debug_wait_1s("debug wait 1s", &do_debug_wait, false);




// main menu
MenuItem* const main_menu_items[] PROGMEM = {
  &run_cycle,
  &item_move_linear_step,
  &run_rotations,
  // &debug_rotation_x,
  // &debug_seq,
  // &debug_wait_1s,
  &item_washing_on_off,
  &item_heating_on_off,
  // &item_e0_heater_on_off,
  // &item_home_washer_linear,
  // &item_half_rot_dir,
  // &item_half_rot_no,
  // &item_rpm_start,
  // &item_rpm_target,
  &item_valve_0_on_off,
  &item_valve_1_on_off,
  // &motor_all,
  &motor_x,
  &motor_y,
  &motor_z,
  &motor_e,
  // &timer1,
  // &timer2a,
  // &timer2b,
  // &timer3,
  // &timer4,
  // &timer5,
  // &item_babystep_up,
  // &item_babystep_down,
  // &item_babystepping_xy,

  // &item_mode_stealth,
  // &item_mode_normal,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);


#endif
