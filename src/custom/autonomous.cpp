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

  pinModeOutput(PIN_VALVE_0);
  digitalWriteExt(PIN_VALVE_0, LOW);

  pinModeOutput(PIN_VALVE_1);
  digitalWriteExt(PIN_VALVE_1, LOW);



  // normal mode
  // for (size_t i = 0; i < MOTORS_MAX; i++) {
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

  processCommand(F("home e f80 g60 b0.1 w100"));
  processCommand(F("move_rot e0.25"));
  processCommand(F("set_position e0"));

  processCommand(F("dir z0"));
  processCommand(F("move_rot z-0.25 f20"));
  processCommand(F("move_rot z0.25 f40"));
  processCommand(F("set_position z0"));

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

  // move up
  processCommand(F("empty_queue z e"));
  motors[2].plan_ramp_move_to(10, arms_rpm, arms_rpm_to, arms_accel, arms_decel); // arms up
  motors[3].plan_rotations_to(0, linear_rpm); // linear up
  motors[2].start();
  motors[3].start();
  processCommand(F("wait_for_motor z e"));

  motors[3].plan_rotations_to(1.5, linear_rpm); // ready linear
  motors[2].plan_ramp_move_to(0, arms_rpm, arms_rpm_to, arms_accel, arms_decel); // arms down
  motors[2].start();
  motors[3].start();
  processCommand(F("wait_for_motor z e"));

  motors[3].plan_rotations_to(0, linear_rpm); // linear up
  motors[3].start();
  processCommand(F("wait_for_motor e"));

  motors[2].plan_ramp_move_to(1, arms_rpm, arms_rpm_to, arms_accel, arms_decel); // arms away
  motors[2].start();
  processCommand(F("wait_for_motor z"));


  motors[3].plan_rotations_to(4, linear_rpm); // linear half way down
  motors[3].start();
  processCommand(F("wait_for_motor e"));

  motors[2].plan_ramp_move_to(0, arms_rpm, arms_rpm_to, arms_accel, arms_decel); // arms down
  motors[3].plan_rotations_to(8, linear_rpm); // linear all the way down
  motors[2].start();
  motors[3].start();
  processCommand(F("wait_for_motor z e"));

  beep();

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


bool is_washing_on(){ return digitalReadExt(PIN_WASHING); }
void do_washing_on(){ digitalWriteExt(PIN_WASHING, HIGH); }
void do_washing_off(){ digitalWriteExt(PIN_WASHING, LOW); }
const char pgmstr_washing_on[] PROGMEM = "Washing: on";
const char pgmstr_washing_off[] PROGMEM = "Washing: off";
MenuItemToggleCallable item_washing_on_off(&is_washing_on, pgmstr_washing_on, pgmstr_washing_off, &do_washing_off, &do_washing_on);

bool is_valve_0_on(){ return digitalReadExt(PIN_VALVE_0); }
void do_valve_0_on(){ digitalWriteExt(PIN_VALVE_0, HIGH); }
void do_valve_0_off(){ digitalWriteExt(PIN_VALVE_0, LOW); }
MenuItemToggleCallable item_valve_0_on_off(&is_valve_0_on, "Ventil VYPUST: on", "Ventil VYPUST: off", &do_valve_0_off, &do_valve_0_on);

bool is_valve_1_on(){ return digitalReadExt(PIN_VALVE_1); }
void do_valve_1_on(){ digitalWriteExt(PIN_VALVE_1, HIGH); }
void do_valve_1_off(){ digitalWriteExt(PIN_VALVE_1, LOW); }
MenuItemToggleCallable item_valve_1_on_off(&is_valve_1_on, "Ventil NAPUST: on", "Ventil NAPUST: off", &do_valve_1_off, &do_valve_1_on);


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
  &run_rotations,
  // &debug_rotation_x,
  // &debug_seq,
  // &debug_wait_1s,
  // &item_washing_on_off,
  // &item_e0_heater_on_off,
  // &item_home_washer_linear,
  // &item_half_rot_dir,
  // &item_half_rot_no,
  // &item_rpm_start,
  // &item_rpm_target,
  // &item_valve_0_on_off,
  // &item_valve_1_on_off,
  // &motor_all,
  // &motor_x,
  // &motor_y,
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
