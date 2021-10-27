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
MenuItem item_half_rot_no("Half rot. no.", &menu_half_rot_no);

MenuRange<uint8_t> menu_rpm_start("RPM start:", rpm_start, 1, 255);
MenuItem item_rpm_start("RPM start", &menu_rpm_start);

MenuRange<uint8_t> menu_rpm_target("RPM target:", rpm_target, 1, 255);
MenuItem item_rpm_target("RPM target", &menu_rpm_target);


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


}



void do_run_rotations(){
  if(motors[0].steps_to_do || motors[1].steps_to_do){
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
MenuItemCallable run_rotations("Do rotations!", &do_run_rotations, false);

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
MenuItemCallable item_home_washer_linear("Home washer linear", &do_home_washer_linear, false);

// void do_babystep_up(){
//   processCommand(F("rpm x30 y30 z30"));
//   processCommand(F("dir x1 y0 z1"));
//   processCommand(F("move_rot x0.05 y0.05"));
// }
// MenuItemCallable item_babystep_up("Babystep up", &do_babystep_up, false);

// void do_babystep_down(){
//   processCommand(F("rpm x30 y30 z30"));
//   processCommand(F("dir x0 y1 z0"));
//   processCommand(F("move_rot x0.05 y0.05"));
// }
// MenuItemCallable item_babystep_down("Babystep down", &do_babystep_down, false);

bool is_washing_on(){ return digitalReadExt(PIN_WASHING); }
void do_washing_on(){ digitalWriteExt(PIN_WASHING, HIGH); }
void do_washing_off(){ digitalWriteExt(PIN_WASHING, LOW); }
MenuItemToggleCallable item_washing_on_off(&is_washing_on, "Washing: on", "Washing: off", &do_washing_off, &do_washing_on);

// bool is_e0_heater_on(){ return digitalReadExt(PIN_WATER_PUMP); }
// void do_e0_heater_on(){ digitalWriteExt(PIN_WATER_PUMP, HIGH); }
// void do_e0_heater_off(){ digitalWriteExt(PIN_WATER_PUMP, LOW); }
// MenuItemToggleCallable item_e0_heater_on_off(&is_e0_heater_on, "Heater: on", "Heater: off", &do_e0_heater_off, &do_e0_heater_on);

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
MenuItemCallable item_mode_stealth("Mode: stealth", &do_mode_stealth, false);

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
MenuItemCallable item_mode_normal("Mode: normal", &do_mode_normal, false);


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



/*
  menu motor manual steps - xy
*/
MenuMotorManualStepsXY::MenuMotorManualStepsXY():
  Menu(nullptr, 0){
    redraw_interval = 50;
  }


void MenuMotorManualStepsXY::on_enter(){
  motors[0].rpm(50.0);
  motors[1].rpm(50.0);
  lcd.clear();
}


void MenuMotorManualStepsXY::on_press(uint16_t duration){
  motors[0].stop();
  motors[1].stop();
  go_back();
}


void MenuMotorManualStepsXY::draw(bool clear){
  lcd.print("\3", 0, 0);
  lcd.print("Manual stepping");
  lcd.print(" \1");

  lcd.print("<", 0, 2);
  lcd.setCursor(8, 2);
  lcd.print((uint16_t)(motors[0].steps_to_do / 10));
  // lcd.print("rot", 8, 2);
  lcd.print(">", 19, 2);
}


void MenuMotorManualStepsXY::move(int8_t amount){
  const bool dir = amount > 0;
  const uint32_t steps = abs(amount) * motors[0].rot2usteps(0.1);

  motors[0].pause_steps = true;
  motors[1].pause_steps = true;
  if(dir == motors[0].dir()){
    motors[0].steps_to_do += steps;
    motors[1].steps_to_do += steps;

  }else{
    if(motors[0].steps_to_do > steps){
      motors[0].steps_to_do -= steps;
      motors[1].steps_to_do -= steps;

    }else{
      motors[0].steps_to_do = steps;
      motors[1].steps_to_do = steps;
      motors[0].dir(dir);
      motors[1].dir(!dir);

    }

  }
  motors[0].pause_steps = false;
  motors[1].pause_steps = false;
  motors[0].start();
  motors[1].start();
}
MenuMotorManualStepsXY menu_babystepping_xy;
MenuItem item_babystepping_xy("Babystepping", &menu_babystepping_xy);



// main menu
MenuItem* main_menu_items[] = {
  &run_rotations,
  // &debug_rotation_x,
  // &debug_seq,
  // &debug_wait_1s,
  &item_washing_on_off,
  // &item_e0_heater_on_off,
  &item_home_washer_linear,
  &item_half_rot_dir,
  &item_half_rot_no,
  // &item_rpm_start,
  &item_rpm_target,
  &item_valve_0_on_off,
  &item_valve_1_on_off,
  // &motor_all,
  &motor_x,
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
  &item_babystepping_xy,

  &item_mode_stealth,
  &item_mode_normal,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);


#endif
