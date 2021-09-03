// #include <avr/pgmspace.h>
#include <Arduino.h>
#include "src/LiquidCrystal_Prusa.h"
// #include <inttypes.h>
#include "menus.h"
#include "hardware.h"
// #include <Arduino.h>


// motor on/off
bool is_motor_on(){ return motors[last_entered_motor_menu].is_on(); }
void do_motor_on(){ motors[last_entered_motor_menu].on(); }
void do_motor_off(){ motors[last_entered_motor_menu].off(); }
MenuItemToggleCallable motor_on_off(&is_motor_on, "Driver: on", "Driver: off", &do_motor_off, &do_motor_on);


// motor start/stop
bool is_motor_running(){ return motors[last_entered_motor_menu].running; }
void do_motor_start(){ motors[last_entered_motor_menu].start(); }
void do_motor_stop(){ motors[last_entered_motor_menu].stop(); }
MenuItemToggleCallable motor_start_stop(&is_motor_running, "Stop", "Run continuously", &do_motor_stop, &do_motor_start);


// motor direction
bool get_motor_direction(){ return motors[last_entered_motor_menu].dir(); }
void set_motor_direction_left(){ motors[last_entered_motor_menu].dir(true); }
void set_motor_direction_right(){ motors[last_entered_motor_menu].dir(false); }
MenuItemToggleCallable motor_direction(&get_motor_direction, "Direction: left", "Direction: right", &set_motor_direction_right, &set_motor_direction_left);


// motor double edge
bool is_motor_dedge_on(){ return motors[last_entered_motor_menu].driver.dedge(); }
void do_motor_dedge_on(){ motors[last_entered_motor_menu].driver.dedge(true); }
void do_motor_dedge_off(){ motors[last_entered_motor_menu].driver.dedge(false); }
MenuItemToggleCallable motor_dedge_on_off(&is_motor_dedge_on, "Double edge: on", "Double edge: off", &do_motor_dedge_off, &do_motor_dedge_on);


// motor microstepping
void set_motor_msteps(uint16_t msteps){ motors[last_entered_motor_menu].driver.microsteps(msteps); }
MenuItemCallableArg<uint16_t> motor_msteps_256("256", &set_motor_msteps, 256);
MenuItemCallableArg<uint16_t> motor_msteps_128("128", &set_motor_msteps, 128);
MenuItemCallableArg<uint16_t> motor_msteps_64("64", &set_motor_msteps, 64);
MenuItemCallableArg<uint16_t> motor_msteps_32("32", &set_motor_msteps, 32);
MenuItemCallableArg<uint16_t> motor_msteps_16("16", &set_motor_msteps, 16);
MenuItemCallableArg<uint16_t> motor_msteps_8("8", &set_motor_msteps, 8);
MenuItemCallableArg<uint16_t> motor_msteps_4("4", &set_motor_msteps, 4);
MenuItemCallableArg<uint16_t> motor_msteps_2("2", &set_motor_msteps, 2);
MenuItemCallableArg<uint16_t> motor_msteps_0("0", &set_motor_msteps, 0);
MenuItem* motor_msteps_items[] = {
  &motor_msteps_256,
  &motor_msteps_128,
  &motor_msteps_64,
  &motor_msteps_32,
  &motor_msteps_16,
  &motor_msteps_8,
  &motor_msteps_4,
  &motor_msteps_2,
  &motor_msteps_0,
  &back,
};
Menu menu_motor_msteps(motor_msteps_items, sizeof(motor_msteps_items) / 2);
MenuItem motor_msteps("Microstepping", &menu_motor_msteps);


// motor blank time
void set_motor_blank_time(uint8_t blank_time){ motors[last_entered_motor_menu].driver.blank_time(blank_time); }
MenuItemCallableArg<uint8_t> motor_blank_time_54("54", &set_motor_blank_time, 54);
MenuItemCallableArg<uint8_t> motor_blank_time_36("36", &set_motor_blank_time, 36);
MenuItemCallableArg<uint8_t> motor_blank_time_24("24", &set_motor_blank_time, 24);
MenuItemCallableArg<uint8_t> motor_blank_time_16("16", &set_motor_blank_time, 16);
MenuItem* motor_blank_time_items[] = {
  &motor_blank_time_54,
  &motor_blank_time_36,
  &motor_blank_time_24,
  &motor_blank_time_16,
  &back,
};
Menu menu_motor_blank_time(motor_blank_time_items, sizeof(motor_blank_time_items) / 2);
MenuItem motor_blank_time("Blank time", &menu_motor_blank_time);


// motor off time
void set_motor_off_time(uint8_t off_time){ motors[last_entered_motor_menu].driver.toff(off_time); }
MenuItemCallableArg<uint8_t> motor_off_time_15("15", &set_motor_off_time, 15);
MenuItemCallableArg<uint8_t> motor_off_time_14("14", &set_motor_off_time, 14);
MenuItemCallableArg<uint8_t> motor_off_time_13("13", &set_motor_off_time, 13);
MenuItemCallableArg<uint8_t> motor_off_time_12("12", &set_motor_off_time, 12);
MenuItemCallableArg<uint8_t> motor_off_time_11("11", &set_motor_off_time, 11);
MenuItemCallableArg<uint8_t> motor_off_time_10("10", &set_motor_off_time, 10);
MenuItemCallableArg<uint8_t> motor_off_time_9("9", &set_motor_off_time, 9);
MenuItemCallableArg<uint8_t> motor_off_time_8("8", &set_motor_off_time, 8);
MenuItemCallableArg<uint8_t> motor_off_time_7("7", &set_motor_off_time, 7);
MenuItemCallableArg<uint8_t> motor_off_time_6("6", &set_motor_off_time, 6);
MenuItemCallableArg<uint8_t> motor_off_time_5("5", &set_motor_off_time, 5);
MenuItemCallableArg<uint8_t> motor_off_time_4("4", &set_motor_off_time, 4);
MenuItemCallableArg<uint8_t> motor_off_time_3("3", &set_motor_off_time, 3);
MenuItemCallableArg<uint8_t> motor_off_time_2("2", &set_motor_off_time, 2);
MenuItemCallableArg<uint8_t> motor_off_time_1("1", &set_motor_off_time, 1);
MenuItem* motor_off_time_items[] = {
  &motor_off_time_15,
  &motor_off_time_14,
  &motor_off_time_13,
  &motor_off_time_12,
  &motor_off_time_11,
  &motor_off_time_10,
  &motor_off_time_9,
  &motor_off_time_8,
  &motor_off_time_7,
  &motor_off_time_6,
  &motor_off_time_5,
  &motor_off_time_4,
  &motor_off_time_3,
  &motor_off_time_2,
  &motor_off_time_1,
  &back,
};
Menu menu_motor_off_time(motor_off_time_items, sizeof(motor_off_time_items) / 2);
MenuItem motor_off_time("Off time", &menu_motor_off_time);


// motor show stallguard
static TMC2130_n::DRV_STATUS_t mot_drv_status{0};
uint16_t get_motor_stallguard_value(){
  mot_drv_status.sr = motors[last_entered_motor_menu].driver.DRV_STATUS();
  return mot_drv_status.sg_result;
}
uint16_t get_motor_actual_current(){
  return motors[last_entered_motor_menu].driver.cs2rms(mot_drv_status.cs_actual);
}
MenuItemDynamicCallable<uint16_t> motor_stallguard_value_item("stallGuard", &get_motor_stallguard_value);
MenuItemDynamicCallable<uint16_t> motor_actual_current_item("current", &get_motor_actual_current);
MenuItem* motor_stallguard_value_items[] = {
  &back,
  &motor_stallguard_value_item,
  &motor_actual_current_item,
};
Menu menu_motor_stallguard_value(motor_stallguard_value_items, sizeof(motor_stallguard_value_items) / 2);
MenuItem motor_stallguard_value("Show stallGuard", &menu_motor_stallguard_value);


// motor
MenuItem* motor_items[] = {
  &back,
  &motor_on_off,
  &motor_direction,
  &motor_start_stop,
  &motor_stallguard_value,
  &motor_msteps,
  &motor_blank_time,
  &motor_off_time,
  &motor_dedge_on_off,
};
MenuMotor menu_motor_x(MOTOR_X, motor_items, sizeof(motor_items) / 2);
MenuMotor menu_motor_y(MOTOR_Y, motor_items, sizeof(motor_items) / 2);
MenuMotor menu_motor_z(MOTOR_Z, motor_items, sizeof(motor_items) / 2);
MenuMotor menu_motor_e(MOTOR_E, motor_items, sizeof(motor_items) / 2);
MenuItem motor_x("Motor X", &menu_motor_x);
MenuItem motor_y("Motor Y", &menu_motor_y);
MenuItem motor_z("Motor Z", &menu_motor_z);
MenuItem motor_e("Motor E", &menu_motor_e);

// main menu
MenuItem* main_menu_items[] = {
  &motor_x,
  &motor_y,
  &motor_z,
  &motor_e,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);
