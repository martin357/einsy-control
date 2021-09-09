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
void do_motor_start(){
  motors[last_entered_motor_menu].rpm( motors[last_entered_motor_menu].rpm() );
  motors[last_entered_motor_menu].start();
}
void do_motor_stop(){ motors[last_entered_motor_menu].stop(); }
MenuItemToggleCallable motor_start_stop(&is_motor_running, "Stop", "Run continuously", &do_motor_stop, &do_motor_start);


// motor speed
MenuRangeMotorRPM menu_motor_speed;
MenuItem motor_speed("Speed [RPM]", &menu_motor_speed);

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
MenuListMotorMicrostepping menu_motor_msteps;
MenuItem motor_msteps("Microstepping", &menu_motor_msteps);


// motor blank time
MenuListMotorBlankTime menu_motor_blank_time;
MenuItem motor_blank_time("Blank time", &menu_motor_blank_time);


// motor off time
MenuRangeMotorOffTime menu_motor_off_time;
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
  &motor_speed,
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


// OCR1A
uint16_t* timer_ptr1 = &OCR1A;
MenuRange<uint16_t> menu_timer1("OCR1A", *timer_ptr1, 1, 255);
MenuItem timer1("OCR1A", &menu_timer1);

// OCR3A
uint16_t* timer_ptr3 = &OCR3A;
MenuRange<uint16_t> menu_timer3("OCR3A", *timer_ptr3, 1, 255);
MenuItem timer3("OCR3A", &menu_timer3);

// OCR4A
uint16_t* timer_ptr4 = &OCR4A;
MenuRange<uint16_t> menu_timer4("OCR4A", *timer_ptr4, 1, 255);
MenuItem timer4("OCR4A", &menu_timer4);

// OCR5A
uint16_t* timer_ptr5 = &OCR5A;
MenuRange<uint16_t> menu_timer5("OCR5A", *timer_ptr5, 1, 255);
MenuItem timer5("OCR5A", &menu_timer5);


// main menu
MenuItem* main_menu_items[] = {
  &motor_x,
  &motor_y,
  &motor_z,
  &motor_e,
  &timer1,
  &timer3,
  &timer4,
  &timer5,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);
