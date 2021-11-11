#include <Arduino.h>
#include "menus.h"
#include "hardware.h"
#include "serial.h"


// motor on/off
const char pgmstr_motor_on[] PROGMEM = "Driver: on";
const char pgmstr_motor_off[] PROGMEM = "Driver: off";
bool is_motor_on(){ return motors[last_entered_motor_menu].is_on(); }
void do_motor_on(){ motors[last_entered_motor_menu].on(); }
void do_motor_off(){ motors[last_entered_motor_menu].off(); }
MenuItemToggleCallable motor_on_off(&is_motor_on, pgmstr_motor_on, pgmstr_motor_off, &do_motor_off, &do_motor_on);


// motor start/stop
const char pgmstr_motor_start[] PROGMEM = "Run continuously";
const char pgmstr_motor_stop[] PROGMEM = "Stop !!";
bool is_motor_running(){ return motors[last_entered_motor_menu].running; }
void do_motor_start(){
  motors[last_entered_motor_menu].rpm(motors[last_entered_motor_menu].rpm());
  motors[last_entered_motor_menu].start(true);
}
void do_motor_stop(){ motors[last_entered_motor_menu].stop(); }
MenuItemToggleCallable motor_start_stop(&is_motor_running, pgmstr_motor_stop, pgmstr_motor_start, &do_motor_stop, &do_motor_start);


// motor manual steps
const char pgmstr_manual_steps[] PROGMEM = "Manual steps";
MenuMotorManualSteps menu_motor_manual_steps;
MenuItem motor_manual_steps(pgmstr_manual_steps, &menu_motor_manual_steps);


// motor speed
const char pgmstr_speed[] PROGMEM = "Speed [RPM]";
MenuRangeMotorRPM menu_motor_speed;
MenuItem motor_speed(pgmstr_speed, &menu_motor_speed);

// motor accel
const char pgmstr_accel[] PROGMEM = "Accel [RPM2]";
MenuRangeMotorAccel menu_motor_accel;
MenuItem motor_accel(pgmstr_accel, &menu_motor_accel);

// motor decel
const char pgmstr_decel[] PROGMEM = "Decel [RPM2]";
MenuRangeMotorDecel menu_motor_decel;
MenuItem motor_decel(pgmstr_decel, &menu_motor_decel);

// motor direction
const char pgmstr_direction_true[] PROGMEM = "Direction: " MOTOR_DIR_1;
const char pgmstr_direction_false[] PROGMEM = "Direction: " MOTOR_DIR_0;
bool get_motor_direction(){ return motors[last_entered_motor_menu].dir(); }
void set_motor_direction_left(){ motors[last_entered_motor_menu].dir(true); }
void set_motor_direction_right(){ motors[last_entered_motor_menu].dir(false); }
MenuItemToggleCallable motor_direction(&get_motor_direction, pgmstr_direction_true, pgmstr_direction_false,
  &set_motor_direction_right, &set_motor_direction_left);


// motor double edge
const char pgmstr_double_edge_on[] PROGMEM = "Double edge: on";
const char pgmstr_double_edge_off[] PROGMEM = "Double edge: off";
bool is_motor_dedge_on(){ return motors[last_entered_motor_menu].driver.dedge(); }
void do_motor_dedge_on(){ motors[last_entered_motor_menu].driver.dedge(true); }
void do_motor_dedge_off(){ motors[last_entered_motor_menu].driver.dedge(false); }
MenuItemToggleCallable motor_dedge_on_off(&is_motor_dedge_on, pgmstr_double_edge_on, pgmstr_double_edge_off,
  &do_motor_dedge_off, &do_motor_dedge_on);


// motor vsense
const char pgmstr_vsense_on[] PROGMEM = "Vsense: on";
const char pgmstr_vsense_off[] PROGMEM = "Vsense: off";
bool is_motor_vsense_on(){ return motors[last_entered_motor_menu].driver.vsense(); }
void do_motor_vsense_on(){ motors[last_entered_motor_menu].driver.vsense(true); }
void do_motor_vsense_off(){ motors[last_entered_motor_menu].driver.vsense(false); }
MenuItemToggleCallable motor_vsense_on_off(&is_motor_vsense_on, pgmstr_vsense_on, pgmstr_vsense_off,
  &do_motor_vsense_off, &do_motor_vsense_on);


// motor enable pwm mode
const char pgmstr_pwm_mode_on[] PROGMEM = "PWM Mode: on";
const char pgmstr_pwm_mode_off[] PROGMEM = "PWM Mode: off";
bool is_motor_en_pwm_mode_on(){ return motors[last_entered_motor_menu].driver.en_pwm_mode(); }
void do_motor_en_pwm_mode_on(){ motors[last_entered_motor_menu].driver.en_pwm_mode(true); }
void do_motor_en_pwm_mode_off(){ motors[last_entered_motor_menu].driver.en_pwm_mode(false); }
MenuItemToggleCallable motor_en_pwm_mode_on_off(&is_motor_en_pwm_mode_on, pgmstr_pwm_mode_on, pgmstr_pwm_mode_off,
  &do_motor_en_pwm_mode_off, &do_motor_en_pwm_mode_on);


// motor pwm autoscale
const char pgmstr_pwm_autoscale_on[] PROGMEM = "PWM Autoscale: on";
const char pgmstr_pwm_autoscale_off[] PROGMEM = "PWM Autoscale: off";
bool is_motor_pwm_autoscale_on(){ return motors[last_entered_motor_menu].driver.pwm_autoscale(); }
void do_motor_pwm_autoscale_on(){ motors[last_entered_motor_menu].driver.pwm_autoscale(true); }
void do_motor_pwm_autoscale_off(){ motors[last_entered_motor_menu].driver.pwm_autoscale(false); }
MenuItemToggleCallable motor_pwm_autoscale_on_off(&is_motor_pwm_autoscale_on, pgmstr_pwm_autoscale_on, pgmstr_pwm_autoscale_off,
  &do_motor_pwm_autoscale_off, &do_motor_pwm_autoscale_on);


// motor interpolate
const char pgmstr_interpolate_on[] PROGMEM = "Interpolation: on";
const char pgmstr_interpolate_off[] PROGMEM = "Interpolation: off";
bool is_motor_intpol_on(){ return motors[last_entered_motor_menu].driver.intpol(); }
void do_motor_intpol_on(){ motors[last_entered_motor_menu].driver.intpol(true); }
void do_motor_intpol_off(){ motors[last_entered_motor_menu].driver.intpol(false); }
MenuItemToggleCallable motor_intpol_on_off(&is_motor_intpol_on, pgmstr_interpolate_on, pgmstr_interpolate_off,
  &do_motor_intpol_off, &do_motor_intpol_on);


// motor current
const char pgmstr_current[] PROGMEM = "Current";
MenuListMotorCurrent menu_motor_current;
MenuItem motor_current(pgmstr_current, &menu_motor_current);


// motor microstepping
const char pgmstr_microstepping[] PROGMEM = "Microstepping";
MenuListMotorMicrostepping menu_motor_msteps;
MenuItem motor_msteps(pgmstr_microstepping, &menu_motor_msteps);


// motor blank time
const char pgmstr_blank_time[] PROGMEM = "Blank time";
MenuListMotorBlankTime menu_motor_blank_time;
MenuItem motor_blank_time(pgmstr_blank_time, &menu_motor_blank_time);


// motor off time
const char pgmstr_off_time[] PROGMEM = "Off time";
MenuRangeMotorOffTime menu_motor_off_time;
MenuItem motor_off_time(pgmstr_off_time, &menu_motor_off_time);


// motor semin
const char pgmstr_semin[] PROGMEM = "SmartEnergy Min";
MenuRangeMotorSEMIN menu_motor_semin;
MenuItem motor_semin(pgmstr_semin, &menu_motor_semin);


// motor semax
const char pgmstr_semax[] PROGMEM = "SmartEnergy Max";
MenuRangeMotorSEMAX menu_motor_semax;
MenuItem motor_semax(pgmstr_semax, &menu_motor_semax);


// motor show stallguard
const char pgmstr_show_stallguard[] PROGMEM = "Show stallGuard";
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
MenuItem* const motor_stallguard_value_items[] PROGMEM = {
  &back,
  &motor_stallguard_value_item,
  &motor_actual_current_item,
};
Menu menu_motor_stallguard_value(motor_stallguard_value_items, sizeof(motor_stallguard_value_items) / 2);
MenuItem motor_stallguard_value(pgmstr_show_stallguard, &menu_motor_stallguard_value);


// motor stallguard sensitivity
const char pgmstr_stallguard_sensitivity[] PROGMEM = "Sensitivity";
MenuRangeMotorSgThreshold menu_motor_sg_threshold;
MenuItem motor_sg_threshold(pgmstr_stallguard_sensitivity, &menu_motor_sg_threshold);


// motor stop on stallguard
bool is_motor_stop_on_stallguard_on(){ return motors[last_entered_motor_menu].stop_on_stallguard; }
void do_motor_stop_on_stallguard_on(){ motors[last_entered_motor_menu].stop_on_stallguard = true; }
void do_motor_stop_on_stallguard_off(){ motors[last_entered_motor_menu].stop_on_stallguard = false; }
const char pgmstr_stop_on_stallguard_on[] PROGMEM = "Stop on sg: on";
const char pgmstr_stop_on_stallguard_off[] PROGMEM = "Stop on sg: off";
MenuItemToggleCallable motor_stop_on_stallguard_on_off(&is_motor_stop_on_stallguard_on,
  pgmstr_stop_on_stallguard_on, pgmstr_stop_on_stallguard_off, &do_motor_stop_on_stallguard_off, &do_motor_stop_on_stallguard_on);


// motor print stallguard to serial
bool is_motor_stallguard_to_serial_on(){ return motors[last_entered_motor_menu].print_stallguard_to_serial; }
void do_motor_stallguard_to_serial_on(){ motors[last_entered_motor_menu].print_stallguard_to_serial = true; }
void do_motor_stallguard_to_serial_off(){ motors[last_entered_motor_menu].print_stallguard_to_serial = false; }
const char pgmstr_echo_to_serial_on[] PROGMEM = "Echo to serial: on";
const char pgmstr_echo_to_serial_off[] PROGMEM = "Echo to serial: off";
MenuItemToggleCallable motor_stallguard_to_serial_on_off(&is_motor_stallguard_to_serial_on,
  pgmstr_echo_to_serial_on, pgmstr_echo_to_serial_off, &do_motor_stallguard_to_serial_off, &do_motor_stallguard_to_serial_on);



// motor stallguard
const char pgmstr_stallguard[] PROGMEM = "stallGuard";
MenuItem* const motor_stallguard_items[] PROGMEM = {
  &back,
  &motor_stallguard_value,
  &motor_sg_threshold,
  &motor_stop_on_stallguard_on_off,
  &motor_stallguard_to_serial_on_off,
};
Menu menu_motor_stallguard(motor_stallguard_items, sizeof(motor_stallguard_items) / 2);
MenuItem motor_stallguard(pgmstr_stallguard, &menu_motor_stallguard);


// motor
const char pgmstr_motor_x[] PROGMEM = "Motor X";
const char pgmstr_motor_y[] PROGMEM = "Motor Y";
const char pgmstr_motor_z[] PROGMEM = "Motor Z";
const char pgmstr_motor_e[] PROGMEM = "Motor E";
MenuItem* const motor_items[] PROGMEM = {
  &back,
  &motor_on_off,
  &motor_speed,
  &motor_direction,
  &motor_start_stop,
  &motor_manual_steps,
  &motor_accel,
  &motor_decel,
  &motor_stallguard,
  &motor_current,
  &motor_vsense_on_off,
  &motor_msteps,
  &motor_blank_time,
  &motor_off_time,
  &motor_dedge_on_off,
  &motor_semin,
  &motor_semax,
  &motor_en_pwm_mode_on_off,
  &motor_pwm_autoscale_on_off,
  &motor_intpol_on_off,
};
MenuMotor menu_motor_x(MOTOR_X, motor_items, sizeof(motor_items) / 2);
MenuMotor menu_motor_y(MOTOR_Y, motor_items, sizeof(motor_items) / 2);
MenuMotor menu_motor_z(MOTOR_Z, motor_items, sizeof(motor_items) / 2);
MenuMotor menu_motor_e(MOTOR_E, motor_items, sizeof(motor_items) / 2);
MenuItem motor_x(pgmstr_motor_x, &menu_motor_x);
MenuItem motor_y(pgmstr_motor_y, &menu_motor_y);
MenuItem motor_z(pgmstr_motor_z, &menu_motor_z);
MenuItem motor_e(pgmstr_motor_e, &menu_motor_e);


// OCR1A
const char pgmstr_timer1[] PROGMEM = "OCR1A";
uint16_t* timer_ptr1 = &OCR1A;
MenuRange<uint16_t> menu_timer1("OCR1A", *timer_ptr1, 1, 65535);
MenuItem timer1(pgmstr_timer1, &menu_timer1);

// OCR3A
const char pgmstr_timer3[] PROGMEM = "OCR3A";
uint16_t* timer_ptr3 = &OCR3A;
MenuRange<uint16_t> menu_timer3("OCR3A", *timer_ptr3, 1, 65535);
MenuItem timer3(pgmstr_timer3, &menu_timer3);

// OCR4A
const char pgmstr_timer4[] PROGMEM = "OCR4A";
uint16_t* timer_ptr4 = &OCR4A;
MenuRange<uint16_t> menu_timer4("OCR4A", *timer_ptr4, 1, 65535);
MenuItem timer4(pgmstr_timer4, &menu_timer4);

// OCR5A
const char pgmstr_timer5[] PROGMEM = "OCR5A";
uint16_t* timer_ptr5 = &OCR5A;
MenuRange<uint16_t> menu_timer5("OCR5A", *timer_ptr5, 1, 65535);
MenuItem timer5(pgmstr_timer5, &menu_timer5);

// OCR2A
const char pgmstr_timer2a[] PROGMEM = "OCR2A";
uint8_t* timer_ptr2a = &OCR2A;
MenuRange<uint8_t> menu_timer2a("OCR2A", *timer_ptr2a, 1, 255);
MenuItem timer2a(pgmstr_timer2a, &menu_timer2a);

// OCR2B
const char pgmstr_timer2b[] PROGMEM = "OCR2B";
uint8_t* timer_ptr2b = &OCR2B;
MenuRange<uint8_t> menu_timer2b("OCR2B", *timer_ptr2b, 1, 255);
MenuItem timer2b(pgmstr_timer2b, &menu_timer2b);



// main menu
#ifndef CUSTOM_MENU
  MenuItem* const main_menu_items[] PROGMEM = {
    &motor_x,
    &motor_y,
    &motor_z,
    &motor_e,
    &timer1,
    &timer2a,
    &timer2b,
    &timer3,
    &timer4,
    &timer5,
  };
  Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);
#endif
