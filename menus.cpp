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
MenuRangeMotorRPM menu_motor_speed;
MenuItem motor_speed(pgmstr_speed_rpm, &menu_motor_speed);

// motor accel
MenuRangeMotorAccel menu_motor_accel;
MenuItem motor_accel(pgmstr_accel_rpms, &menu_motor_accel);

// motor decel
MenuRangeMotorDecel menu_motor_decel;
MenuItem motor_decel(pgmstr_decel_rpms, &menu_motor_decel);

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


// motor invert direction
const char pgmstr_invert_direction_on[] PROGMEM = "Invert dir.: on";
const char pgmstr_invert_direction_off[] PROGMEM = "Invert dir.: off";
bool is_motor_invert_direction_on(){ return motors[last_entered_motor_menu].invert_direction; }
void do_motor_invert_direction_on(){ motors[last_entered_motor_menu].invert_direction = true; }
void do_motor_invert_direction_off(){ motors[last_entered_motor_menu].invert_direction = false; }
MenuItemToggleCallable motor_invert_direction_on_off(&is_motor_invert_direction_on, pgmstr_invert_direction_on,
  pgmstr_invert_direction_off, &do_motor_invert_direction_off, &do_motor_invert_direction_on);


// motor current
MenuListMotorCurrent menu_motor_current;
MenuItem motor_current(pgmstr_current, &menu_motor_current);


// motor microstepping
MenuListMotorMicrostepping menu_motor_msteps;
MenuItem motor_msteps(pgmstr_microstepping, &menu_motor_msteps);


// motor blank time
MenuListMotorBlankTime menu_motor_blank_time;
MenuItem motor_blank_time(pgmstr_blank_time, &menu_motor_blank_time);


// motor off time
MenuRangeMotorOffTime menu_motor_off_time;
MenuItem motor_off_time(pgmstr_off_time, &menu_motor_off_time);


// motor semin
MenuRangeMotorSEMIN menu_motor_semin;
MenuItem motor_semin(pgmstr_smartenergy_min, &menu_motor_semin);


// motor semax
MenuRangeMotorSEMAX menu_motor_semax;
MenuItem motor_semax(pgmstr_smartenergy_max, &menu_motor_semax);


// motor show stallguard
const char pgmstr_stallguard[] PROGMEM = "stallGuard";
const char pgmstr_show_stallguard[] PROGMEM = "Show stallGuard";
static TMC2130_n::DRV_STATUS_t mot_drv_status{0};
uint16_t get_motor_stallguard_value(){
  mot_drv_status.sr = motors[last_entered_motor_menu].driver.DRV_STATUS();
  return mot_drv_status.sg_result;
}
uint16_t get_motor_actual_current(){
  return motors[last_entered_motor_menu].driver.cs2rms(mot_drv_status.cs_actual);
}
MenuItemDynamicCallable<uint16_t> motor_stallguard_value_item(pgmstr_stallguard, &get_motor_stallguard_value);
MenuItemDynamicCallable<uint16_t> motor_actual_current_item(pgmstr_current, &get_motor_actual_current);
MenuItem* const motor_stallguard_value_items[] PROGMEM = {
  &back,
  &motor_stallguard_value_item,
  &motor_actual_current_item,
};
Menu menu_motor_stallguard_value(motor_stallguard_value_items, sizeof(motor_stallguard_value_items) / 2);
MenuItem motor_stallguard_value(pgmstr_show_stallguard, &menu_motor_stallguard_value);


// motor stallguard sensitivity
MenuRangeMotorSgThreshold menu_motor_sg_threshold;
MenuItem motor_sg_threshold(pgmstr_sg_thres, &menu_motor_sg_threshold);


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
MenuItem* const motor_stallguard_items[] PROGMEM = {
  &back,
  &motor_stallguard_value,
  &motor_sg_threshold,
  &motor_stop_on_stallguard_on_off,
  &motor_stallguard_to_serial_on_off,
};
Menu menu_motor_stallguard(motor_stallguard_items, sizeof(motor_stallguard_items) / 2);
MenuItem motor_stallguard(pgmstr_stallguard, &menu_motor_stallguard);


// motor show position
const char pgmstr_pos[] PROGMEM = "Pos";
// const char pgmstr_pos_r[] PROGMEM = "Pos r.";
// const char pgmstr_pos_us[] PROGMEM = "Pos us.";
const char pgmstr_speed[] PROGMEM = "Speed";
const char pgmstr_plan_speed[] PROGMEM = "Plan. speed";
const char pgmstr_target[] PROGMEM = "Target";
const char pgmstr_show_pos_speed[] PROGMEM = "Show pos. & speed";
int32_t get_motor_actual_position(){
  return motors[last_entered_motor_menu].position_usteps;
}
float get_motor_actual_speed(){
  return motors[last_entered_motor_menu]._rpm;
}
float get_motor_planned_speed(){
  return motors[last_entered_motor_menu].planned.rpm;
}
float get_motor_target_speed(){
  return motors[last_entered_motor_menu].target_rpm;
}
MenuItemDynamicCallable<int32_t> motor_actual_position_item(pgmstr_pos, &get_motor_actual_position);
MenuItemDynamicCallable<float> motor_actual_speed_item(pgmstr_speed, &get_motor_actual_speed);
MenuItemDynamicCallable<float> motor_planned_speed_item(pgmstr_plan_speed, &get_motor_planned_speed);
MenuItemDynamicCallable<float> motor_target_speed_item(pgmstr_target, &get_motor_target_speed);
MenuItem* const motor_position_items[] PROGMEM = {
  &back,
  &motor_actual_position_item,
  &motor_actual_speed_item,
  &motor_planned_speed_item,
  &motor_target_speed_item,
};
Menu menu_motor_position(motor_position_items, sizeof(motor_position_items) / 2);
MenuItem motor_position(pgmstr_show_pos_speed, &menu_motor_position);



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
  &motor_position,
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
  &motor_invert_direction_on_off,
};
MenuMotor menu_motor_x(MOTOR_X, motor_items, sizeof(motor_items) / 2);
MenuMotor menu_motor_y(MOTOR_Y, motor_items, sizeof(motor_items) / 2);
MenuMotor menu_motor_z(MOTOR_Z, motor_items, sizeof(motor_items) / 2);
MenuMotor menu_motor_e(MOTOR_E, motor_items, sizeof(motor_items) / 2);
MenuItem motor_x(pgmstr_motor_x, &menu_motor_x);
MenuItem motor_y(pgmstr_motor_y, &menu_motor_y);
MenuItem motor_z(pgmstr_motor_z, &menu_motor_z);
MenuItem motor_e(pgmstr_motor_e, &menu_motor_e);


// main menu
#ifndef CUSTOM_MENU
  MenuItem* const main_menu_items[] PROGMEM = {
    &motor_x,
    &motor_y,
    &motor_z,
    &motor_e,
  };
  Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);
#endif
