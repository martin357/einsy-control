#include <Arduino.h>
#include "menus.h"
#include "hardware.h"


// motor on/off
bool is_motor_on(){ return motors[last_entered_motor_menu].is_on(); }
void do_motor_on(){ motors[last_entered_motor_menu].on(); }
void do_motor_off(){ motors[last_entered_motor_menu].off(); }
MenuItemToggleCallable motor_on_off(&is_motor_on, "Driver: on", "Driver: off", &do_motor_off, &do_motor_on);


// motor start/stop
bool is_motor_running(){ return motors[last_entered_motor_menu].running; }
void do_motor_start(){
  motors[last_entered_motor_menu].rpm( motors[last_entered_motor_menu].rpm() );
  motors[last_entered_motor_menu].start(true);
}
void do_motor_stop(){ motors[last_entered_motor_menu].stop(); }
MenuItemToggleCallable motor_start_stop(&is_motor_running, "Stop", "Run continuously", &do_motor_stop, &do_motor_start);


// motor speed
MenuRangeMotorRPM menu_motor_speed;
MenuItem motor_speed("Speed [RPM]", &menu_motor_speed);

// motor accel
MenuRangeMotorAccel menu_motor_accel;
MenuItem motor_accel("Accel [RPM2]", &menu_motor_accel);

// motor decel
MenuRangeMotorAccel menu_motor_decel;
MenuItem motor_decel("Decel [RPM2]", &menu_motor_decel);

// motor direction
bool get_motor_direction(){ return motors[last_entered_motor_menu].dir(); }
void set_motor_direction_left(){ motors[last_entered_motor_menu].dir(true); }
void set_motor_direction_right(){ motors[last_entered_motor_menu].dir(false); }
MenuItemToggleCallable motor_direction(&get_motor_direction, "Direction: left", "Direction: right",
  &set_motor_direction_right, &set_motor_direction_left);


// motor double edge
bool is_motor_dedge_on(){ return motors[last_entered_motor_menu].driver.dedge(); }
void do_motor_dedge_on(){ motors[last_entered_motor_menu].driver.dedge(true); }
void do_motor_dedge_off(){ motors[last_entered_motor_menu].driver.dedge(false); }
MenuItemToggleCallable motor_dedge_on_off(&is_motor_dedge_on, "Double edge: on", "Double edge: off",
  &do_motor_dedge_off, &do_motor_dedge_on);


// motor current
MenuListMotorCurrent menu_motor_current;
MenuItem motor_current("Current", &menu_motor_current);


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


// motor stallguard sensitivity
MenuRangeMotorSgThreshold menu_motor_sg_threshold;
MenuItem motor_sg_threshold("Stallgaurd sens.", &menu_motor_sg_threshold);


// motor stop on stallguard
bool is_motor_stop_on_stallguard_on(){ return motors[last_entered_motor_menu].stop_on_stallguard; }
void do_motor_stop_on_stallguard_on(){ motors[last_entered_motor_menu].stop_on_stallguard = true; }
void do_motor_stop_on_stallguard_off(){ motors[last_entered_motor_menu].stop_on_stallguard = false; }
MenuItemToggleCallable motor_stop_on_stallguard_on_off(&is_motor_stop_on_stallguard_on,
  "Stop on SG: on", "Stop on SG: off", &do_motor_stop_on_stallguard_off, &do_motor_stop_on_stallguard_on);


// motor print stallguard to serial
bool is_motor_stallguard_to_serial_on(){ return motors[last_entered_motor_menu].print_stallguard_to_serial; }
void do_motor_stallguard_to_serial_on(){ motors[last_entered_motor_menu].print_stallguard_to_serial = true; }
void do_motor_stallguard_to_serial_off(){ motors[last_entered_motor_menu].print_stallguard_to_serial = false; }
MenuItemToggleCallable motor_stallguard_to_serial_on_off(&is_motor_stallguard_to_serial_on,
  "Echo Sg2serial: on", "Echo Sg2serial: off", &do_motor_stallguard_to_serial_off, &do_motor_stallguard_to_serial_on);




// motor
MenuItem* motor_items[] = {
  &back,
  &motor_on_off,
  &motor_speed,
  &motor_direction,
  &motor_start_stop,
  &motor_accel,
  &motor_decel,
  &motor_stallguard_value,
  &motor_sg_threshold,
  &motor_stop_on_stallguard_on_off,
  &motor_stallguard_to_serial_on_off,
  &motor_current,
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
MenuRange<uint16_t> menu_timer1("OCR1A", *timer_ptr1, 1, 65535);
MenuItem timer1("OCR1A", &menu_timer1);

// OCR3A
uint16_t* timer_ptr3 = &OCR3A;
MenuRange<uint16_t> menu_timer3("OCR3A", *timer_ptr3, 1, 65535);
MenuItem timer3("OCR3A", &menu_timer3);

// OCR4A
uint16_t* timer_ptr4 = &OCR4A;
MenuRange<uint16_t> menu_timer4("OCR4A", *timer_ptr4, 1, 65535);
MenuItem timer4("OCR4A", &menu_timer4);

// OCR5A
uint16_t* timer_ptr5 = &OCR5A;
MenuRange<uint16_t> menu_timer5("OCR5A", *timer_ptr5, 1, 65535);
MenuItem timer5("OCR5A", &menu_timer5);

// OCR2A
uint8_t* timer_ptr2a = &OCR2A;
MenuRange<uint8_t> menu_timer2a("OCR2A", *timer_ptr2a, 1, 255);
MenuItem timer2a("OCR2A", &menu_timer2a);

// OCR2B
uint8_t* timer_ptr2b = &OCR2B;
MenuRange<uint8_t> menu_timer2b("OCR2B", *timer_ptr2b, 1, 255);
MenuItem timer2b("OCR2B", &menu_timer2b);



/// custom stuff
uint8_t rotations_no = 1;
MenuRange<uint8_t> menu_rotations_no("Rotations no.:", rotations_no, 1, 255);
MenuItem item_rotations_no("Rotations no.", &menu_rotations_no);


void do_run_rotations(){
  if(motors[0].steps_to_do || motors[1].steps_to_do){
    beep(30);
    return;
  }
  motors[0].on();
  motors[0].dir(!motors[0].dir());
  motors[0].steps_to_do = 200ul * motors[0].usteps;
  motors[0].steps_to_do *= rotations_no;
  motors[0].rpm(60.0);

  motors[1].on();
  motors[1].dir(!motors[0].dir());
  motors[1].steps_to_do = 200ul * motors[1].usteps;
  motors[1].steps_to_do *= rotations_no;
  motors[1].rpm(60.0);

  cli();
  motors[0].ramp_to(120.0);
  motors[1].ramp_to(120.0);
  sei();
}
MenuItemCallable run_rotations("Single rotation", &do_run_rotations, false);


// main menu
MenuItem* main_menu_items[] = {
  &item_rotations_no,
  &run_rotations,
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
