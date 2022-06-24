#pragma once
#include "menu_system_derivates.h"
#include "custom.h"


extern Menu main_menu;
extern Menu menu_motor_stallguard_value;
extern Menu menu_motor_position_speed;
extern Menu menu_motor_position_direction;
extern uint32_t last_menu_redraw;

extern MenuItem motor_x;
extern MenuItem motor_y;
extern MenuItem motor_z;
extern MenuItem motor_e;


extern const char pgmstr_motor_on[]; // "Driver: on"
extern const char pgmstr_motor_off[]; // "Driver: off"
extern const char pgmstr_motor_start[]; // "Run continuously"
extern const char pgmstr_motor_stop[]; // "Stop !!"
extern const char pgmstr_manual_steps[]; // "Manual steps"
extern const char pgmstr_direction_true[]; // "Direction: " MOTOR_DIR_1
extern const char pgmstr_direction_false[]; // "Direction: " MOTOR_DIR_0
extern const char pgmstr_double_edge_on[]; // "Double edge: on"
extern const char pgmstr_double_edge_off[]; // "Double edge: off"
extern const char pgmstr_vsense_on[]; // "Vsense: on"
extern const char pgmstr_vsense_off[]; // "Vsense: off"
extern const char pgmstr_pwm_mode_on[]; // "PWM Mode: on"
extern const char pgmstr_pwm_mode_off[]; // "PWM Mode: off"
extern const char pgmstr_pwm_autoscale_on[]; // "PWM Autoscale: on"
extern const char pgmstr_pwm_autoscale_off[]; // "PWM Autoscale: off"
extern const char pgmstr_interpolate_on[]; // "Interpolation: on"
extern const char pgmstr_interpolate_off[]; // "Interpolation: off"
extern const char pgmstr_invert_direction_on[]; // "Invert dir.: on"
extern const char pgmstr_invert_direction_off[]; // "Invert dir.: off"
extern const char pgmstr_stallguard[]; // "stallGuard"
extern const char pgmstr_show_stallguard[]; // "Show stallGuard"
extern const char pgmstr_stop_on_stallguard_on[]; // "Stop on sg: on"
extern const char pgmstr_stop_on_stallguard_off[]; // "Stop on sg: off"
extern const char pgmstr_echo_to_serial_on[]; // "Echo to serial: on"
extern const char pgmstr_echo_to_serial_off[]; // "Echo to serial: off"
extern const char pgmstr_motor_x[]; // "Motor X"
extern const char pgmstr_motor_y[]; // "Motor Y"
extern const char pgmstr_motor_z[]; // "Motor Z"
extern const char pgmstr_motor_e[]; // "Motor E"
