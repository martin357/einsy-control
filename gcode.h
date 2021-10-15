#pragma once
#include "serial.h"


void gcode_on();
void gcode_off();
void gcode_start();
void gcode_stop();
void gcode_halt();
void gcode_run();
void gcode_rpm();
void gcode_dir();
void gcode_accel();
void gcode_decel();
void gcode_ramp_to();
void gcode_move_rot();
void gcode_move_ramp();
void gcode_home();
void gcode_print_queue();
void gcode_empty_queue();
void gcode_wait_for_motor();
void gcode_wait();
