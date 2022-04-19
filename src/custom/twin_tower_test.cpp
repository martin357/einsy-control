#include "twin_tower_test.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#include "../../serial.h"
#ifdef CUSTOM_TWIN_TOWER_TEST

const uint8_t towers[] = {ML, MR};

double rpm_min = 50.0;
double rpm_optimal = 180.0; // 120.0;
double rpm_max = 180.0; // 160.0; // 80.0; // sucking resin out
double new_rpm = 0.0001;
double total_dist = 0.0;


void pump_start(bool dir, double target_rpm = 150){
  processCommand(F("halt y"));
  processCommand(F("empty_queue y"));
  processCommand(dir ? F("dir y0") : F("dir y1"));
  // processCommand(F("accel y150"));
  processCommand(F("rpm y0.01"));
  processCommand(F("start y1"));
  // processCommand(F("ramp_to y150"));
  motors[1].ramp_to(target_rpm);
}

void pump_stop(bool wait = true){
  // processCommand(F("decel y150"));
  // processCommand(F("ramp_to y0.01"));
  processCommand(F("ramp_to y0"));
  if(wait) processCommand(F("wait_for_motor y"));
}

#define PRINT_ANALOG(pin)  Serial.print("" # pin " = "); Serial.println(analogRead(pin));

void setupCustom(){
  // setup diag pin interrupt
  PCICR |= (1 << PCIE0);
  PCMSK0 = 0;
  PCMSK0 |= (1 << PCINT4); // Z_MIN
  // PCMSK0 |= (1 << PCINT5); // Y_MIN
  // PCMSK0 |= (1 << PCINT6); // X_MIN

  for(size_t i = 1; i < MOTORS_MAX; i++){
    motors[i].inactivity_timeout = 0;
  }
  motors[3].rpm(30);
  motors[3].on();

  motors[3].autohome.enabled = true;
  motors[3].autohome.direction = false;
  motors[3].autohome.initial_rpm = 50; // 80;
  motors[3].autohome.final_rpm = 10;
  motors[3].autohome.backstep_rot = 0.15;
  motors[3].autohome.ramp_from = 10;
  motors[3].autohome.wait_duration = 50;
  motors[3].stop_on_stallguard = false; // since we don't want IR gate to displace
  motors[3].stop_on_stallguard_only_when_homing = true;

  unsigned long seed = analogRead(A4) + analogRead(A6) + analogRead(A8) + analogRead(A10);
  seed += analogRead(A11) + analogRead(A14) + analogRead(A15);

  randomSeed(seed);
}


void loopCustom(){
  const uint32_t _millis = millis();

}


// stallguard(pinda) pin change interrupt
ISR(PCINT0_vect){
  const bool sg[MOTORS_MAX] = {
    false, // !(PINB & (1 << PINB4)), // Z_MIN, motor X is stopped by pinda
    false, // PINB & (1 << PINB5), // Y_MIN
    false, // PINB & (1 << PINB6), // X_MIN
    !(PINB & (1 << PINB4)), // false, // E0_MIN
  };

  Serial.print("SG Cust Int ");
  Serial.print(sg[0]);
  Serial.print(sg[1]);
  Serial.print(sg[2]);
  Serial.println(sg[3]);

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



// main menu
// void do_menu_item_template(){
// }
// const char pgmstr_menu_item_template[] PROGMEM = "menu_item_template";
// MenuItemCallable item_menu_item_template(pgmstr_menu_item_template, &do_menu_item_template, false);


void do_z_motors_off(){
  for (size_t i = 0; i < 2; i++) motors[towers[i]].off();
}
const char pgmstr_z_motors_off[] PROGMEM = "Z motors off";
MenuItemCallable item_z_motors_off(pgmstr_z_motors_off, &do_z_motors_off, false);



void do_home_coarse(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(5); // 8
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(0);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(80);

    motors[towers[i]].start(true);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_home_coarse_nowait(){ do_home_coarse(false); }
const char pgmstr_home_coarse[] PROGMEM = "home coarse";
MenuItemCallable item_home_coarse(pgmstr_home_coarse, &do_home_coarse_nowait, false);



void do_home_fast(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(5); // 8
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(0);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(120);

    motors[towers[i]].start(true);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_home_fast_nowait(){ do_home_fast(false); }
const char pgmstr_home_fast[] PROGMEM = "home fast";
MenuItemCallable item_home_fast(pgmstr_home_fast, &do_home_fast_nowait, false);



void do_home_sensitive(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(300*CURRENT_MP);
    motors[towers[i]].driver.sgt(1); // 8
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(16);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(80);

    motors[towers[i]].start(true);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_home_sensitive_nowait(){ do_home_sensitive(false); }
const char pgmstr_home_sensitive[] PROGMEM = "home sensitive";
MenuItemCallable item_home_sensitive(pgmstr_home_sensitive, &do_home_sensitive_nowait, false);



void do_coarse_backstep(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(8);
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(16);
    motors[towers[i]].dir(true);
    motors[towers[i]].rpm(120);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(0.1);
    motors[towers[i]].start(false);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_coarse_backstep_nowait(){ do_coarse_backstep(false); }
const char pgmstr_coarse_backstep[] PROGMEM = "coarse backstep";
MenuItemCallable item_coarse_backstep(pgmstr_coarse_backstep, &do_coarse_backstep_nowait, false);


void do_big_backstep(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(8);
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(16);
    motors[towers[i]].dir(true);
    motors[towers[i]].rpm(120);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(5.0);
    motors[towers[i]].start(false);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_big_backstep_nowait(){ do_big_backstep(false); }
const char pgmstr_big_backstep[] PROGMEM = "big backstep";
MenuItemCallable item_big_backstep(pgmstr_big_backstep, &do_big_backstep_nowait, false);



void do_home_fine(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(1);
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(128);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(45);

    motors[towers[i]].start(true);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_home_fine_nowait(){ do_home_fine(false); }
const char pgmstr_home_fine[] PROGMEM = "home fine";
MenuItemCallable item_home_fine(pgmstr_home_fine, &do_home_fine_nowait, false);



void do_tram_z(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(13);
    motors[towers[i]].stop_on_stallguard = false;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(256);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(20);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(0.5);
    motors[towers[i]].start(false);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_tram_z_nowait(){ do_tram_z(false); }
const char pgmstr_tram_z[] PROGMEM = "tram z";
MenuItemCallable item_tram_z(pgmstr_tram_z, &do_tram_z_nowait, false);


void do_tram_z_weak(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(300*CURRENT_MP);
    motors[towers[i]].driver.sgt(13);
    motors[towers[i]].stop_on_stallguard = false;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(256);
    motors[towers[i]].dir(false);
    motors[towers[i]].rpm(5);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(0.3);
    motors[towers[i]].start(false);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_tram_z_weak_nowait(){ do_tram_z_weak(false); }
const char pgmstr_tram_z_weak[] PROGMEM = "tram z weak";
MenuItemCallable item_tram_z_weak(pgmstr_tram_z_weak, &do_tram_z_weak_nowait, false);



void do_fine_backstep(bool wait = true){
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(5);
    motors[towers[i]].stop_on_stallguard = true;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(16);
    motors[towers[i]].dir(true);
    motors[towers[i]].rpm(50);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(0.3);
    motors[towers[i]].start(false);
  }
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
const char pgmstr_fine_backstep[] PROGMEM = "fine backstep";
MenuItemCallable item_fine_backstep(pgmstr_fine_backstep, &do_fine_backstep, false);



void do_home(){
  do_home_coarse(true);
  delay(500);

  // do_coarse_backstep(true);
  // delay(500);

  // do_home_fine(true);
  // delay(500);

  do_tram_z(true);
  delay(500);

  do_fine_backstep(true);

}
const char pgmstr_home[] PROGMEM = "home....";
MenuItemCallable item_home(pgmstr_home, &do_home, false);



void do_home_weak(){
  do_home_coarse(true);
  delay(500);

  // do_coarse_backstep(true);
  // delay(500);

  // do_home_fine(true);
  // delay(500);

  do_tram_z_weak(true);
  delay(500);

  do_fine_backstep(true);

}
void custom_gcode_home_weak(){ do_home_weak(); }
const char pgmstr_home_weak[] PROGMEM = "home weak....";
MenuItemCallable item_home_weak(pgmstr_home_weak, &do_home_weak, false);



void do_home_tilt(){
  processCommand(F("autohome e"));
  processCommand(F("move e3.8 f50"));
  processCommand(F("start e0"));
  delay(10);
  processCommand(F("wait_for_motor e"));

}
void custom_gcode_home_tilt(){ do_home_tilt(); }
const char pgmstr_home_tilt[] PROGMEM = "home tilt";
MenuItemCallable item_home_tilt(pgmstr_home_tilt, &do_home_tilt, false);



void do_random_skew(bool wait = true){
  float dist = 0.3; // + (float(random(100)) / 100.0);
  for (size_t i = 0; i < 2; i++) {
    motors[towers[i]].driver.rms_current(500*CURRENT_MP);
    motors[towers[i]].driver.sgt(8);
    motors[towers[i]].stop_on_stallguard = false;
    // motors[towers[i]].print_stallguard_to_serial = true;
    motors[towers[i]].microsteps(0);
    motors[towers[i]].dir(true);
    motors[towers[i]].rpm(80);

    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(dist);
    motors[towers[i]].start(false);
  }
  WAIT_FOR_TOWERS;
  for (size_t i = 0; i < 2; i++) {
    dist = float(random(40)) / 40.0;
    motors[towers[i]].steps_to_do = motors[towers[i]].rot2usteps(dist);
    Serial.print("motor ");
    Serial.print(motors[towers[i]].axis);
    Serial.print(" rot: ");
    Serial.println(dist);
  }
  for (size_t i = 0; i < 2; i++) motors[towers[i]].start(false);
  if(wait){
    WAIT_FOR_TOWERS;
  }
}
void do_random_skew_nowait(){ do_random_skew(false); }
const char pgmstr_random_skew[] PROGMEM = "random skew";
MenuItemCallable item_random_skew(pgmstr_random_skew, &do_random_skew_nowait, false);


bool is_invert_tower_direction_on(){ return motors[towers[0]].invert_direction; }
void do_invert_tower_direction_on(){ for (size_t i = 0; i < 2; i++) motors[towers[i]].invert_direction = true; }
void do_invert_tower_direction_off(){ for (size_t i = 0; i < 2; i++) motors[towers[i]].invert_direction = false; }
const char pgmstr_invert_tower_direction_on[] PROGMEM = "Motors invert: on";
const char pgmstr_invert_tower_direction_off[] PROGMEM = "Motors invert: off";
MenuItemToggleCallable item_invert_tower_direction_on_off(&is_invert_tower_direction_on, pgmstr_invert_tower_direction_on,
  pgmstr_invert_tower_direction_off, &do_invert_tower_direction_off, &do_invert_tower_direction_on);


bool is_print_stallguard_on(){ return motors[towers[0]].print_stallguard_to_serial; }
void do_print_stallguard_on(){ for (size_t i = 0; i < 2; i++) motors[towers[i]].print_stallguard_to_serial = true; }
void do_print_stallguard_off(){ for (size_t i = 0; i < 2; i++) motors[towers[i]].print_stallguard_to_serial = false; }
const char pgmstr_print_stallguard_on[] PROGMEM = "Print sg: on";
const char pgmstr_print_stallguard_off[] PROGMEM = "Print sg: off";
MenuItemToggleCallable item_print_stallguard_on_off(&is_print_stallguard_on, pgmstr_print_stallguard_on,
  pgmstr_print_stallguard_off, &do_print_stallguard_off, &do_print_stallguard_on);




MenuItem* const main_menu_items[] PROGMEM = {
  &item_z_motors_off,
  &item_home_fast,
  &item_home_coarse,
  &item_home_sensitive,
  &item_coarse_backstep,
  &item_big_backstep,
  &item_home_fine,
  &item_tram_z,
  &item_tram_z_weak,
  &item_fine_backstep,
  &item_home,
  &item_home_weak,
  &item_home_tilt,
  &item_random_skew,
  // &motor_x,
  &motor_y,
  &motor_z,
  &motor_e,
  &item_print_stallguard_on_off,
  &item_invert_tower_direction_on_off,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);




#endif
