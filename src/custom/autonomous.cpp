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
// bool engage_pinda = false;

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


void home_x(bool wait = true){
  // processCommand(F("home x1 g1 f4 b0.1 w100"));
  processCommand(F("home x1 g4 f8 b0.1 w100"));
  processCommand(F("dir x1"));
  processCommand(F("do_steps x20"));
  processCommand(F("set_position x0"));
  if(wait){
    delay(20);
    processCommand(F("start x"));
    processCommand(F("wait_for_motor x"));
  }
}

void home_z(bool wait = true, bool move_away = true){
  processCommand(F("dir z0"));
  // processCommand(F("home z0 f60 b0.5"));
  processCommand(F("home z0 f60 g20 b0.3"));
  // processCommand(F("stop_on_stallguard z0"));
  processCommand(F("move z0.15"));
  processCommand(F("set_position z0"));
  // processCommand(F("stop_on_stallguard z1"));
  processCommand(F("rpm z60"));
  if(move_away){
    processCommand(F("move z0.8"));
  }
  processCommand(F("start z"));
  if(wait){
    delay(20);
    processCommand(F("wait_for_motor z"));
    motors[2].stallguard_triggered = false;
  }
}

void home_e(bool wait = true){
  processCommand(F("home e f80 g60 b0.1 w100"));
  processCommand(F("move_rot e0.25"));
  processCommand(F("set_position e0"));
  processCommand(F("start e"));
  if(wait){
    delay(20);
    processCommand(F("wait_for_motor e"));
  }
}


void do_run_cycle();
void do_hand_over_print();

void custom_gcode_home_x(){ home_x(true); }
void custom_gcode_home_z(){ home_z(true, false); }
void custom_gcode_home_e(){ home_e(true); }
void custom_gcode_collect(){ do_run_cycle(); }
void custom_gcode_hand_it_over(){ do_hand_over_print(); }


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

    motors[i].stop_on_stallguard = false;
    motors[i].reset_is_homed_on_stall = false;
    // motors[i].driver.rms_current(400);
    // motors[i].driver.sgt(6);
    // motors[i].rpm(120);
  }

  motors[0].driver.semax(5);
  motors[0].driver.semin(2);
  motors[0].driver.en_pwm_mode(true);
  motors[0].driver.pwm_autoscale(true);
  motors[0].driver.intpol(true);
  motors[0].driver.rms_current(400);
  motors[0].driver.sgt(4);
  motors[0].driver.TCOOLTHRS(460);

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

  pinModeOutput(PIN_HEATER);
  digitalWriteExt(PIN_HEATER, LOW);

  pinModeOutput(PIN_VALVE_OUT);
  digitalWriteExt(PIN_VALVE_OUT, LOW);

  pinModeOutput(PIN_VALVE_IN);
  digitalWriteExt(PIN_VALVE_IN, LOW);

  pinModeOutput(PIN_DRYING_FAN);
  digitalWriteExt(PIN_DRYING_FAN, HIGH);

  pinModeOutput(PIN_UV_LED);
  digitalWriteExt(PIN_UV_LED, HIGH);

  // setup diag pin interrupt
  PCICR |= (1 << PCIE0);
  PCMSK0 = 0;
  PCMSK0 |= (1 << PCINT4); // Z_MIN
  // PCMSK0 |= (1 << PCINT5); // Y_MIN
  PCMSK0 |= (1 << PCINT6); // X_MIN


  // normal mode
  // for (size_t i = 0; i < MOTORS_MAX; i++) {
  motors[0].inactivity_timeout = 0;

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


  if(digitalReadExt(11) == LOW){
    lcd.clear();
    lcd.print("Skip homing...");
    for (size_t i = 0; i < MOTORS_MAX; i++) motors[i].off();
    return;

  }

  lcd.clear();
  lcd.print("Homing:");

  // prepare linear (x)
  lcd.setCursor(0, 1);
  lcd.print("Platform prepare    ");

  // processCommand(F("move_rot z0.08 f30")); // move arms out of the way
  processCommand(F("move_rot z0.13 f30")); // move arms out of the way
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));

  processCommand(F("stop_on_stallguard e1"));
  processCommand(F("dir e0"));
  processCommand(F("move_rot e3.5 f80"));
  processCommand(F("start e"));
  delay(20); processCommand(F("wait_for_motor e"));
  processCommand(F("stop_on_stallguard e0"));


  // home platform rotation (x)
  lcd.setCursor(0, 1);
  lcd.print("Platform rotation   ");
  // processCommand(F("stop_on_stallguard x0"));
  // processCommand(F("home x1 f60 g10 b0.1 w100"));
  // processCommand(F("home x1 g1 f4 b0.1 w100"));
  home_x(true);

  // home arms (z)
  lcd.setCursor(0, 1);
  lcd.print("Collector arms      ");
  home_z(true, true);

  // home linear (e)
  lcd.setCursor(0, 1);
  lcd.print("Platform rail       ");
  // processCommand(F("home e f80 g60 b0.1 w100"));
  home_e(true);

  // park arms
  lcd.clear();
  lcd.print("Park arms");

  processCommand(F("move z0"));
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));

  beep();
  // // Serial.println("[XX] pre wait");
  //
  // processCommand(F("wait_for_motor z"));
  // // processCommand(F("wait_for_motor x z e"));
  //
  // // Serial.println("[XX] post wait");
  // processCommand(F("set_position z0"));
  // // Serial.println("[XX] Z position reset!");
  // motors[2].stallguard_triggered = false;
  // // engage_pinda = false;

}



// stallguard(pinda) pin change interrupt
ISR(PCINT0_vect){
  const bool sg[MOTORS_MAX] = {
    !(PINB & (1 << PINB4)), // Z_MIN, motor X is stopped by pinda
    false, // PINB & (1 << PINB5), // Y_MIN
    PINB & (1 << PINB6), // X_MIN, motor Z is stopped by microswitch
    false, // E0_MIN
  };

  // Serial.print("SG Cust Int ");
  // Serial.print(sg[0]);
  // Serial.print(sg[1]);
  // Serial.print(sg[2]);
  // Serial.println(sg[3]);

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



bool heating_is_on = false;
bool is_heater_on(){ return heating_is_on; }
void do_heater_on(){ heating_is_on = true; }
void do_heater_off(){ digitalWriteExt(PIN_HEATER, LOW); heating_is_on = false; }
const char pgmstr_heater_on[] PROGMEM = "Heater: on";
const char pgmstr_heater_off[] PROGMEM = "Heater: off";
MenuItemToggleCallable item_heater_on_off(&is_heater_on, pgmstr_heater_on, pgmstr_heater_off, &do_heater_off, &do_heater_on);


const char pgmstr_temperature[] PROGMEM = "Cilova teplota";
MenuRange<uint8_t> menu_target_temperature("Teplota: [C]", storage.target_temperature, 20, 45, true);
MenuItem item_target_temperature(pgmstr_temperature, &menu_target_temperature);


static float analog2tempBed(int raw) {
  float celsius = 0;
  byte i;

  for (i=1; i<TEMPERATURE_TABLE_EINSY_LEN; i++)
  {
    if (PGM_RD_W(temperature_table_einsy[i][0]) > raw)
    {
      celsius  = PGM_RD_W(temperature_table_einsy[i-1][1]) +
        (raw - PGM_RD_W(temperature_table_einsy[i-1][0])) *
        (float)(PGM_RD_W(temperature_table_einsy[i][1]) - PGM_RD_W(temperature_table_einsy[i-1][1])) /
        (float)(PGM_RD_W(temperature_table_einsy[i][0]) - PGM_RD_W(temperature_table_einsy[i-1][0]));
      break;
    }
  }

  // Overflow: Set to last value in the table
  if (i == TEMPERATURE_TABLE_EINSY_LEN) celsius = PGM_RD_W(temperature_table_einsy[i-1][1]);

	// temperature offset adjustment
#ifdef BED_OFFSET
	float _offset = BED_OFFSET;
	float _offset_center = BED_OFFSET_CENTER;
	float _offset_start = BED_OFFSET_START;
	float _first_koef = (_offset / 2) / (_offset_center - _offset_start);
	float _second_koef = (_offset / 2) / (100 - _offset_center);

	if (celsius >= _offset_start && celsius <= _offset_center){
		celsius = celsius + (_first_koef * (celsius - _offset_start));
	}
	else if (celsius > _offset_center && celsius <= 100){
		celsius = celsius + (_first_koef * (_offset_center - _offset_start)) + ( _second_koef * ( celsius - ( 100 - _offset_center ) )) ;
	}
	else if (celsius > 100){
		celsius = celsius + _offset;
	}
#endif

  return celsius;
}


#define THERMISTOR_CNT 3

const uint8_t samples_total = 64;
uint8_t samples_collected = 0;
double temperature_raw[THERMISTOR_CNT] = {0};
double temperature[THERMISTOR_CNT] = {0};
uint32_t last_temp_change = 0;


void loopCustom(){
  const uint32_t _millis = millis();
  for (size_t i = 0; i < THERMISTOR_CNT; i++) {
    const uint16_t uval = analogRead(A0 + i);

    if(samples_collected < samples_total){
      temperature_raw[i] = ((temperature_raw[i] * samples_collected) + (double)uval) / (samples_collected + 1);
    }else{
      temperature_raw[i] = ((temperature_raw[i] * (samples_collected - 1)) + (double)uval) / samples_collected;
    }
    temperature[i] = analog2tempBed(temperature_raw[i]);
  }
  if(samples_collected < samples_total) samples_collected++;

  if(heating_is_on && _millis > last_temp_change + 333){
    const bool heater_pin = digitalReadExt(PIN_HEATER);
    if(heater_pin){
      if(temperature[HEATER_THERM] + 2.0 >= storage.target_temperature) digitalWriteExt(PIN_HEATER, LOW);

    }else{
      if(temperature[HEATER_THERM] + 1.0 <= storage.target_temperature) digitalWriteExt(PIN_HEATER, HIGH);

    }
    last_temp_change = _millis;
  }
}


MenuItemDynamic<double> item_temp0("Temp0", temperature[0]);
MenuItemDynamic<double> item_temp1("T. zasobnik", temperature[1]);
MenuItemDynamic<double> item_temp2("T. trubka <-", temperature[2]);


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
  // processCommand(F("set_position x0 y0 z0 e0"));
  processCommand(F("empty_queue x y z e"));

  processCommand(F("move_ramp_to z10.28 s20 f120 a100 d50")); // arms up
  processCommand(F("move_rot_to e-0.1 f0")); // linear above top
  processCommand(F("start z e"));
  delay(20); processCommand(F("wait_for_motor z e"));

  delay(500); // 800

  processCommand(F("move_rot_to e0.18 f0")); // linear to ready position
  // processCommand(F("move z9 f30")); // arms slowly away
  // processCommand(F("move_ramp_to z-0.15 s20 f120 a50 d100")); // arms down (slow, not working)
  processCommand(F("move_ramp_to z-0.15 s80 f120 a80 d100")); // arms down (faster, is working)
  processCommand(F("start z e"));
  delay(20); processCommand(F("wait_for_motor z e"));

  // do_tower_to_locked_position(control, printer)

  // beep(); delay(500);

  processCommand(F("move_rot_to e-0.2 f0"));
  processCommand(F("start e"));
  delay(20); processCommand(F("wait_for_motor e"));

  // beep(); delay(500);

  processCommand(F("move_ramp_to z1.0 s20 f120 a100 d100"));
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));

  // beep(); delay(500);

  processCommand(F("move_rot_to e1.8 f0"));
  processCommand(F("start e"));
  delay(20); processCommand(F("wait_for_motor e"));

  // beep(); delay(500);
  processCommand(F("move_ramp_to z0.13 s20 f120 a100 d100"));
  processCommand(F("move x-0.008"));

  processCommand(F("move_rot_to e8.32 f0"));
  processCommand(F("start z e"));
  delay(20); processCommand(F("wait_for_motor z e"));


  // back to beginning...
  delay(2500);
  beep();

  // processCommand(F("move_ramp_to z0.0 s20 f120 a100 d100"));
  processCommand(F("move_rot_to e1.8 f0"));
  processCommand(F("start e"));
  delay(20); processCommand(F("wait_for_motor e"));

  processCommand(F("move x0"));
  processCommand(F("move_rot_to e0.0 f0"));
  processCommand(F("start x e"));
  delay(20); processCommand(F("wait_for_motor x e"));

  beep();
}
const char pgmstr_run_cycle[] PROGMEM = "Collect print";
MenuItemCallable run_cycle(pgmstr_run_cycle, &do_run_cycle, false);






// void do_home_washer_linear(){
//   // rpm e180;home e1 f180 g60 b0.25;start e;wait_for_motor e;dir e0;move_rot e0.5
//   processCommand(F("empty_queue e"));
//   processCommand(F("rpm e180"));
//   processCommand(F("home e1 f120 g50 b0.25"));
//   processCommand(F("start e"));
//   processCommand(F("wait_for_motor e"));
//   processCommand(F("dir e0"));
//   processCommand(F("move_rot e0.25"));
//   processCommand(F("wait_for_motor e"));
//   processCommand(F("stop e"));
//   processCommand(F("off e"));
//   beep(30);
// }
// const char pgmstr_home_washer_linear[] PROGMEM = "Home washer linear";
// MenuItemCallable item_home_washer_linear(pgmstr_home_washer_linear, &do_home_washer_linear, false);



void do_move_linear_step(){
  // rpm e180;home e1 f180 g60 b0.25;start e;wait_for_motor e;dir e0;move_rot e0.5
  // processCommand(F("move_rot x0.125 f15"));
  processCommand(F("move_ramp x0.125 s1 f10 a30 d30"));
  // processCommand(F("start x"));
  // beep(30);
}
const char pgmstr_move_linear_step[] PROGMEM = "Rotate by 1/8";
MenuItemCallable item_move_linear_step(pgmstr_move_linear_step, &do_move_linear_step, false);


bool is_washing_on(){ return digitalReadExt(PIN_WASHING); }
void do_washing_on(){ digitalWriteExt(PIN_WASHING, HIGH); }
void do_washing_off(){ digitalWriteExt(PIN_WASHING, LOW); }
const char pgmstr_washing_on[] PROGMEM = "Washing: on";
const char pgmstr_washing_off[] PROGMEM = "Washing: off";
MenuItemToggleCallable item_washing_on_off(&is_washing_on, pgmstr_washing_on, pgmstr_washing_off, &do_washing_off, &do_washing_on);

bool is_valve_out_on(){ return digitalReadExt(PIN_VALVE_OUT); }
void do_valve_out_on(){ digitalWriteExt(PIN_VALVE_OUT, HIGH); }
void do_valve_out_off(){ digitalWriteExt(PIN_VALVE_OUT, LOW); }
const char pgmstr_valve_out_on[] PROGMEM = "Ventil VYPUST: on";
const char pgmstr_valve_out_off[] PROGMEM = "Ventil VYPUST: off";
MenuItemToggleCallable item_valve_out_on_off(&is_valve_out_on, pgmstr_valve_out_on, pgmstr_valve_out_off, &do_valve_out_off, &do_valve_out_on);

bool is_valve_in_on(){ return digitalReadExt(PIN_VALVE_IN); }
void do_valve_in_on(){ digitalWriteExt(PIN_VALVE_IN, HIGH); }
void do_valve_in_off(){ digitalWriteExt(PIN_VALVE_IN, LOW); }
const char pgmstr_valve_in_on[] PROGMEM = "Ventil NAPUST: on";
const char pgmstr_valve_in_off[] PROGMEM = "Ventil NAPUST: off";
MenuItemToggleCallable item_valve_in_on_off(&is_valve_in_on, pgmstr_valve_in_on, pgmstr_valve_in_off, &do_valve_in_off, &do_valve_in_on);


bool is_drying_fan_on(){ return !digitalReadExt(PIN_DRYING_FAN); }
void do_drying_fan_on(){ digitalWriteExt(PIN_DRYING_FAN, LOW); }
void do_drying_fan_off(){ digitalWriteExt(PIN_DRYING_FAN, HIGH); }
const char pgmstr_drying_fan_on[] PROGMEM = "Drying fan: on";
const char pgmstr_drying_fan_off[] PROGMEM = "Drying fan: off";
MenuItemToggleCallable item_drying_fan_on_off(&is_drying_fan_on, pgmstr_drying_fan_on, pgmstr_drying_fan_off, &do_drying_fan_off, &do_drying_fan_on);

bool is_uv_led_on(){ return !digitalReadExt(PIN_UV_LED); }
void do_uv_led_on(){ digitalWriteExt(PIN_UV_LED, LOW); }
void do_uv_led_off(){ digitalWriteExt(PIN_UV_LED, HIGH); }
const char pgmstr_uv_led_on[] PROGMEM = "UV led: on";
const char pgmstr_uv_led_off[] PROGMEM = "UV led: off";
MenuItemToggleCallable item_uv_led_on_off(&is_uv_led_on, pgmstr_uv_led_on, pgmstr_uv_led_off, &do_uv_led_off, &do_uv_led_on);



void do_washing_cycle(){
  lcd.clear();
  lcd.print("Washing cycle");

  // fill water tank
  lcd.setCursor(0, 1);
  lcd.print("Fill water tank     ");
  do_valve_in_on();

  // TODO: replace with liquid level sensor
  delay(5000);

  // let water fill the pump
  lcd.setCursor(0, 1);
  lcd.print("Filling pump        ");
  do_washing_on();

  delay(8000);

  // cut off water inflow
  lcd.setCursor(0, 1);
  lcd.print("Cut off water       ");
  do_valve_in_off();

  // TODO: heating ON
  // TODO: preheat now

  // actual washing
  beep(10);
  delay(10000);
  beep(10);

  // TODO: heating OFF

  lcd.setCursor(0, 1);
  lcd.print("Empty pump          ");
  do_valve_out_on();

  delay(10000);

  do_valve_out_off();
  do_washing_off();

  lcd.clear();
  lcd.print("Done!");
  beep();
}
const char pgmstr_washing_cycle[] PROGMEM = "Washing cycle";
MenuItemCallable washing_cycle(pgmstr_washing_cycle, &do_washing_cycle, false);



void rotate_platform_by_cycles(const uint8_t cycles){
  for (size_t i = 0; i < cycles * 8; i++) {
    processCommand(F("empty_queue x e"));

    processCommand(F("move e1 f15"));
    processCommand(F("start e"));
    delay(20); processCommand(F("wait_for_motor e"));

    processCommand(F("move_ramp x0.125 s1 f10 a30 d30"));
    processCommand(F("start x"));
    delay(20); processCommand(F("wait_for_motor x"));

    processCommand(F("move e0"));
    processCommand(F("start e"));
    delay(20); processCommand(F("wait_for_motor e"));
  }
}


void do_drying_cycle(){
  const uint8_t cycles = 1;
  do_drying_fan_on();
  processCommand(F("move z0.13"));
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));

  rotate_platform_by_cycles(cycles);

  do_drying_fan_off();
  beep();
}
const char pgmstr_drying_cycle[] PROGMEM = "Do drying cycle";
MenuItemCallable drying_cycle(pgmstr_drying_cycle, &do_drying_cycle, false);


void do_curing_cycle(){
  const uint8_t cycles = 1;
  do_uv_led_on();
  processCommand(F("move z0.13"));
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));

  rotate_platform_by_cycles(cycles);

  do_uv_led_off();
  beep();
}
const char pgmstr_curing_cycle[] PROGMEM = "Do curing cycle";
MenuItemCallable curing_cycle(pgmstr_curing_cycle, &do_curing_cycle, false);



void do_arms_to_zero(){
  lcd.clear();
  lcd.print("Parking arms...");

  processCommand(F("move_ramp_to z0 s60 f120 a100 d100"));
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));
  beep();
}
const char pgmstr_arms_to_zero[] PROGMEM = "Park arms to zero";
MenuItemCallable arms_to_zero(pgmstr_arms_to_zero, &do_arms_to_zero, false);



void do_hand_over_print(){
  processCommand(F("move e1"));
  processCommand(F("start e"));
  delay(20); processCommand(F("wait_for_motor e"));

  processCommand(F("move z0.6"));
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));


  // processCommand(F("home x1 g1 f4 b0.1 w100"));
  // home_x(true);

  motors[0].driver.semax(0);
  motors[0].driver.semin(0);
  motors[0].driver.en_pwm_mode(false);
  motors[0].driver.pwm_autoscale(false);
  motors[0].driver.intpol(false);
  motors[0].driver.rms_current(800);
  // motors[0].driver.sgt(4);
  motors[0].driver.TCOOLTHRS(0);

  processCommand(F("move x0.01"));
  processCommand(F("move e0.1"));
  processCommand(F("start x e"));
  delay(20); processCommand(F("wait_for_motor x e"));

  processCommand(F("move z0.45"));
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));

  processCommand(F("move x-0.01"));
  processCommand(F("start x"));
  delay(20); processCommand(F("wait_for_motor x"));

  processCommand(F("move z-0.2"));
  processCommand(F("start z"));
  delay(20); processCommand(F("wait_for_motor z"));

  motors[0].off();
  motors[0].driver.semax(5);
  motors[0].driver.semin(2);
  motors[0].driver.en_pwm_mode(true);
  motors[0].driver.pwm_autoscale(true);
  motors[0].driver.intpol(true);
  motors[0].driver.rms_current(400);
  // motors[0].driver.sgt(4);
  motors[0].driver.TCOOLTHRS(460);

  // processCommand(F("off z"));
  processCommand(F("move e0.23"));
  processCommand(F("start e"));
  delay(20); processCommand(F("wait_for_motor e"));

  // processCommand(F("move_ramp_to z8 s20 f120 a100 d50")); // arms up
  // processCommand(F("wait x800"));
  // home_x(false);
  // processCommand(F("start x z"));
  // delay(20); processCommand(F("wait_for_motor x z"));

  motors[2].driver.semax(0);
  motors[2].driver.semin(0);
  motors[2].driver.en_pwm_mode(false);
  motors[2].driver.pwm_autoscale(false);
  motors[2].driver.intpol(false);
  motors[2].driver.rms_current(1400);
  // motors[2].driver.sgt(4);
  motors[2].driver.TCOOLTHRS(0xFFFF);

  // processCommand(F("move_ramp_to z8.5 s20 f120 a100 d50")); // arms up
  beep(5);
  // processCommand(F("move_ramp_to z2.3 s20 f120 a100 d50")); // arms up
  // processCommand(F("move_ramp_to z7 s20 f120 a100 d50")); // arms up
  processCommand(F("move z3.2 f100")); // arms somewhat up
  processCommand(F("move e0.52"));
  processCommand(F("start e z"));
  delay(20); processCommand(F("wait_for_motor e z"));
  beep(5);

  delay(100);
  motors[2].driver.semax(5);
  motors[2].driver.semin(2);
  motors[2].driver.en_pwm_mode(true);
  motors[2].driver.pwm_autoscale(true);
  motors[2].driver.intpol(true);
  motors[2].driver.rms_current(400);
  // motors[2].driver.sgt(4);
  motors[2].driver.TCOOLTHRS(460);

  beep();
}
const char pgmstr_hand_over_print[] PROGMEM = "Hand it over";
MenuItemCallable hand_over_print(pgmstr_hand_over_print, &do_hand_over_print, false);



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
  &washing_cycle,
  &drying_cycle,
  &curing_cycle,
  &hand_over_print,
  &arms_to_zero,

  &item_move_linear_step,
  &run_rotations,
  // &debug_rotation_x,
  // &debug_seq,
  // &debug_wait_1s,
  &item_washing_on_off,
  &item_heater_on_off,
  &item_target_temperature,
  // &item_e0_heater_on_off,
  // &item_home_washer_linear,
  // &item_half_rot_dir,
  // &item_half_rot_no,
  // &item_rpm_start,
  // &item_rpm_target,
  &item_valve_out_on_off,
  &item_valve_in_on_off,
  // &motor_all,
  &motor_x,
  &motor_y,
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
  &item_drying_fan_on_off,
  &item_uv_led_on_off,
  &item_temp0,
  &item_temp1,
  &item_temp2,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);


#endif
