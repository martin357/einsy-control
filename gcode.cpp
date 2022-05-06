#include "gcode.h"
#include "hardware.h"
#include "menus.h"


#define FOREACH_PARAM_AS_AXIS  \
  for (size_t i = 0; i < rx_params; i++) {  \
    const uint8_t len = strlen(rx_param[i]);  \
    if(len > 0){  \
      strToLower(rx_param[i]);  \
      const int index = axis2motor(rx_param[i][0]);  \
      if(index > -1){  \

#define FOREACH_PARAM_AS_AXIS_WITH_VALUE  \
  FOREACH_PARAM_AS_AXIS;  \
  const int32_t value = (len < 2) ? 0 : strtol(&rx_param[i][1], nullptr, 10);  \

#define FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE  \
  FOREACH_PARAM_AS_AXIS;  \
  const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);  \

#define FOREACH_PARAM_AS_AXIS_WITH_UNSIGNED_VALUE  \
  FOREACH_PARAM_AS_AXIS;  \
  const uint32_t value = (len < 2) ? 0 : strtoul(&rx_param[i][1], nullptr, 10);  \

#define FOREACH_PARAM_AS_AXIS_END  }}}
#define FOREACH_PARAM_AS_AXIS_WITH_VALUE_END  FOREACH_PARAM_AS_AXIS_END
#define FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END  FOREACH_PARAM_AS_AXIS_END
#define FOREACH_PARAM_AS_AXIS_WITH_UNSIGNED_VALUE_END  FOREACH_PARAM_AS_AXIS_END
#define ADD_TO_QUEUE_FLOAT(t) motors[index].set_next_empty_queue_item(MotorQueueItemType::t, value * 100.0)
#define ADD_TO_QUEUE(t, v) motors[index].set_next_empty_queue_item(MotorQueueItemType::t, v)



void gcode_on(){
  FOREACH_PARAM_AS_AXIS;
  Serial.print("axis ");
  Serial.print(index);
  Serial.println(" on");
  ADD_TO_QUEUE(TURN_ON, 0);
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_off(){
  FOREACH_PARAM_AS_AXIS;
  ADD_TO_QUEUE(TURN_OFF, 0);
  motors[index].planned.is_homed = false;
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_start(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  const bool old_running = motors[index].running; // we need to check against old value since start() might change it
  motors[index].start((bool)value);
  if(motors[index].steps_to_do == 0 && !old_running){
    // Serial.println("got start gcode, but no steps scheduled. PNQ!");
    motors[index].process_next_queue_item();
  }
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_stop(){
  FOREACH_PARAM_AS_AXIS;
  ADD_TO_QUEUE(STOP, 0);
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_halt(){
  FOREACH_PARAM_AS_AXIS;
  motors[index].stop();
  // motors[index].empty_queue();
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_run(){
  FOREACH_PARAM_AS_AXIS;
  ADD_TO_QUEUE(RUN_CONTINUOUS, 0);
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_rpm(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  ADD_TO_QUEUE_FLOAT(SET_RPM);
  motors[index].planned.rpm = value;
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_dir(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  if(len >= 2 && rx_param[i][1] == '!'){
    motors[index].planned.direction = !motors[index].planned.direction;
    ADD_TO_QUEUE(SET_DIRECTION, motors[index].planned.direction);

  }else{
    ADD_TO_QUEUE(SET_DIRECTION, value);
    motors[index].planned.direction = value;

  }
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_accel(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  ADD_TO_QUEUE_FLOAT(SET_ACCEL);
  motors[index].planned.accel = value;
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_decel(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  ADD_TO_QUEUE_FLOAT(SET_DECEL);
  motors[index].planned.decel = value;
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_ramp_to(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  ADD_TO_QUEUE_FLOAT(RAMP_TO);
  motors[index].planned.rpm = value;
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_ramp_to_nq(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  motors[index].ramp_to(value);
  motors[index].planned.rpm = value;
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_do_steps(){
  FOREACH_PARAM_AS_AXIS_WITH_UNSIGNED_VALUE;
  if(value > 0){
    ADD_TO_QUEUE(DO_STEPS, value);
    motors[index].planned.position_usteps += motors[index].planned.direction ? value : -value;
  }
  FOREACH_PARAM_AS_AXIS_WITH_UNSIGNED_VALUE_END;
}


void gcode_do_steps_dir(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  if(value != 0){
    const bool direction = value > 0;
    if(direction != motors[index].planned.direction){
      ADD_TO_QUEUE(SET_DIRECTION, direction);
      motors[index].planned.direction = direction;
    }
    ADD_TO_QUEUE(DO_STEPS, value < 0 ? -value : value);
    motors[index].planned.position_usteps += direction ? value : -value;
  }
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_do_steps_to(){
  float rpm = 0.0;

  for (size_t i = 0; i < rx_params; i++) {
    const uint8_t len = strlen(rx_param[i]);
    if(len > 0){
      strToLower(rx_param[i]);
      const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);
      switch (rx_param[i][0]) {
        case 'f': rpm = value; break;
      }

    }
  }

  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  const int32_t steps_delta = value - motors[index].planned.position_usteps;
  if(steps_delta != 0){
    const bool direction = steps_delta > 0;
    if(rpm > 0.0 && rpm != motors[index].planned.rpm){
      ADD_TO_QUEUE(SET_RPM, rpm * 100.0);
      motors[index].planned.rpm = value;
    }
    if(direction != motors[index].planned.direction){
      ADD_TO_QUEUE(SET_DIRECTION, direction);
      motors[index].planned.direction = direction;
    }
    ADD_TO_QUEUE(DO_STEPS, steps_delta < 0 ? -steps_delta : steps_delta);
    motors[index].planned.position_usteps = value;
  }
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_move_rot(){
  float rpm = 0.0;

  for (size_t i = 0; i < rx_params; i++) {
    const uint8_t len = strlen(rx_param[i]);
    if(len > 0){
      strToLower(rx_param[i]);
      const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);
      switch (rx_param[i][0]) {
        case 'f': rpm = value; break;
      }

    }
  }

  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  if(value != 0.0) motors[index].plan_rotations(value, rpm);
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_move_rot_to(){
  float rpm = 0.0;

  for (size_t i = 0; i < rx_params; i++) {
    const uint8_t len = strlen(rx_param[i]);
    if(len > 0){
      strToLower(rx_param[i]);
      const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);
      switch (rx_param[i][0]) {
        case 'f': rpm = value; break;
      }

    }
  }

  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  motors[index].plan_rotations_to(value, rpm);
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_move_ramp(){
  float rpm_from = 40.0;
  float rpm_to = 160; // rpm_target; // 220.0; // <-- TODO paste value to command
  float accel = 300.0;
  float decel = 100.0;

  for (size_t i = 0; i < rx_params; i++) {
    const uint8_t len = strlen(rx_param[i]);
    if(len > 0){
      strToLower(rx_param[i]);
      const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);
      switch (rx_param[i][0]) {
        case 's': rpm_from = value; break;
        case 'f': rpm_to = value; break;
        case 'a': accel = value; break;
        case 'd': decel = value; break;
      }

    }
  }

  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  if(value != 0.0) motors[index].plan_ramp_move(value, rpm_from, rpm_to, accel, decel);
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_move_ramp_to(){
  float rpm_from = 40.0;
  float rpm_to = 160; // rpm_target; // 220.0; // <-- TODO paste value to command
  float accel = 300.0;
  float decel = 100.0;

  for (size_t i = 0; i < rx_params; i++) {
    const uint8_t len = strlen(rx_param[i]);
    if(len > 0){
      strToLower(rx_param[i]);
      const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);
      switch (rx_param[i][0]) {
        case 's': rpm_from = value; break;
        case 'f': rpm_to = value; break;
        case 'a': accel = value; break;
        case 'd': decel = value; break;
      }

    }
  }

  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  motors[index].plan_ramp_move_to(value, rpm_from, rpm_to, accel, decel);
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_move(){
  float rpm = 0.0;
  float rpm_to = 0.0;
  float accel = 0.0;
  float decel = 0.0;

  for (size_t i = 0; i < rx_params; i++) {
    const uint8_t len = strlen(rx_param[i]);
    if(len > 0){
      strToLower(rx_param[i]);
      const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);
      switch (rx_param[i][0]) {
        case 'f': rpm = value; break;
        case 'g': rpm_to = value; break;
        case 'a': accel = value; break;
        case 'd': decel = value; break;
      }

    }
  }

  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  const float motor_rpm = rpm == 0.0 ? motors[index].default_ramp_rpm_from : rpm;
  const float motor_rpm_to = rpm_to == 0.0 ? motors[index].default_ramp_rpm_to : rpm_to;

  if(motor_rpm != 0.0 && motor_rpm_to != 0.0){
    motors[index].plan_ramp_move_to(value, motor_rpm, motor_rpm_to, accel, decel);
  }else{
    motors[index].plan_rotations_to(value, motor_rpm);
  }
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_move_usteps(){
  float rpm = 0.0;
  float rpm_to = 0.0;
  float accel = 0.0;
  float decel = 0.0;

  for (size_t i = 0; i < rx_params; i++) {
    const uint8_t len = strlen(rx_param[i]);
    if(len > 0){
      strToLower(rx_param[i]);
      const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);
      switch (rx_param[i][0]) {
        case 'f': rpm = value; break;
        case 'g': rpm_to = value; break;
        case 'a': accel = value; break;
        case 'd': decel = value; break;
      }

    }
  }

  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  const float motor_rpm = rpm == 0.0 ? motors[index].default_ramp_rpm_from : rpm;
  const float motor_rpm_to = rpm_to == 0.0 ? motors[index].default_ramp_rpm_to : rpm_to;

  if(motor_rpm != 0.0 && motor_rpm_to != 0.0){
    motors[index].plan_ramp_move_to(value, motor_rpm, motor_rpm_to, accel, decel);
  }else{
    const int32_t steps_delta = value - motors[index].planned.position_usteps;
    if(steps_delta != 0){
      const bool direction = steps_delta > 0;
      if(motor_rpm > 0.0 && motor_rpm != motors[index].planned.rpm){
        ADD_TO_QUEUE(SET_RPM, motor_rpm * 100.0);
        motors[index].planned.rpm = motor_rpm;
      }
      if(direction != motors[index].planned.direction){
        ADD_TO_QUEUE(SET_DIRECTION, direction);
        motors[index].planned.direction = direction;
      }
      ADD_TO_QUEUE(DO_STEPS, steps_delta < 0 ? -steps_delta : steps_delta);
      motors[index].planned.position_usteps = value;
    }
  }
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_home(){
  float initial_rpm = 120.0;
  float final_rpm = 0.0; // 40.0;
  float initial_backstep_rot = 0.1;
  float final_backstep_rot = 0.1;
  float ramp_from = 0.0;
  uint16_t wait_duration = 50;
  for (size_t i = 0; i < rx_params; i++) {
    const uint8_t len = strlen(rx_param[i]);
    if(len > 0){
      strToLower(rx_param[i]);
      const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);
      switch (rx_param[i][0]) {
        case 'f': initial_rpm = value; break;
        case 'g': final_rpm = value; break;
        case 'i': initial_backstep_rot = value; break;
        case 'b': final_backstep_rot = value; break;
        case 'r': ramp_from = value; break;
        case 'w': wait_duration = strtoul(&rx_param[i][1], nullptr, 10); break;
      }

    }
  }

  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  motors[index].plan_home((bool)value, initial_rpm, final_rpm, initial_backstep_rot, final_backstep_rot, ramp_from, wait_duration);
  motors[index].start();
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_autohome(){
  FOREACH_PARAM_AS_AXIS;
  if(motors[index].autohome.enabled){
    motors[index].plan_autohome();
  }else{
    Serial.print(F("autohome is disabled for "));
    Serial.print(motors[index].axis);
    Serial.println(F(" axis"));
  }
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_print_queue(){
  FOREACH_PARAM_AS_AXIS;
  motors[index].debugPrintQueue();
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_empty_queue(){
  FOREACH_PARAM_AS_AXIS;
  motors[index].empty_queue();
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_print_info(){
  FOREACH_PARAM_AS_AXIS;
  motors[index].debugPrintInfo();
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_pos_usteps(){
  FOREACH_PARAM_AS_AXIS;
  Serial.println(motors[index].position_usteps);
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_pos(){
  FOREACH_PARAM_AS_AXIS;
  Serial.println(motors[index].position(), FLOAT_DECIMALS);
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_stop_on_stallguard(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  ADD_TO_QUEUE(SET_STOP_ON_STALLGUARD, value);
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_print_stallguard(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  ADD_TO_QUEUE(SET_PRINT_STALLGUARD_TO_SERIAL, value);
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_wait_for_motor(){
  bool keep_waiting = true;
  while(keep_waiting){
    keep_waiting = false;
    for (size_t i = 0; i < rx_params; i++) {
      const uint8_t len = strlen(rx_param[i]);
      if(len > 0){
        strToLower(rx_param[i]);
        const int index = axis2motor(rx_param[i][0]);
        if(index > -1){
          if(motors[index].is_busy()){
            keep_waiting = true;
            break;
          }
        }
      }
    }
    delay(10);
  }
  // Serial.println(F("wait_for_motor done!!"));
}


void gcode_wait(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  ADD_TO_QUEUE(WAIT, value);
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_beep(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  ADD_TO_QUEUE(BEEP, value > 0 ? value : 30);
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_repeat_queue(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  ADD_TO_QUEUE(REPEAT_QUEUE, value);
  motors[index].start(false);
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_set_position(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  ADD_TO_QUEUE(SET_POSITION, value);
  ADD_TO_QUEUE(SET_IS_HOMED, 1);
  motors[index].planned.position_usteps = value < 0 ? \
    -(motors[index].rot2usteps(value)) : motors[index].rot2usteps(value);
  motors[index].planned.is_homed = true;
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_set_position_usteps(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  const bool negative = value < 0;
  ADD_TO_QUEUE(SET_POSITION_USTEPS, value);
  ADD_TO_QUEUE(SET_IS_HOMED, 1);
  motors[index].planned.position_usteps = value;
  motors[index].planned.is_homed = true;
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_set_invert_direction(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  motors[index].invert_direction = (bool)value;
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_reset_steps_total(){
  FOREACH_PARAM_AS_AXIS;
  motors[index].steps_total = 0;
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_sync_position(){
  FOREACH_PARAM_AS_AXIS;
  motors[index].planned.position_usteps = motors[index].position_usteps;
  motors[index].planned.is_homed = true;
  motors[index].is_homed = true;
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_stallguard_threshold(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  ADD_TO_QUEUE(SET_STALLGUARD_THRESHOLD, value);
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_current(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  ADD_TO_QUEUE(SET_CURRENT, value);
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_microstepping(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  const bool is_pow2 = (value & (value - 1)) == 0;
  if(value < 0 || value > 256 || !is_pow2 || value == 1){
    Serial.print("invalid microstepping value ");
    Serial.println(value);
  }else{
    ADD_TO_QUEUE(SET_MICROSTEPPING, value);
  }
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_set_is_homed(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  ADD_TO_QUEUE(SET_IS_HOMED, value == 0 ? 0 : 1);
  motors[index].planned.is_homed = false;
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_set_is_homing(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  ADD_TO_QUEUE(SET_IS_HOMING, value == 0 ? 0 : 1);
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_is_homed(){
  FOREACH_PARAM_AS_AXIS;
  Serial.println(motors[index].is_homed);
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_is_busy(){
  FOREACH_PARAM_AS_AXIS;
  Serial.println(motors[index].is_busy());
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_is_homing(){
  FOREACH_PARAM_AS_AXIS;
  Serial.println(motors[index].is_homing);
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_set_default_ramp_rpm_from(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  motors[index].default_ramp_rpm_from = value;
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_set_default_ramp_rpm_to(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  motors[index].default_ramp_rpm_to = value;
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_test_sg(){
  const uint8_t wait_duration = 50;
  float rpm = 120;
  float backstep_rot = 0.1;

  for (size_t i = 0; i < rx_params; i++) {
    const uint8_t len = strlen(rx_param[i]);
    if(len > 0){
      strToLower(rx_param[i]);
      const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);
      switch (rx_param[i][0]) {
        case 'f': rpm = value; break;
        case 'b': backstep_rot = value; break;
      }
    }
  }

  const uint32_t backstep_duration = backstep_rot / rpm * 60000.0;

  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  const uint32_t backstep_usteps = motors[index].rot2usteps(backstep_rot);
  const bool direction = (bool)value;
  // Serial.print(F("backstep_duration="));
  // Serial.println(backstep_duration);

  // uint8_t next = motors[index].next_empty_queue_index();

  // backstep
  // set_queue_item(next++, MotorQueueItemType::SET_RPM, rpm * 100);
  // set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, !direction);
  // set_queue_item(next++, MotorQueueItemType::DO_STEPS, rot2usteps(backstep_rot));
  // set_queue_item(next++, MotorQueueItemType::WAIT, wait_duration);
  // Serial.println(F("[x] plan backstep"));
  ADD_TO_QUEUE(SET_RPM, rpm * 100);
  ADD_TO_QUEUE(SET_DIRECTION, !direction);
  ADD_TO_QUEUE(DO_STEPS, backstep_usteps);
  ADD_TO_QUEUE(WAIT, wait_duration);
  // ADD_TO_QUEUE(WAIT, 1000);

  // Serial.println(F("[x] start backstep"));
  // beep(10);
  motors[index].start();
  delay(10); // let process_next_queue_item() take place
  while(motors[index].is_busy()) delay(1);
  // Serial.println(F("[x] backstep done"));
  // beep(10);
  // delay(100);
  // beep(10);

  // delay(1500);

  // Serial.println(F("[x] plan sg move"));
  // move forward until stallguard or timeout
  ADD_TO_QUEUE(SET_DIRECTION, direction);
  ADD_TO_QUEUE(RUN_UNTIL_STALLGUARD, 2000);
  // ADD_TO_QUEUE(BEEP, 5);

  // // backstep again
  // ADD_TO_QUEUE(SET_DIRECTION, !direction);
  // ADD_TO_QUEUE(DO_STEPS, rot2usteps(backstep_rot));
  // // ADD_TO_QUEUE(BEEP, 5);
  // ADD_TO_QUEUE(WAIT, wait_duration);
  //
  // // slow forward until stallguard
  // ADD_TO_QUEUE(SET_RPM, final_rpm * 100);
  // ADD_TO_QUEUE(SET_DIRECTION, direction);
  // ADD_TO_QUEUE(RUN_UNTIL_STALLGUARD);
  // // ADD_TO_QUEUE(BEEP, 5);
  //
  // // ADD_TO_QUEUE(SET_STOP_ON_STALLGUARD, 1);
  // // ADD_TO_QUEUE(SET_PRINT_STALLGUARD_TO_SERIAL, 0);
  // ADD_TO_QUEUE(SET_IS_HOMED, 1);
  // ADD_TO_QUEUE(SET_POSITION, 0);

  // ADD_TO_QUEUE(RUN_UNTIL_STALLGUARD, 2000);
  // Serial.println(F("[x] start sg move"));
  // beep(10);
  motors[index].stallguard_triggered = false;
  motors[index].start();
  delay(10); // let process_next_queue_item() take place

  const uint32_t start_time = millis();
  while(motors[index].is_busy()) delay(1);
  // Serial.println(F("[x] sg move finished"));
  // beep(10);

  const bool triggered = motors[index].stallguard_triggered;
  motors[index].stallguard_triggered = false;
  motors[index].stop_at_millis = 0;

  const uint32_t trigger_time = millis() - start_time;

  if(triggered){
    if(trigger_time < backstep_duration / 2){
      Serial.print(F("[sg test] Triggered too early! (in "));
      Serial.print(trigger_time);
      Serial.println(F("ms)"));

    }else if(trigger_time > backstep_duration * 2){
      Serial.print(F("[sg test] Trigger took too long! (in "));
      Serial.print(trigger_time);
      Serial.println(F("ms)"));

    }else{
      Serial.print(F("[sg test] Triggered in "));
      Serial.print(trigger_time);
      Serial.println(F("ms"));

    }

  }else{
    Serial.println(F("[sg test] not triggered (timeouted...) x.x"));

  }
  // Serial.println();
  // Serial.print(F("stallguard_triggered="));
  // Serial.println((uint8_t)triggered);

  // Serial.println(F(">>> run until stallguard <<<"));
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}



#undef FOREACH_PARAM_AS_AXIS
#undef FOREACH_PARAM_AS_AXIS_END
#undef FOREACH_PARAM_AS_AXIS_WITH_VALUE
#undef FOREACH_PARAM_AS_AXIS_WITH_VALUE_END
#undef FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE
#undef FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END
#undef FOREACH_PARAM_AS_AXIS_WITH_UNSIGNED_VALUE
#undef FOREACH_PARAM_AS_AXIS_WITH_UNSIGNED_VALUE_END
#undef ADD_TO_QUEUE_FLOAT
#undef ADD_TO_QUEUE
