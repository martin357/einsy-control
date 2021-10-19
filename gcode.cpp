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
  const int32_t value = (len < 2) ? 0 : atoi(&rx_param[i][1]);  \

#define FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE  \
  FOREACH_PARAM_AS_AXIS;  \
  const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);  \

#define FOREACH_PARAM_AS_AXIS_END  }}}
#define FOREACH_PARAM_AS_AXIS_WITH_VALUE_END  FOREACH_PARAM_AS_AXIS_END
#define FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END  FOREACH_PARAM_AS_AXIS_END
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
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_start(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  const bool old_running = motors[index].running; // we need to check against old value since start() might change it
  motors[index].start((bool)value);
  if(motors[index].steps_to_do == 0 && !old_running){
    Serial.println("got start gcode, but no steps scheduled. PNQ!");
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
  motors[index].off();
  motors[index].empty_queue();
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
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_dir(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  ADD_TO_QUEUE(SET_DIRECTION, value);
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}


void gcode_accel(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  ADD_TO_QUEUE_FLOAT(SET_ACCEL);
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_decel(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  ADD_TO_QUEUE_FLOAT(SET_DECEL);
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_ramp_to(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  ADD_TO_QUEUE_FLOAT(RAMP_TO);
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_move_rot(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  if(value > 0.0) ADD_TO_QUEUE(DO_STEPS, motors[index].rot2usteps(value));
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_move_ramp(){
  float rpm_from = 1.0;
  float rpm_to = 160; // rpm_target; // 220.0; // <-- TODO paste value to command
  float delta_rpm = rpm_to - rpm_from;
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
  if(value > 0.0){
    // Serial.print("Mot");
    // Serial.print(index);
    // Serial.print(" ");
    // Serial.println(value);


    uint16_t sps_from = motors[index].rpm2sps(rpm_from);
    uint16_t sps_to = motors[index].rpm2sps(rpm_to);
    uint16_t delta_sps = sps_to - sps_from;
    uint16_t accel_sps = motors[index].rpm2sps(accel);
    uint16_t decel_sps = motors[index].rpm2sps(decel);

    float accel_t = delta_rpm / accel;
    float decel_t = delta_rpm / decel;

    float rot = value; //5.0;
    // Serial.print("rot\t"); Serial.println(rot);
    //
    // Serial.print("sps_from\t"); Serial.println(sps_from);
    // Serial.print("sps_to\t"); Serial.println(sps_to);
    // Serial.print("accel_sps\t"); Serial.println(accel_sps);
    // Serial.print("decel_sps\t"); Serial.println(decel_sps);
    //
    // Serial.print("accel_t\t"); Serial.println(accel_t);
    // Serial.print("decel_t\t"); Serial.println(decel_t);

    uint32_t steps = motors[index].rot2usteps(rot);
    uint32_t decel_steps = (long)round(((float)sps_to * sps_to) / (2.0 * decel_sps));

    // decel_steps += 50;

    // Serial.print("decel_steps\t"); Serial.println(decel_steps);
    // Serial.print("sps_to * sps_to\t"); Serial.println(sps_to * sps_to);
    // Serial.print("(float)sps_to * sps_to\t"); Serial.println((float)sps_to * sps_to);
    // Serial.print("2.0 * decel_sps\t"); Serial.println(2.0 * decel_sps);

    //
    // check if travel distance is too short to accelerate up to the desired velocity
    //
    // if (distanceToTravel_InSteps <= (decelerationDistance_InSteps * 2L))
    //   decelerationDistance_InSteps = (distanceToTravel_InSteps / 2L);

    // rpm x1
    // ramp x120
    // move_rot x5
    // wait x2300
    // ramp x20
    // start x

    uint8_t next = motors[index].next_empty_queue_index();
    motors[index].set_queue_item(next++, MotorQueueItemType::SET_STOP_ON_STALLGUARD, 0);
    motors[index].set_queue_item(next++, MotorQueueItemType::SET_ACCEL, accel * 100);
    motors[index].set_queue_item(next++, MotorQueueItemType::SET_DECEL, decel * 100);

    motors[index].set_queue_item(next++, MotorQueueItemType::SET_RPM, rpm_from * 100);
    motors[index].set_queue_item(next++, MotorQueueItemType::RAMP_TO, rpm_to * 100);

    motors[index].set_queue_item(next++, MotorQueueItemType::DO_STEPS, steps - decel_steps);
    motors[index].set_queue_item(next++, MotorQueueItemType::RAMP_TO, rpm_from * 100);
    motors[index].set_queue_item(next++, MotorQueueItemType::DO_STEPS, decel_steps);

    // motors[index].start(false);
  }
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_home(){
  float initial_rpm = 120.0;
  float final_rpm = 40.0;
  float backstep_rot = 0.1;
  for (size_t i = 0; i < rx_params; i++) {
    const uint8_t len = strlen(rx_param[i]);
    if(len > 0){
      strToLower(rx_param[i]);
      const float value = (len < 2) ? 0.0 : atof(&rx_param[i][1]);
      switch (rx_param[i][0]) {
        case 'f': initial_rpm = value; break;
        case 'g': final_rpm = value; break;
        case 'b': backstep_rot = value; break;
      }

    }
  }

  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  // motors[index].stop();
  // motors[index].empty_queue();
  const int dir = (bool)value;
  uint8_t next = motors[index].next_empty_queue_index();


  // motors[index].set_queue_item(next++, MotorQueueItemType::SET_PRINT_STALLGUARD_TO_SERIAL, 1);
  // backstep
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_STOP_ON_STALLGUARD, 0);
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_RPM, initial_rpm * 100);
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, !(bool)dir);
  motors[index].set_queue_item(next++, MotorQueueItemType::DO_STEPS, motors[index].rot2usteps(backstep_rot));
  // motors[index].set_queue_item(next++, MotorQueueItemType::BEEP, 5);

  // fast forward until stallguard
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, (bool)dir);
  motors[index].set_queue_item(next++, MotorQueueItemType::RUN_UNTIL_STALLGUARD);
  // motors[index].set_queue_item(next++, MotorQueueItemType::BEEP, 5);

  // backstep again
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, !(bool)dir);
  motors[index].set_queue_item(next++, MotorQueueItemType::DO_STEPS, motors[index].rot2usteps(backstep_rot));
  // motors[index].set_queue_item(next++, MotorQueueItemType::BEEP, 5);

  // slow forward until stallguard
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_RPM, final_rpm * 100);
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, (bool)dir);
  motors[index].set_queue_item(next++, MotorQueueItemType::RUN_UNTIL_STALLGUARD);
  // motors[index].set_queue_item(next++, MotorQueueItemType::BEEP, 5);

  // motors[index].set_queue_item(next++, MotorQueueItemType::SET_PRINT_STALLGUARD_TO_SERIAL, 0);
  // motors[index].start(false);

  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
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
          if(motors[index].pause_steps || motors[index].running || motors[index].steps_to_do > 0 /*|| !motors[index].queue[motors[index].next_queue_index()].processed*/){
            keep_waiting = true;
            break;
          }
        }
      }
    }
    delay(10);
  }
  Serial.println(F("wait_for_motor done!!"));
}


void gcode_wait(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  ADD_TO_QUEUE(WAIT, value);
  FOREACH_PARAM_AS_AXIS_WITH_VALUE_END;
}



#undef FOREACH_PARAM_AS_AXIS
#undef FOREACH_PARAM_AS_AXIS_END
#undef FOREACH_PARAM_AS_AXIS_WITH_VALUE
#undef FOREACH_PARAM_AS_AXIS_WITH_VALUE_END
#undef FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE
#undef FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END
#undef ADD_TO_QUEUE_FLOAT
#undef ADD_TO_QUEUE
