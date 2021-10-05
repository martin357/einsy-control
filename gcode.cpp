#include "gcode.h"
#include "hardware.h"


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
  motors[index].start((bool)value);
  if(motors[index].steps_to_do == 0 && !motors[index].running){
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
  FOREACH_PARAM_AS_AXIS_END;
}


void gcode_rpm(){
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  ADD_TO_QUEUE_FLOAT(SET_RPM);
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
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
  Serial.println(F("motor move!"));
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE;
  if(value > 0.0){
    Serial.print("Mot");
    Serial.print(index);
    Serial.print(" ");
    Serial.println(value);

    ADD_TO_QUEUE(DO_STEPS, motors[index].rot2usteps(value));
  }
  FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END;
}


void gcode_home(){
  FOREACH_PARAM_AS_AXIS_WITH_VALUE;
  // motors[index].stop();
  // motors[index].empty_queue();
  const int dir = (bool)value;
  uint8_t next = motors[index].next_queue_index();

  motors[index].set_queue_item(next++, MotorQueueItemType::SET_PRINT_STALLGUARD_TO_SERIAL, 1);
  // backstep
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_STOP_ON_STALLGUARD, 0);
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_RPM, 12000);
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, !(bool)dir);
  motors[index].set_queue_item(next++, MotorQueueItemType::DO_STEPS, motors[index].rot2usteps(0.1));

  // fast forward until stallguard
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, (bool)dir);
  motors[index].set_queue_item(next++, MotorQueueItemType::RUN_UNTIL_STALLGUARD);

  // backstep again
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, !(bool)dir);
  motors[index].set_queue_item(next++, MotorQueueItemType::DO_STEPS, motors[index].rot2usteps(0.1));

  // slow forward until stallguard
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_RPM, 4000);
  motors[index].set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, (bool)dir);
  motors[index].set_queue_item(next++, MotorQueueItemType::RUN_UNTIL_STALLGUARD);

  motors[index].set_queue_item(next++, MotorQueueItemType::SET_PRINT_STALLGUARD_TO_SERIAL, 0);
  motors[index].start(false);

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



#undef FOREACH_PARAM_AS_AXIS
#undef FOREACH_PARAM_AS_AXIS_END
#undef FOREACH_PARAM_AS_AXIS_WITH_VALUE
#undef FOREACH_PARAM_AS_AXIS_WITH_VALUE_END
#undef FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE
#undef FOREACH_PARAM_AS_AXIS_WITH_FLOAT_VALUE_END
#undef ADD_TO_QUEUE_FLOAT
#undef ADD_TO_QUEUE
