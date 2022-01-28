#include "motor.h"
#include "pins.h"
#include "hardware.h"


void setupMotorTimers(){
  #if MOTORS_PRESCALER != 8
    #error MOTORS_PRESCALER is not set to 8 !!!
  #endif

  // OCR##t##A = F_CPU / MOTORS_PRESCALER / 1000;
  // TIMSK##t |= (1 << OCIE##t##A);
  // TIMSK##t = 0;
  #define SETUP_TIMER(t) \
    TCCR##t##A = 0; \
    TCCR##t##B = 0; \
    TCNT##t  = 0; \
    TCCR##t##B |= (1 << WGM##t##2); \
    TCCR##t##B |= (1 << CS##t##1);  \
    TIMSK##t = 0;

  cli();
  // setup acceleration timer
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 0xFF;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
  TIMSK2 |= (1 << OCIE2A);

  SETUP_TIMER(1);
  SETUP_TIMER(3);
  SETUP_TIMER(4);
  SETUP_TIMER(5);

  // setup diag pin interrupt
  PCICR |= (1 << PCIE2);
  PCMSK2 = 0;
  PCMSK2 |= (1 << PCINT18); // X_DIAG
  PCMSK2 |= (1 << PCINT19); // E0_DIAG
  PCMSK2 |= (1 << PCINT22); // Z_DIAG
  PCMSK2 |= (1 << PCINT23); // Y_DIAG

  // setup "wait" handling timer
  TIMSK0 |= (1 << OCIE0A);
  sei();

  #undef SETUP_TIMER

}


int8_t axis2motor(const char axis){
  for (size_t i = 0; i < MOTORS_MAX; i++) if(motors[i].axis == axis) return i;
  return -1;
}


inline float _rpm2rps(float rpm){
  return rpm / 60;
}


uint32_t _rps2sps(float rps, uint16_t usteps){
  return uint16_t(FSTEPS_PER_REVOLUTION * (usteps > 0 ? usteps : 1) * rps);
}


inline uint32_t _sps2ocr(uint16_t sps){
  return F_CPU / MOTORS_PRESCALER / sps;
}


inline uint32_t _rpm2ocr(float rpm, uint16_t usteps){
  return _rps2ocr(rpm / 60, usteps);
}


inline uint32_t _rps2ocr(float rps, uint16_t usteps){
  return F_CPU / MOTORS_PRESCALER / (FSTEPS_PER_REVOLUTION * (usteps > 0 ? usteps : 1) * rps);
}


float _ocr2rpm(uint16_t ocr, uint16_t usteps){
  return _ocr2rps(ocr, usteps) * 60;
}


float _ocr2rps(uint16_t ocr, uint16_t usteps){
  return F_CPU / MOTORS_PRESCALER / ocr / (usteps > 0 ? usteps : 1) / FSTEPS_PER_REVOLUTION;
}


uint32_t _rot2usteps(float rot, uint16_t usteps){
  return (rot < 0.0 ? -rot : rot) * (usteps > 0 ? usteps : 1) * FSTEPS_PER_REVOLUTION;
}


float _usteps2rot(uint32_t value, uint16_t usteps){
  return (float)value / (usteps > 0 ? usteps : 1) / FSTEPS_PER_REVOLUTION;
}



Motor::Motor(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, uint8_t cs_pin, uint8_t diag_pin,
    uint8_t* step_port, uint8_t step_bit, uint16_t* timer_compare_port, uint16_t* timer_counter_port,
    uint8_t* timer_enable_port, uint8_t timer_enable_bit, const char axis):
  axis(axis),
  step_pin(step_pin),
  dir_pin(dir_pin),
  enable_pin(enable_pin),
  cs_pin(cs_pin),
  diag_pin(diag_pin),
  step_port(step_port),
  step_bit(step_bit),
  driver(cs_pin, 0.2f),
  usteps(16),
  pause_steps(false),
  enabled(false),
  invert_direction(false),
  stop_on_stallguard(true),
  print_stallguard_to_serial(false),
  is_homed(false),
  is_homing(false),
  reset_is_homed_on_power_off(true),
  reset_is_homed_on_stall(true),
  inactivity_timeout(120000),
  stop_at_millis(0),
  position_usteps(0),
  running(false),
  stallguard_triggered(false),
  steps_to_do(0),
  steps_total(0),
  ignore_stallguard_steps(0),
  last_movement(0),
  planned({0.0, false, false, 0.0, 120.0, 120.0}),
  autohome({false, false, 120.0, 40.0, 0.1, 0.0, 50}),
  target_rpm(-1.0),
  accel(120),
  decel(120),
  last_speed_change(last_speed_change),
  queue_index(0),
  _rpm(0.0),
  _dir(false),
  timer_compare_port(timer_compare_port),
  timer_counter_port(timer_counter_port),
  timer_enable_port(timer_enable_port),
  timer_enable_bit(timer_enable_bit){
    pinModeOutput(enable_pin);
    pinModeOutput(dir_pin);
    pinModeOutput(step_pin);
    pinModeOutput(cs_pin);
    pinModeInput(diag_pin, true);
    digitalWriteExt(enable_pin, HIGH);
    digitalWriteExt(dir_pin, LOW);
    digitalWriteExt(step_pin, LOW);
    digitalWriteExt(cs_pin, HIGH);
  }


void Motor::on(){
  digitalWriteExt(enable_pin, LOW);
}


void Motor::off(){
  digitalWriteExt(enable_pin, HIGH);
  stop();
  if(reset_is_homed_on_power_off){
    is_homed = false;
    planned.is_homed = false;
  }
}


bool Motor::is_on(){
  return digitalReadExt(enable_pin) == LOW;
}


void Motor::start(bool start_running){
  if(!is_on()) on();
  running = start_running;
  enabled = true;
  *timer_counter_port = 0;
  *timer_enable_port |= (1 << timer_enable_bit);
  // Serial.println(F("[motor] start"));
}


void Motor::stop(){
  *timer_enable_port = 0;
  enabled = false;
  running = false;
  steps_to_do = 0;
  ramp_off();
  // Serial.println(F("[motor] stop"));
}


void Motor::step(){
  *step_port ^= 1 << step_bit;
  delayMicroseconds(2);
  *step_port ^= 1 << step_bit;
  steps_total++;
  position_usteps += _dir ? 1 : -1;
  if(ignore_stallguard_steps > 0) ignore_stallguard_steps--;
}


bool Motor::dir(){
  return invert_direction ? !_dir : _dir;
}


void Motor::dir(bool direction){
  _dir = invert_direction ? !direction : direction;
  digitalWriteExt(dir_pin, _dir);
}


uint16_t Motor::sg_value(){
  TMC2130_n::DRV_STATUS_t drv_status{0};
  drv_status.sr = driver.DRV_STATUS();
  return drv_status.sg_result;
}


void Motor::microsteps(uint16_t microstepping){
  driver.microsteps(microstepping);
  usteps = driver.microsteps();
  rpm(_rpm); // update timer

  if(millis() > 300){
    SERIAL_PRINT("new usteps: ");
    SERIAL_PRINTLN(usteps);
  }
}


void Motor::rpm(float value){
  _rpm = value;
  uint32_t ocr = rpm2ocr(value);
  if(ocr < 70) ocr = 70;
  if(ocr > 65535) ocr = 65535;

  if(value <= 0.0){
    stop();
    // if(millis() > 300) SERIAL_PRINTLN("low rpm - stop!");
  }

  if(*timer_compare_port != ocr ){
    // cli();
    // *timer_counter_port = 0;
    *timer_compare_port = ocr;
    // sei();

    // static uint32_t last_millis = 0;
    // uint32_t _millis = millis();
    // if(_millis > 300){
    // // if(1){
    //   SERIAL_PRINT("new ocr=");
    //   SERIAL_PRINT(ocr);
    //   last_millis = _millis;
    //   // SERIAL_PRINT("\tnew rpm=");
    //   // SERIAL_PRINT(new_rpm);
    //   SERIAL_PRINTLN();
    // }
  }
}


float Motor::rpm(){
  return _rpm;
}


void Motor::ramp_to(float value, bool keep_running){
  last_speed_change = millis();
  target_rpm = value;
}


void Motor::ramp_off(){
  target_rpm = -1.0;
  last_speed_change = millis();
}


bool Motor::is_busy(){
  const uint8_t next = next_queue_index();
  return pause_steps || running || steps_to_do > 0 ||
    (queue[queue_index].type == MotorQueueItemType::WAIT_IN_PROGRESS &&
      !queue[queue_index].processed) ||
    (queue[next].type != MotorQueueItemType::NOOP && !queue[next].processed);
}


bool Motor::is_expecting_stallguard(){
  return (queue[queue_index].type == MotorQueueItemType::RUN_UNTIL_STALLGUARD);
}


MotorStallguardInfo Motor::get_stallguard_info(){
  TMC2130_n::DRV_STATUS_t drv_status{0};
  drv_status.sr = driver.DRV_STATUS();

  MotorStallguardInfo result{0};
  result.sg_result = drv_status.sg_result;
  result.fsactive = drv_status.fsactive;
  result.cs_actual = drv_status.cs_actual;
  result.rms = driver.cs2rms(drv_status.cs_actual);
  return result;
}


uint32_t Motor::rot2usteps(float rot){
  return _rot2usteps(rot, usteps);
}


float Motor::usteps2rot(uint32_t value){
  return _usteps2rot(value, usteps);
}


uint32_t Motor::rpm2ocr(float rpm){
  return _rpm2ocr(rpm, usteps);
}


uint32_t Motor::rpm2sps(float rps){
  return FSTEPS_PER_REVOLUTION * (usteps > 0 ? usteps : 1) * rps / 60;
}


float Motor::position(){
  const bool negative = position_usteps < 0;
  const uint32_t steps = negative ? -position_usteps : position_usteps;
  return negative ? -usteps2rot(steps) : usteps2rot(steps);
}


uint8_t Motor::next_queue_index(){
  return queue_index + 1 >= MOTOR_QUEUE_LEN ? 0 : queue_index + 1;
}


int16_t Motor::next_empty_queue_index(){
  uint8_t next = queue_index;
  for (size_t i = 0; i < MOTOR_QUEUE_LEN - 1; i++) {
    if(++next >= MOTOR_QUEUE_LEN) next = 0;
    if(queue[next].processed || queue[next].type == MotorQueueItemType::NOOP) return next;
  }
  return -1;
}


void Motor::set_queue_item(uint8_t index, MotorQueueItemType type, uint32_t value){
  // while(index >= MOTOR_QUEUE_LEN) index -= MOTOR_QUEUE_LEN;
  while(index >= MOTOR_QUEUE_LEN){
    Serial.print(F("[sqi] overflow from "));
    Serial.print(index);
    Serial.print(F(" to "));
    index -= MOTOR_QUEUE_LEN;
    Serial.println(index);
  }
  queue[index].type = type;
  queue[index].value = value;
  queue[index].processed = false;
}


bool Motor::set_next_empty_queue_item(MotorQueueItemType type, uint32_t value){
  const int16_t empty = next_empty_queue_index();
  if(empty < 0) return false;
  set_queue_item(empty, type, value);
  return true;
}


void Motor::empty_queue(){
  queue_index = 0;
  memset(&queue, 0, sizeof(queue));
}



bool Motor::process_next_queue_item(bool force_ignore_wait){
  if(queue[queue_index].type == MotorQueueItemType::WAIT_IN_PROGRESS && !queue[queue_index].processed && !force_ignore_wait) return;

  SERIAL_PRINT(F("PNQ="));
  uint8_t next = next_queue_index();
  SERIAL_PRINTLN(next);
  bool process_next = false;

  if(queue[next].processed || queue[next].type == MotorQueueItemType::NOOP){
    if(running || steps_to_do > 0){
      SERIAL_PRINTLN(F("[pnq] empty queue but motor running"));

    }else{
      stop();
      // SERIAL_PRINTLN(F("[pnq] empty queue, stopping!"));

    }
    return false;
  }

  switch (queue[next].type) {
    case MotorQueueItemType::TURN_ON: {
      on();
      SERIAL_PRINTLN(F("[pnq] turn on"));
      break;
    }
    case MotorQueueItemType::TURN_OFF: {
      off();
      SERIAL_PRINTLN(F("[pnq] turn off"));
      process_next = true;
      break;
    }
    case MotorQueueItemType::STOP: {
      stop();
      SERIAL_PRINTLN(F("[pnq] stop"));
      process_next = true;
      break;
    }
    case MotorQueueItemType::RUN_CONTINUOUS: {
      // stop_on_stallguard = false;
      start(true);
      SERIAL_PRINTLN(F("[pnq] run cont"));
      process_next = true;
      break;
    }
    case MotorQueueItemType::RUN_UNTIL_STALLGUARD: {
      // stop_on_stallguard = true;
      if(queue[next].value > 0) stop_at_millis = millis() + queue[next].value;
      // ignore_stallguard_steps += 64;
      start(true);
      SERIAL_PRINTLN(F("[pnq] run until sg"));
      break;
    }
    case MotorQueueItemType::DO_STEPS: {
      steps_to_do += queue[next].value;

      SERIAL_PRINT(F("[pnq] do steps "));
      SERIAL_PRINTLN(queue[next].value);
      break;
    }
    case MotorQueueItemType::RAMP_TO: {
      // *reinterpret_cast<float*>(&queue[next].value)
      ramp_to(queue[next].value / 100.0);

      SERIAL_PRINT(F("[pnq] ramp to "));
      SERIAL_PRINTLN(queue[next].value / 100.0);
      process_next = true;
      break;
    }
    case MotorQueueItemType::SET_DIRECTION: {
      dir((bool)queue[next].value);

      SERIAL_PRINT(F("[pnq] set dir "));
      SERIAL_PRINTLN(queue[next].value);
      process_next = true;
      break;
    }
    case MotorQueueItemType::SET_RPM: {
      rpm(queue[next].value / 100.0);

      SERIAL_PRINT(F("[pnq] set rpm to "));
      SERIAL_PRINTLN(_rpm);
      process_next = true;
      break;
    }
    case MotorQueueItemType::SET_ACCEL: {
      accel = queue[next].value / 100.0;

      SERIAL_PRINT(F("[pnq] set accel to "));
      SERIAL_PRINTLN(accel);
      process_next = true;
      break;
    }
    case MotorQueueItemType::SET_DECEL: {
      decel = queue[next].value / 100.0;

      SERIAL_PRINT(F("[pnq] set decel to "));
      SERIAL_PRINTLN(decel);
      process_next = true;
      break;
    }
    case MotorQueueItemType::SET_STOP_ON_STALLGUARD: {
      stop_on_stallguard = (bool)queue[next].value;

      SERIAL_PRINT(F("[pnq] set stop-on-sg to "));
      SERIAL_PRINTLN(stop_on_stallguard ? F("true") : F("false"));
      process_next = true;
      break;
    }
    case MotorQueueItemType::SET_PRINT_STALLGUARD_TO_SERIAL: {
      print_stallguard_to_serial = (bool)queue[next].value;

      SERIAL_PRINTLN(F("[pnq] set print sg to serial"));
      process_next = true;
      break;
    }
    case MotorQueueItemType::WAIT: {
      uint32_t _millis = millis();
      uint32_t old_val = queue[next].value;
      queue[next].value += _millis;
      queue[next].type = MotorQueueItemType::WAIT_IN_PROGRESS;

      SERIAL_PRINTLN(F("[***] wait -> wait_in_progress"));
      SERIAL_PRINT(F("[pnq] wait "));
      SERIAL_PRINT(old_val);
      SERIAL_PRINT(F(" (q:"));
      SERIAL_PRINT(next);
      SERIAL_PRINT(F(") finishes at "));
      SERIAL_PRINT(queue[next].value);
      SERIAL_PRINT(" now is ");
      SERIAL_PRINT(_millis);
      SERIAL_PRINTLN();
      break;
    }
    case MotorQueueItemType::BEEP: {
      beep(queue[next].value);

      SERIAL_PRINTLN(F("[pnq] beep!"));
      process_next = true;
      break;
    }
    case MotorQueueItemType::SET_IS_HOMED: {
      is_homed = (bool)queue[next].value;

      SERIAL_PRINT(F("[pnq] set is_homed to "));
      SERIAL_PRINTLN(is_homed ? F("true") : F("false"));
      process_next = true;
      break;
    }
    case MotorQueueItemType::SET_POSITION: {
      // position = *reinterpret_cast<float*>(&queue[next].value);
      position_usteps = rot2usteps(*reinterpret_cast<float*>(&queue[next].value));
      // planned.position = position;

      SERIAL_PRINT(F("[pnq] set position to "));
      SERIAL_PRINTLN(position());
      process_next = true;
      break;
    }
    case MotorQueueItemType::RESET_STALLGUARD_TRIGGERED: {
      stallguard_triggered = false;

      SERIAL_PRINTLN(F("[pnq] reset sg triggered"));
      process_next = true;
      break;
    }
    case MotorQueueItemType::REPEAT_QUEUE: {
      if(queue[next].value > 0){
        queue[next].value--;
        for (size_t j = 0; j < queue_index; j++) {
          queue[j].processed = false;
        }
        queue_index = 0;
        SERIAL_PRINT(F("[pnq] repeat queue: "));
        SERIAL_PRINTLN(queue[next].value);
        return true;
      }

      process_next = true;
      break;
    }
    case MotorQueueItemType::ADD_IGNORE_STALLGUARD_STEPS: {
      ignore_stallguard_steps += queue[next].value;

      SERIAL_PRINT(F("[pnq] add ignore sg steps "));
      SERIAL_PRINTLN(queue[next].value);
      process_next = true;
      break;
    }
    case MotorQueueItemType::SET_IS_HOMING: {
      is_homing = (bool)queue[next].value;

      SERIAL_PRINT(F("[pnq] set is_homing "));
      SERIAL_PRINTLN(queue[next].value ? "true" : "false");
      process_next = true;
      break;
    }

  }
  // memset(&queue[queue_index], 0, sizeof(queue[queue_index]));
  if(next < queue_index){
    Serial.print(F("[pnq] rollover from "));
    Serial.print(queue_index);
    Serial.print(F(" to "));
    Serial.println(next);
  }

  queue[queue_index].processed = true;
  queue_index = next;

  // debugPrintQueue();
  if(queue[queue_index].type == MotorQueueItemType::WAIT){
    SERIAL_PRINTLN(F("[[pnq]] is wait:"));
    // debugPrintQueue();
  }
  if(queue[queue_index].type == MotorQueueItemType::WAIT_IN_PROGRESS){
    SERIAL_PRINTLN(F("[[pnq]] is wait_in_progress:"));
    // debugPrintQueue();
  }
  if(process_next){
    SERIAL_PRINTLN(F("[pqn finished] process_next!"));
    process_next_queue_item();
  }
  return true;

}


void Motor::debugPrintQueue(){
  const uint8_t next = next_queue_index();
  for (size_t i = 0; i < MOTOR_QUEUE_LEN; i++) {
    if(queue_index == i) Serial.print(F("-> "));
    else if(next == i) Serial.print(F("n> "));
    Serial.print(i);
    Serial.print(F(":\tT:"));
    Serial.print(queue[i].type);

    Serial.print(queue[i].processed ? F("\t[x]") : F("\t[ ]"));

    Serial.print(F("\tv:"));
    Serial.print(queue[i].value);
    Serial.println();
    if(queue[i].type==0 && !queue[i].processed && queue[i].value==0) break;
  }
}


void Motor::debugPrintInfo(){
  Serial.print(F("Motor: ")); Serial.println(axis);
  PRINT_VAR(pause_steps);
  PRINT_VAR(usteps);
  PRINT_VAR(enabled);
  PRINT_VAR(dir());
  PRINT_VAR(stop_on_stallguard);
  PRINT_VAR(running);
  PRINT_VAR(position_usteps);
  PRINT_VAR(position());
  PRINT_VAR(is_homed);
  PRINT_VAR(reset_is_homed_on_power_off);
  PRINT_VAR(reset_is_homed_on_stall);
  PRINT_VAR(steps_to_do);
  PRINT_VAR(steps_total);
  PRINT_VAR(target_rpm);
  PRINT_VAR(accel);
  PRINT_VAR(decel);
  PRINT_VAR(_rpm);
  PRINT_VAR(_dir);
  PRINT_VAR(queue_index);
  PRINT_VAR(stallguard_triggered);
  PRINT_VAR(inactivity_timeout);
  PRINT_VAR(stop_at_millis);
  PRINT_VAR(last_speed_change);
  PRINT_VAR(last_movement);
  PRINT_VAR(planned.rpm);
  PRINT_VAR(planned.direction);
  PRINT_VAR(planned.is_homed);
  PRINT_VAR(planned.position);
  PRINT_VAR(planned.accel);
  PRINT_VAR(planned.decel);
  Serial.print(F("__min_rpm: ")); Serial.println(_ocr2rpm(65535, usteps));
  Serial.print(F("__max_rpm: ")); Serial.println(_ocr2rpm(70, usteps));
}


void Motor::plan_steps(uint32_t){
}


void Motor::plan_rotations(float rotations, float rpm){
  const bool rot_direction = rotations > 0.0;
  uint8_t next = next_empty_queue_index();
  if(rot_direction != planned.direction){
    set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, rot_direction);
    planned.direction = rot_direction;
  }
  if(rpm > 0.0 && rpm != planned.rpm){
    set_queue_item(next++, MotorQueueItemType::SET_RPM, rpm * 100);
    planned.rpm = rpm;
  }
  set_queue_item(next++, MotorQueueItemType::DO_STEPS, rot2usteps(rotations));
  planned.position += rotations;
}


void Motor::plan_rotations_to(float rotations, float rpm){
  const float rot_delta = rotations - planned.position;
  if(rot_delta != 0.0){
    if(planned.is_homed){
      plan_rotations(rot_delta, rpm);

    }else{
      if(autohome.enabled){
        plan_autohome();
        plan_rotations(rot_delta, rpm);
        Serial.print(F("axis "));
        Serial.print(axis);
        Serial.println(F(" is autohoming first."));

      }else{
        Serial.print(F("axis "));
        Serial.print(axis);
        Serial.println(F(" must home first!"));

      }
    }
  }
}


void Motor::plan_home(bool direction, float initial_rpm, float final_rpm, float backstep_rot, float ramp_from, uint16_t wait_duration){
  const uint32_t backstep_usteps = rot2usteps(backstep_rot);
  const uint32_t ignore_stallguard_steps = backstep_usteps;
  uint8_t next = next_empty_queue_index();

  set_queue_item(next++, MotorQueueItemType::SET_IS_HOMING, 1);

  if(backstep_rot > 0.0){
    // backstep
    if(initial_rpm != planned.rpm){
      set_queue_item(next++, MotorQueueItemType::SET_RPM, initial_rpm * 100);
      planned.rpm = initial_rpm;
    }
    if(!direction != planned.direction){
      set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, !direction);
      planned.direction = !direction;
    }
    // set_queue_item(next++, MotorQueueItemType::ADD_IGNORE_STALLGUARD_STEPS, ignore_stallguard_steps);
    set_queue_item(next++, MotorQueueItemType::DO_STEPS, backstep_usteps);
    set_queue_item(next++, MotorQueueItemType::WAIT, wait_duration);
  }

  // fast forward until stallguard
  if(direction != planned.direction){
    set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, direction);
    planned.direction = direction;
  }
  if(ramp_from > 0.0){
    if(ramp_from != planned.rpm){
      set_queue_item(next++, MotorQueueItemType::SET_RPM, ramp_from * 100);
      planned.rpm = ramp_from;
    }
    if(initial_rpm != planned.rpm){
      set_queue_item(next++, MotorQueueItemType::RAMP_TO, initial_rpm * 100);
      planned.rpm = initial_rpm;
    }
  }
  set_queue_item(next++, MotorQueueItemType::RUN_UNTIL_STALLGUARD);
  set_queue_item(next++, MotorQueueItemType::RESET_STALLGUARD_TRIGGERED);

  if(final_rpm > 0.0){
    if(backstep_rot > 0.0){
      // backstep again
      if(final_rpm != planned.rpm){
        set_queue_item(next++, MotorQueueItemType::RAMP_TO, final_rpm * 100);
        planned.rpm = final_rpm;
      }
      if(!direction != planned.direction){
        set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, !direction);
        planned.direction = !direction;
      }
      set_queue_item(next++, MotorQueueItemType::ADD_IGNORE_STALLGUARD_STEPS, ignore_stallguard_steps);
      set_queue_item(next++, MotorQueueItemType::DO_STEPS, backstep_usteps);
      set_queue_item(next++, MotorQueueItemType::WAIT, wait_duration);
    }

    // slow forward until stallguard
    if(final_rpm != planned.rpm){
      set_queue_item(next++, MotorQueueItemType::SET_RPM, final_rpm * 100);
      planned.rpm = final_rpm;
    }
    if(direction != planned.direction){
      set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, direction);
      planned.direction = direction;
    }
    set_queue_item(next++, MotorQueueItemType::RUN_UNTIL_STALLGUARD);
    set_queue_item(next++, MotorQueueItemType::RESET_STALLGUARD_TRIGGERED);
  }

  set_queue_item(next++, MotorQueueItemType::SET_IS_HOMED, 1);
  set_queue_item(next++, MotorQueueItemType::SET_POSITION, 0);
  set_queue_item(next++, MotorQueueItemType::SET_IS_HOMING, 0);

  planned.is_homed = true;
  planned.position = 0.0;

}


void Motor::plan_autohome(){
  plan_home(autohome.direction, autohome.initial_rpm, autohome.final_rpm, autohome.backstep_rot, autohome.ramp_from, autohome.wait_duration);
}


void Motor::plan_ramp_move(float rotations, float rpm_from, float rpm_to, float accel, float decel){
  if(accel == 0.0) accel = planned.accel;
  if(decel == 0.0) decel = accel;
  const bool rot_direction = rotations > 0.0;
  const float delta_rpm = rpm_to - rpm_from;
  const uint32_t sps_from = rpm2sps(rpm_from);
  const uint32_t sps_to = rpm2sps(rpm_to);
  const uint32_t delta_sps = sps_to - sps_from;
  const uint32_t accel_sps = rpm2sps(accel);
  const uint32_t decel_sps = rpm2sps(decel);
  const float accel_t = delta_rpm / accel;
  const float decel_t = delta_rpm / decel;
  const uint32_t steps = rot2usteps(rotations);
  uint32_t decel_steps = (unsigned long)round(((float)delta_sps * delta_sps) / (2.0 * decel_sps));

  // Serial.println(F("[[motor]] plan_ramp_move:"));
  // PRINT_VAR(rotations);
  // PRINT_VAR(rpm_from);
  // PRINT_VAR(rpm_to);
  // PRINT_VAR(accel);
  // PRINT_VAR(decel);
  // Serial.println(F("------"));
  // PRINT_VAR(rot_direction);
  // PRINT_VAR(delta_rpm);
  // PRINT_VAR(sps_from);
  // PRINT_VAR(sps_to);
  // PRINT_VAR(delta_sps);
  // PRINT_VAR(accel_sps);
  // PRINT_VAR(decel_sps);
  // PRINT_VAR(accel_t);
  // PRINT_VAR(decel_t);
  // PRINT_VAR(steps);
  // PRINT_VAR(decel_steps);

  if(steps <= (decel_steps * 2UL)) decel_steps = steps / 2;
  // PRINT_VAR(decel_steps);

  uint8_t next = next_empty_queue_index();
  if(rot_direction != planned.direction){
    set_queue_item(next++, MotorQueueItemType::SET_DIRECTION, rot_direction);
    planned.direction = rot_direction;
  }
  if(accel != planned.accel){
    set_queue_item(next++, MotorQueueItemType::SET_ACCEL, accel * 100);
    planned.accel = accel;
  }
  if(decel != planned.decel){
    set_queue_item(next++, MotorQueueItemType::SET_DECEL, decel * 100);
    planned.decel = decel;
  }
  if(rpm_from != planned.rpm){
    set_queue_item(next++, MotorQueueItemType::SET_RPM, rpm_from * 100);
    // planned.rpm = rpm_from;
  }
  set_queue_item(next++, MotorQueueItemType::RAMP_TO, rpm_to * 100);

  set_queue_item(next++, MotorQueueItemType::DO_STEPS, steps - decel_steps);
  set_queue_item(next++, MotorQueueItemType::RAMP_TO, rpm_from * 100);
  set_queue_item(next++, MotorQueueItemType::DO_STEPS, decel_steps);

  planned.position += rotations;
  planned.rpm = rpm_from;

}


void Motor::plan_ramp_move_to(float rotations, float rpm_from, float rpm_to, float accel, float decel){
  const float rot_delta = rotations - planned.position;
  if(rot_delta != 0.0){
    if(planned.is_homed){
      plan_ramp_move(rot_delta, rpm_from, rpm_to, accel, decel);

    }else{
      if(autohome.enabled){
        plan_autohome();
        plan_ramp_move(rot_delta, rpm_from, rpm_to, accel, decel);
        Serial.print(F("axis "));
        Serial.print(axis);
        Serial.println(F(" is autohoming first."));

      }else{
        Serial.print(F("axis "));
        Serial.print(axis);
        Serial.println(F(" must home first!"));

      }
    }
  }
}


// stepping timer
#define TIMER_ISR(t, m) ISR(TIMER##t##_COMPA_vect){  \
  if(motors[m].pause_steps) return;  \
  if(motors[m].running){  \
    motors[m].step();  \
  }else if(motors[m].steps_to_do){  \
    motors[m].steps_to_do--;  \
    motors[m].step();  \
  }else{  \
    motors[m].pause_steps = true;  \
    /*SERIAL_PRINTLN("[isr] pnq!");*/  \
    motors[m].process_next_queue_item();  \
    motors[m].pause_steps = false;  \
  }  \
}
TIMER_ISR(1, 0)
TIMER_ISR(3, 1)
TIMER_ISR(4, 2)
TIMER_ISR(5, 3)
#undef TIMER_ISR


// "wait" handling timer, beeper off timer
ISR(TIMER0_COMPA_vect){
  const uint32_t _millis = millis();
  readEncoder();

  for(size_t i = 0; i < MOTORS_MAX; i++){
    if(motors[i].inactivity_timeout > 0 && motors[i].last_movement > 0 &&
      _millis > motors[i].last_movement + motors[i].inactivity_timeout
    ){
      motors[i].last_movement = 0;
      motors[i].stop_at_millis = 0;
      motors[i].stop();
      motors[i].off();
      Serial.print(F("inactivity timeout, motor "));
      Serial.print(motors[i].axis);
      Serial.println(F(" off"));
    }

    if(motors[i].pause_steps) continue;
    uint8_t next = motors[i].next_queue_index();

    if(motors[i].queue[next].type != MotorQueueItemType::NOOP &&
      !motors[i].queue[next].processed &&
      !motors[i].queue[motors[i].queue_index].processed &&
      motors[i].queue[motors[i].queue_index].type != MotorQueueItemType::DO_STEPS &&
      motors[i].queue[motors[i].queue_index].type != MotorQueueItemType::WAIT_IN_PROGRESS &&
      motors[i].queue[motors[i].queue_index].type != MotorQueueItemType::RUN_UNTIL_STALLGUARD
    ){
      // SERIAL_PRINTLN(F("[wait] move queue! PNQ!"));
      // motors[i].debugPrintQueue();
      motors[i].pause_steps = true;
      motors[i].process_next_queue_item();
      motors[i].pause_steps = false;
    }

    // handle WAIT_IN_PROGRESS
    if(motors[i].queue[motors[i].queue_index].type == MotorQueueItemType::WAIT_IN_PROGRESS &&
      !motors[i].queue[motors[i].queue_index].processed &&
      _millis >= motors[i].queue[motors[i].queue_index].value
    ){
      motors[i].queue[motors[i].queue_index].processed = true;
      SERIAL_PRINT(F(" wait("));
      SERIAL_PRINT(motors[i].queue_index);
      SERIAL_PRINTLN(F(") finished! pnq!"));
      motors[i].pause_steps = true;
      if(!motors[i].process_next_queue_item(true)){
        // motors[i].queue[motors[i].queue_index].processed = true;
        // SERIAL_PRINTLN(F("[wait] queue is empty, marking current as processed!"));
        // SERIAL_PRINTLN(F("[wait] queue is empty, pnq returned false!"));
      }
      motors[i].pause_steps = false;
    }

    if(motors[i].stop_at_millis > 0 && motors[i].stop_at_millis < _millis) {
      motors[i].stop_at_millis = 0;
      motors[i].stop();
      Serial.print(F("[sam] stopped "));
      Serial.println(motors[i].axis);
    }

  }

  if(beeper_off_at && _millis > beeper_off_at){
    beeper_off_at = 0;
    digitalWriteExt(BEEPER, LOW);
  }

  #ifdef CUSTOM_TIMER0_ISR
    timer0Custom();
  #endif
}


// acceleration handling
ISR(TIMER2_COMPA_vect){
  static uint32_t last_tick = 0;
  uint32_t _millis = millis();

  for(size_t i = 0; i < MOTORS_MAX; i++){
    if(!motors[i].pause_steps && (motors[i].running || motors[i].steps_to_do > 0)){
      motors[i].last_movement = _millis;
      if(motors[i].target_rpm >= 0.0){
        float rpm = motors[i].rpm();
        float rpm_delta = motors[i].target_rpm - rpm;
        uint16_t delta_t = _millis - motors[i].last_speed_change;
        float change_fraction;

        if(rpm_delta > 0.0){
          // accelerating
          change_fraction = motors[i].accel / 1000.0 * delta_t;
          rpm += change_fraction;
          if(rpm >= motors[i].target_rpm){
            rpm = motors[i].target_rpm;
            motors[i].target_rpm = -1.0;  // stop ramping since we reached set
          }

        }else{
          // decelerating
          change_fraction = motors[i].decel / 1000.0 * delta_t;
          rpm -= change_fraction;
          if(rpm <= motors[i].target_rpm){
            rpm = motors[i].target_rpm;
            motors[i].target_rpm = -1.0;  // stop ramping since we reached set
          }

        }

        motors[i]._rpm = rpm;
        uint32_t ocr = motors[i].rpm2ocr(rpm);
        if(ocr < 70) ocr = 70;
        if(ocr > 65535) ocr = 65535;

        if(rpm <= 0.0) motors[i].stop();

        cli();
        *motors[i].timer_counter_port = 0;
        *motors[i].timer_compare_port = ocr;
        sei();

        motors[i].last_speed_change = _millis;
      }
    }
  }

  last_tick = _millis;
}


// stallguard pin change interrupt
ISR(PCINT2_vect){
  const bool sg[MOTORS_MAX] = {
    PINK & (1 << PINK2), // X_DIAG
    PINK & (1 << PINK7), // Y_DIAG
    PINK & (1 << PINK6), // Z_DIAG
    PINK & (1 << PINK3), // E0_DIAG
  };
  if(!sg[0] && !sg[1] && !sg[2] && !sg[3]) return;

  SERIAL_PRINT("SG Int ");
  SERIAL_PRINT(sg[0]);
  SERIAL_PRINT(sg[1]);
  SERIAL_PRINT(sg[2]);
  SERIAL_PRINTLN(sg[3]);

  // Serial.print("SG Int ");
  // Serial.print(sg[0]);
  // Serial.print(sg[1]);
  // Serial.print(sg[2]);
  // Serial.println(sg[3]);

  for(size_t i = 0; i < MOTORS_MAX; i++){
    if(sg[i]){
      // beep(30);
      if(motors[i].ignore_stallguard_steps > 0){
        // Serial.print(F("[sg] ignore sg steps "));
        // Serial.println(motors[i].ignore_stallguard_steps);
        continue;
      }

      motors[i].stallguard_triggered = true;
      if(motors[i].is_expecting_stallguard()){
        SERIAL_PRINTLN("[sg] is expected");

        motors[i].running = false;
        motors[i].pause_steps = true;
        motors[i].steps_to_do = 0;
        SERIAL_PRINT("[sg] ");
        motors[i].process_next_queue_item();
        SERIAL_PRINT("s2d=");
        SERIAL_PRINTLN(motors[i].steps_to_do);
        motors[i].pause_steps = false;

      }else{
        SERIAL_PRINTLN("[sg] unexpected");

        // motors[i].stallguard_triggered = true;
        // if(motors[i].stop_on_stallguard) motors[i].stop();
        if(motors[i].stop_on_stallguard){
          motors[i].stop();
          SERIAL_PRINTLN("[sg] stop motor");
        }
        if(motors[i].reset_is_homed_on_stall){
          motors[i].is_homed = false;
          motors[i].planned.is_homed = false;
        }

      }

      SERIAL_PRINT("[sg] is_triggered=");
      SERIAL_PRINTLN((uint8_t)motors[i].stallguard_triggered);
    }
  }

}
