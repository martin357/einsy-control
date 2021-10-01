#include "motor.h"
#include "pins.h"


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
  sei();

  #undef SETUP_TIMER

}


float rpm2rps(float rpm){
  return rpm / 60;
}


uint16_t rps2sps(float rps, uint16_t usteps){
  return uint16_t(FSTEPS_PER_REVOLUTION * usteps * rps);
}


uint16_t sps2ocr(uint16_t sps){
  return F_CPU / MOTORS_PRESCALER / sps;
}


uint16_t rpm2ocr(float rpm, uint16_t usteps){
  return rps2ocr(rpm / 60, usteps);
}


uint16_t rps2ocr(float rps, uint16_t usteps){
  return F_CPU / MOTORS_PRESCALER / (FSTEPS_PER_REVOLUTION * usteps * rps);
}


float ocr2rpm(uint16_t ocr, uint16_t usteps){
  return ocr2rps(ocr, usteps) * 60;
}


float ocr2rps(uint16_t ocr, uint16_t usteps){
  return F_CPU / MOTORS_PRESCALER / ocr / usteps / FSTEPS_PER_REVOLUTION;
}



Motor::Motor(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, uint8_t cs_pin, uint8_t diag_pin,
    uint8_t* step_port, uint8_t step_bit, uint16_t* timer_compare_port, uint16_t* timer_counter_port,
    uint8_t* timer_enable_port, uint8_t timer_enable_bit):
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
  stop_on_stallguard(true),
  print_stallguard_to_serial(false),
  running(false),
  stallguard_triggered(false),
  steps_to_do(0),
  steps_total(0),

  target_rpm(-1.0),
  // accel(400),
  // decel(800),
  accel(40),
  decel(120),
  // accel(450),
  // decel(1000),
  // ramp_interval(30),

  last_speed_change(last_speed_change),

  _rpm(0.0),
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
  if(millis() > 300) SERIAL_PRINTLN("on");
}


void Motor::off(){
  digitalWriteExt(enable_pin, HIGH);
  stop();
  if(millis() > 300) SERIAL_PRINTLN("off");
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
  if(millis() > 300) SERIAL_PRINTLN("start");
}


void Motor::stop(){
  *timer_enable_port = 0;
  enabled = false;
  running = false;
  steps_to_do = 0;
  ramp_off();
  if(millis() > 300) SERIAL_PRINTLN("stop");
}


void Motor::step(){
  *step_port ^= 1 << step_bit;
  delayMicroseconds(2);
  *step_port ^= 1 << step_bit;
  steps_total++;
}


bool Motor::dir(){
  return digitalReadExt(dir_pin);
}


void Motor::dir(bool direction){
  digitalWriteExt(dir_pin, direction);
}


uint16_t Motor::sg_value(){
  TMC2130_n::DRV_STATUS_t drv_status{0};
  drv_status.sr = driver.DRV_STATUS();
  return drv_status.sg_result;
}


void Motor::microsteps(uint16_t microstepping){
  driver.microsteps(microstepping);
  usteps = driver.microsteps();
  // rpm(_rpm); // update timer

  if(millis() > 300){
    SERIAL_PRINT("new usteps: ");
    SERIAL_PRINTLN(usteps);
  }
}


void Motor::rpm(float value){
  _rpm = value;
  uint32_t ocr = rpm2ocr(value, usteps);
  if(ocr < 70) ocr = 70;
  if(ocr > 65535) ocr = 65535;

  if(value <= 0.0){
    stop();
    if(millis() > 300) SERIAL_PRINTLN("low rpm - stop!");
  }

  if(*timer_compare_port != ocr ){
    cli();
    // *timer_counter_port = 0;
    *timer_compare_port = ocr;
    sei();

    static uint32_t last_millis = 0;
    uint32_t _millis = millis();
    if(_millis > 300){
    // if(1){
      SERIAL_PRINT("new ocr=");
      SERIAL_PRINT(ocr);
      last_millis = _millis;
      // SERIAL_PRINT("\tnew rpm=");
      // SERIAL_PRINT(new_rpm);
      SERIAL_PRINTLN();
    }
  }
}


float Motor::rpm(){
  return _rpm;
}


void Motor::ramp_to(float value, bool keep_running){
  last_speed_change = millis();
  target_rpm = value;
  // target_rpm = ocr2rpm(rpm2ocr(value, usteps), usteps);
  if(!running){
    rpm(rpm()); // update timer
    start(keep_running);
  }
}


void Motor::ramp_off(){
  target_rpm = -1.0;
  last_speed_change = millis();
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


uint8_t Motor::next_queue_index(){
  return queue_index + 1 >= MOTOR_QUEUE_LEN ? 0 : queue_index + 1;
}


void Motor::set_queue_item(uint8_t index, MotorQueueItemType type, uint32_t steps = 0, uint16_t rpm = 0){
  queue[index].type = type;
  queue[index].steps = steps;
  queue[index].rpm = rpm;
}


bool Motor::process_next_queue_item(){
  Serial.print("PNQ=");
  uint8_t next = next_queue_index();
  Serial.println(next);

  if(queue[next].processed || queue[next].type == MotorQueueItemType::NOOP){
    Serial.println(" empty, stopping");
    stop();
    return false;
  }

  switch (queue[next].type) {
    case MotorQueueItemType::RUN_CONTINUOUS: {
      stop_on_stallguard = false;
      start(true);
      Serial.println(" run cont");
      break;
    }
    case MotorQueueItemType::RUN_UNTIL_STALLGUARD: {
      stop_on_stallguard = true;
      start(true);
      Serial.println(" run until sg");
      break;
    }
    case MotorQueueItemType::DO_STEPS: {
      if(queue[next].rpm > 0) rpm(queue[next].rpm);
      steps_to_do += queue[next].steps;

      Serial.println(" do steps");
      if(queue[next].rpm > 0){
        Serial.print(" new rpm ");
        Serial.println(queue[next].rpm);
      }
      break;
    }
    case MotorQueueItemType::DO_STEPS_WITH_RAMP: {
      Serial.println(" do steps w/ ramp");
      if(queue[next].rpm > 0) ramp_to(queue[next].rpm);
      steps_to_do += queue[next].steps;
      break;
    }

  }
  // memset(&queue[queue_index], 0, sizeof(queue[queue_index]));
  queue[queue_index].processed = true;
  queue_index = next;
  debugPrintQueue();
  return true;

}


void Motor::debugPrintQueue(){
  for (size_t i = 0; i < MOTOR_QUEUE_LEN; i++) {
    if(queue_index == i) Serial.print("-> ");
    Serial.print(i);
    Serial.print(":\tT:");
    Serial.print(queue[i].type);
    Serial.print("\tsteps:");
    Serial.print(queue[i].steps);
    Serial.print("\trpm:");
    Serial.print(queue[i].rpm);
    Serial.println();
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
    Serial.println("[e]");  \
    motors[m].process_next_queue_item();  \
    Serial.print("s2d=");  \
    Serial.println(motors[m].steps_to_do);  \
    motors[m].pause_steps = false;  \
  /*}else{  \
    cli();  \
    Serial.println("[e]");  \
    motors[m].process_next_queue_item();  \
    Serial.print("s2d=");  \
    Serial.println(motors[m].steps_to_do);  \
    Serial.flush();  \
    sei();*/  \
  }  \
}
TIMER_ISR(1, 0)
TIMER_ISR(3, 1)
TIMER_ISR(4, 2)
TIMER_ISR(5, 3)
#undef TIMER_ISR


// acceleration handling
ISR(TIMER2_COMPA_vect){
  static uint8_t cnt = 0;
  static uint32_t last_tick = 0;
  // 6 = 111.11ms
  // 5 = 100.00ms
  // 4 = 83.33ms
  // 3 = 66.67ms
  // 2 = 50.00ms
  // 1 = 33.33ms
  // 0 = 16.39ms
  if(cnt++ >= 1){
    uint32_t _millis = millis();
    // Serial.print("acc:");
    // uint32_t delta_t = _millis - last_tick;

    // Serial.print("tD:");
    // Serial.println(delta_t);

    cli();
    for(size_t i = 0; i < MOTORS_MAX; i++){
      // Serial.print(i);
      if((motors[i].target_rpm >= 0.0) && (motors[i].running || motors[i].steps_to_do)){
        SERIAL_PRINT("M");
        SERIAL_PRINT(i);
        SERIAL_PRINT(": ");
        // SERIAL_PRINT(bool(motors[i].target_rpm >= 0.0) ? '1' : '0');
        // SERIAL_PRINT(bool(motors[i].running || motors[i].steps_to_do) ? '1' : '0');
        // SERIAL_PRINT(bool(_millis >= motors[i].last_speed_change + motors[i].ramp_interval) ? '1' : '0');
        // SERIAL_PRINT(" ");
        SERIAL_PRINT(motors[i].target_rpm);
        SERIAL_PRINT("\t");

        float rpm = motors[i].rpm();
        float rpm_delta = motors[i].target_rpm - rpm;
        // if(abs(rpm_delta) > 1.0){
          // SERIAL_PRINTLN("ramp");
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

          SERIAL_PRINT("rpmD ");
          SERIAL_PRINT(rpm_delta);
          SERIAL_PRINT("\ttD ");
          SERIAL_PRINT(delta_t);
          SERIAL_PRINT("\tchF ");
          SERIAL_PRINT(change_fraction);
          SERIAL_PRINT("\trpmN ");
          SERIAL_PRINT(rpm);
          SERIAL_PRINTLN();

          motors[i].rpm(rpm);

          // }

          motors[i].last_speed_change = _millis;
        }

    }
    sei();
    // Serial.print(" ");
    // Serial.print(_millis - last_tick);
    // Serial.println();


    last_tick = _millis;
    cnt = 0;
  }
}


// stallguard pin change interrupt
ISR(PCINT2_vect){
  const bool sg[MOTORS_MAX] = {
    PINK & (1 << PINK2), // X_DIAG
    PINK & (1 << PINK7), // Y_DIAG
    PINK & (1 << PINK6), // Z_DIAG
    PINK & (1 << PINK3), // E0_DIAG
  };

  for(size_t i = 0; i < MOTORS_MAX; i++){
    if(sg[i]){
      motors[i].stallguard_triggered = true;
      if(motors[i].stop_on_stallguard) motors[i].stop();
    }
  }

}
