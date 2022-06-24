#include "serial.h"
#include "hardware.h"
#include "gcode.h"


uint8_t rx_buf_pos = 0;
char rx_buf[RX_BUF_LEN] = {0};
uint8_t rx_delimiter_pos[RX_PARAMS];
// extern
char rx_param[RX_PARAMS][RX_PARAM_LEN];
uint8_t rx_params;
char rx_command[RX_COMMAND_LEN];
bool print_gcode_to_lcd = false;
uint8_t print_gcode_to_lcd_last_row = 0;



void processCommand(const __FlashStringHelper *cmd){
  PGM_P ptr = reinterpret_cast<PGM_P>(cmd);
  char buf[RX_BUF_LEN];
  size_t len = 0;
  while(1){
    if((buf[len++] = pgm_read_byte(ptr++)) == 0) break;
  }
  processCommand(buf, len - 1);
}



void processCommand(const char *cmd, size_t len){
  if(len > 0 && cmd[0] == '#'){ // it is a comment
    SERIAL_PRINT("----- ");
    SERIAL_PRINT(cmd);
    SERIAL_PRINTLN(" -----");
    Serial1.println(cmd);
    return;
  }

  #ifdef DEBUG_PRINT
    SERIAL_PRINT("CMD: '");
    SERIAL_PRINT(cmd);
    SERIAL_PRINTLN("'");
  #endif

  bool has_hash = false;
  uint8_t given_hash;
  uint8_t hash = 0;
  if(len > 1 && cmd[0] == '@'){
    given_hash = cmd[1];
    has_hash = true;
    cmd += 2;
    len -= 2;
  }

  Serial1.print("cmd> '");
  Serial1.print(cmd);
  Serial1.println("'");

  bool ok = true;
  rx_params = 0;
  memset(rx_command, 0, RX_COMMAND_LEN);
  memset(rx_param, 0, RX_PARAMS * RX_PARAM_LEN);
  for(size_t i = 0; i < len; i++){
    if(has_hash) hash = ((hash << 5) + hash) ^ cmd[i];
    if(cmd[i] == ' '){
      rx_delimiter_pos[rx_params] = i;
      if(rx_params < RX_PARAMS){
        rx_params++;
      }else{
        Serial.println(F("max cmd params reached!"));
      }
    }
  }

  if(has_hash){
    if(hash == 0) hash++;
    if(given_hash != hash){
      Serial.print(F("invalid hash '"));
      Serial.print(cmd);
      Serial.println("'");

      Serial1.print(F("invalid hash '"));
      Serial1.print(cmd);
      Serial1.println("'");

      #ifdef DEBUG_PRINT
        SERIAL_PRINT(F("!!!!!!!! >> INVALID HASH '"));
        SERIAL_PRINT(cmd);
        SERIAL_PRINTLN("'");
      #endif
      return;
    }
  }

  if(rx_params > 0){
    memcpy(rx_command, cmd, rx_delimiter_pos[0]);
    for(size_t i = 0; i < rx_params; i++){
      if(i == rx_params - 1){ // last
        memcpy(rx_param[i], cmd + rx_delimiter_pos[i] + 1, len - rx_delimiter_pos[i] - 1);
      }else{
        memcpy(rx_param[i], cmd + rx_delimiter_pos[i] + 1, rx_delimiter_pos[i + 1] - rx_delimiter_pos[i] - 1);
      }
    }

  }else{
    memcpy(rx_command, cmd, len);
  }

  strToLower(rx_command);
  // Serial.print(">>> ");
  // Serial.println(cmd);
  if(strcmp_P(rx_command, F("on"))) gcode_on();
  else if(strcmp_P(rx_command, F("off"))) gcode_off();
  else if(strcmp_P(rx_command, F("start"))) gcode_start();
  else if(strcmp_P(rx_command, F("stop"))) gcode_stop();
  else if(strcmp_P(rx_command, F("halt"))) gcode_halt();
  else if(strcmp_P(rx_command, F("move_usteps"))) gcode_move_usteps();
  else if(strcmp_P(rx_command, F("move"))) gcode_move();
  else if(strcmp_P(rx_command, F("run"))) gcode_run();
  else if(strcmp_P(rx_command, F("rpm"))) gcode_rpm();
  else if(strcmp_P(rx_command, F("dir"))) gcode_dir();
  else if(strcmp_P(rx_command, F("accel"))) gcode_accel();
  else if(strcmp_P(rx_command, F("decel"))) gcode_decel();
  else if(strcmp_P(rx_command, F("ramp_to_nq"))) gcode_ramp_to_nq();
  else if(strcmp_P(rx_command, F("ramp")) || strcmp_P(rx_command, F("ramp_to"))) gcode_ramp_to();
  else if(strcmp_P(rx_command, F("do_steps_dir"))) gcode_do_steps_dir();
  else if(strcmp_P(rx_command, F("do_steps_to"))) gcode_do_steps_to();
  else if(strcmp_P(rx_command, F("do_steps"))) gcode_do_steps();
  else if(strcmp_P(rx_command, F("move_rot"))) gcode_move_rot();
  else if(strcmp_P(rx_command, F("move_rot_to"))) gcode_move_rot_to();
  else if(strcmp_P(rx_command, F("move_ramp"))) gcode_move_ramp();
  else if(strcmp_P(rx_command, F("move_ramp_to"))) gcode_move_ramp_to();
  else if(strcmp_P(rx_command, F("home"))) gcode_home();
  else if(strcmp_P(rx_command, F("autohome"))) gcode_autohome();
  else if(strcmp_P(rx_command, F("print_queue"))) gcode_print_queue();
  else if(strcmp_P(rx_command, F("empty_queue"))) gcode_empty_queue();
  else if(strcmp_P(rx_command, F("print_info"))) gcode_print_info();
  else if(strcmp_P(rx_command, F("pos_usteps"))) gcode_pos_usteps();
  else if(strcmp_P(rx_command, F("pos"))) gcode_pos();
  else if(strcmp_P(rx_command, F("stop_on_stallguard"))) gcode_stop_on_stallguard();
  else if(strcmp_P(rx_command, F("print_stallguard"))) gcode_print_stallguard();
  else if(strcmp_P(rx_command, F("wait_for_motor"))) gcode_wait_for_motor();
  else if(strcmp_P(rx_command, F("wait"))) gcode_wait();
  else if(strcmp_P(rx_command, F("beep"))) gcode_beep();
  else if(strcmp_P(rx_command, F("repeat_queue"))) gcode_repeat_queue();
  else if(strcmp_P(rx_command, F("set_position_usteps"))) gcode_set_position_usteps();
  else if(strcmp_P(rx_command, F("set_position"))) gcode_set_position();
  else if(strcmp_P(rx_command, F("set_invert_direction"))) gcode_set_invert_direction();
  else if(strcmp_P(rx_command, F("reset_steps_total"))) gcode_reset_steps_total();
  else if(strcmp_P(rx_command, F("sync_position"))) gcode_sync_position();
  else if(strcmp_P(rx_command, F("stallguard_threshold"))) gcode_stallguard_threshold();
  else if(strcmp_P(rx_command, F("current_hold"))) gcode_current_hold();
  else if(strcmp_P(rx_command, F("current"))) gcode_current();
  else if(strcmp_P(rx_command, F("microstepping"))) gcode_microstepping();
  else if(strcmp_P(rx_command, F("set_is_homed"))) gcode_set_is_homed();
  else if(strcmp_P(rx_command, F("set_is_homing"))) gcode_set_is_homing();
  else if(strcmp_P(rx_command, F("is_busy"))) gcode_is_busy();
  else if(strcmp_P(rx_command, F("is_homed"))) gcode_is_homed();
  else if(strcmp_P(rx_command, F("is_homing"))) gcode_is_homing();
  else if(strcmp_P(rx_command, F("set_default_ramp_rpm_from"))) gcode_set_default_ramp_rpm_from();
  else if(strcmp_P(rx_command, F("set_default_ramp_rpm_to"))) gcode_set_default_ramp_rpm_to();
  else if(strcmp_P(rx_command, F("set_hold_multiplier"))) gcode_set_hold_multiplier();
  else if(strcmp_P(rx_command, F("set_is_homed_override"))) gcode_set_is_homed_override();
  else if(strcmp_P(rx_command, F("set_is_homing_override"))) gcode_set_is_homing_override();
  else if(strcmp_P(rx_command, F("set_coolstep_threshold"))) gcode_set_coolstep_threshold();
  else if(strcmp_P(rx_command, F("set_ignore_stallguard"))) gcode_set_ignore_stallguard();
  else if(strcmp_P(rx_command, F("reset_mcu"))) gcode_reset_mcu();
  else if(strcmp_P(rx_command, F("test_sg"))) gcode_test_sg();
  #ifdef CUSTOM_GCODE
    CUSTOM_GCODE
  #endif
  else{
    Serial.print(F("unknown command '"));
    Serial.print(cmd);
    Serial.println("'");

    Serial1.print(F("unknown command '"));
    Serial1.print(cmd);
    Serial1.println("'");
    #ifdef DEBUG_PRINT
      SERIAL_PRINT(F("!!!!!!!! >> UNK CMD '"));
      SERIAL_PRINT(cmd);
      SERIAL_PRINTLN("'");
    #endif
    return;
  }


  if(ok) Serial.println("ok");

  for (size_t i = 0; i < rx_params; i++) {
    const uint8_t len = strlen(rx_param[i]);
    if(len > 0){
      strToLower(rx_param[i]);
      const int index = axis2motor(rx_param[i][0]);
      if(index > -1){
        if(!motors[index].running && motors[index].steps_to_do < 1){
          // const uint8_t next = motors[index].next_queue_index();
          // // if(motors[index].queue[next].processed || motors[index].queue[next].type == MotorQueueItemType::NOOP){
          // //   if(motors[index].running || motors[index].steps_to_do > 0){
          // //     // [pnq] empty queue but motor running
          // //   }else{
          // //     motors[index].stop();
          // //     // SERIAL_PRINTLN(F("[pnq] empty queue, stopping!"));
          // //     // Serial.println(F("[pnq] empty queue, stopping!"));
          // //
          // //   }
          // //   return false;
          // // }
          // if(!(motors[index].queue[next].processed || motors[index].queue[next].type == MotorQueueItemType::NOOP)){
          // // if(motors[index].running || motors[index].steps_to_do > 0){
          //   motors[index].start(false);
          // }
          motors[index].start(false);
        }
      }
    }
  }

  if(lcd_present && print_gcode_to_lcd){
    lcd.setCursor(0, print_gcode_to_lcd_last_row++);
    lcd.print(">");
    lcd.print(cmd);
    lcd.print("< ");
    if(print_gcode_to_lcd_last_row >= 4) print_gcode_to_lcd_last_row = 0;
  }

}



void processCommand(const char *cmd){
  processCommand(cmd, strlen(cmd));

}



void handleSerial(){
  #ifdef DEBUG_PRINT
    const uint8_t bs = Serial.available();
    static uint8_t max_bs = 0;
    if(bs > max_bs) max_bs = bs;
    if(bs>6){
      SERIAL_PRINT("(HS:");
      SERIAL_PRINT(bs);
      SERIAL_PRINT(", M:");
      SERIAL_PRINT(max_bs);
      SERIAL_PRINTLN(")");
    }
  #endif
  uint8_t cnt = 0;
  while(Serial.available()){
    const char ch = Serial.read();
    rx_buf[rx_buf_pos++] = ch;

    if(ch == '\n' || ch == ';'){
      rx_buf[--rx_buf_pos] = 0;
      if(rx_buf[rx_buf_pos - 1] == '\r') rx_buf[--rx_buf_pos] = 0;
      processCommand(rx_buf, rx_buf_pos);
      rx_buf_pos = 0;
      break;
    }
    if(++cnt >= 64) break;
  }
}



bool strcmp_P(const char *s1, const __FlashStringHelper *s2){
  PGM_P ptr = reinterpret_cast<PGM_P>(s2);
  char buf[RX_BUF_LEN];
  size_t len = 0;
  while(1){
    if((buf[len++] = pgm_read_byte(ptr++)) == 0) break;
  }
  return strcmp(s1, buf) == 0;
}



void strToLower(char *str){
  const uint8_t len = strlen(str);
  for (size_t i = 0; i < len; i++) if(65 <= str[i] && str[i] <= 90) str[i] += 32;
}



void strToUpper(char *str){
  const uint8_t len = strlen(str);
  for (size_t i = 0; i < len; i++) if(97 <= str[i] && str[i] <= 122) str[i] -= 32;
}
