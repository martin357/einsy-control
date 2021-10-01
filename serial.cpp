#include "serial.h"
#include "hardware.h"


uint8_t rx_buf_pos = 0;
char rx_buf[RX_BUF_LEN] = {0};
uint8_t rx_delimiter_pos[RX_PARAMS];
// extern
char rx_param[RX_PARAMS][RX_PARAM_LEN];
uint8_t rx_params;
char rx_command[RX_COMMAND_LEN];



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
  // count param delimiters and record their positions
  rx_params = 0;
  memset(rx_command, 0, RX_COMMAND_LEN);
  memset(rx_param, 0, RX_PARAMS * RX_PARAM_LEN);
  for(size_t i = 0; i < len; i++){
    if(cmd[i] == ' '){
      rx_delimiter_pos[rx_params] = i;
      if(rx_params < RX_PARAMS){
        rx_params++;
      }else{
        Serial.println(F("max cmd params reached!"));
      }
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
  if(strstr_P(rx_command, F("mv")) || strstr_P(rx_command, F("move")) || strstr_P(rx_command, F("g0"))){
    Serial.println(F("motor move!"));
    for (size_t i = 0; i < rx_params; i++) {
      const uint8_t len = strlen(rx_param[i]);
      if(len > 1){
        strToLower(rx_param[i]);
        int index = -1;
        int32_t value = atoi(&rx_param[i][1]);
        switch (rx_param[i][0]) {
          case 'x': index = 0; break;
          case 'y': index = 1; break;
          case 'z': index = 2; break;
          case 'e': index = 3; break;
        }

        if(index > -1){
          motors[index].start(true);
          motors[index].ramp_to(value);
        }

      }
    }
  }


  Serial.println("ok");

}



void handleSerial(){
  while(Serial.available()){
    const char ch = Serial.read();
    rx_buf[rx_buf_pos++] = ch;

    if(ch == '\n'){
      rx_buf[--rx_buf_pos] = 0;
      if(rx_buf[rx_buf_pos - 1] == '\r') rx_buf[--rx_buf_pos] = 0;
      processCommand(rx_buf, rx_buf_pos);
      rx_buf_pos = 0;
      break;
    }
  }
}



int strstr_P(const char *s1, const __FlashStringHelper *s2){
  PGM_P ptr = reinterpret_cast<PGM_P>(s2);
  char buf[RX_BUF_LEN];
  size_t len = 0;
  while(1){
    if((buf[len++] = pgm_read_byte(ptr++)) == 0) break;
  }
  return strstr(s1, buf);
}



void strToLower(char *str){
  const uint8_t len = strlen(str);
  for (size_t i = 0; i < len; i++) if(65 <= str[i] && str[i] <= 90) str[i] += 32;
}



void strToUpper(char *str){
  const uint8_t len = strlen(str);
  for (size_t i = 0; i < len; i++) if(97 <= str[i] && str[i] <= 122) str[i] -= 32;
}
