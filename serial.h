#pragma once
#include <Arduino.h>


#define RX_PARAM_LEN 64
#define RX_PARAMS 8
#define RX_COMMAND_LEN 32
#define RX_BUF_LEN 128


void processCommand(const __FlashStringHelper*);
void processCommand(const char*, size_t);
void handleSerial();
bool strcmp_P(const char*, const __FlashStringHelper*);
void strToLower(char*);
void strToUpper(char*);


extern char rx_param[RX_PARAMS][RX_PARAM_LEN];
extern uint8_t rx_params;
extern char rx_command[RX_COMMAND_LEN];
