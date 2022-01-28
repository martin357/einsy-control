#include "gestures_test.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#ifdef CUSTOM_GESTURES_TEST


#define CNT 4
volatile uint8_t next_index = 0;
volatile uint8_t index[CNT] = {0};
volatile uint8_t next_order = 0;
volatile uint8_t order[CNT] = {0};
volatile bool old_state[] = {
  !(PINB & _BV(PINB6)),
  !(PINB & _BV(PINB5)),
  !(PINB & _BV(PINB4)),
  !(PINJ & _BV(PINJ3)),
};


void timer0Custom(){
  bool changed = false;
  const bool state[] = {
    !(PINB & _BV(PINB6)),
    !(PINB & _BV(PINB5)),
    !(PINB & _BV(PINB4)),
    !(PINJ & _BV(PINJ3)),
  };
  for(byte i=0; i<CNT; i++)
    if(state[i] != old_state[i]){
      changed = true;
      break;
    }

  if(changed){
    bool is_any_true = false;
    for(byte i=0; i<CNT; i++)
      if(state[i]){
        is_any_true = true;
        break;
      }

    if(is_any_true){
      for(byte i=0; i<CNT; i++)
        if(state[i] && state[i] != old_state[i] && index[i] == 0){
          index[i] = ++next_index;
          if(next_order < CNT) order[next_order++] = i + 1;
          break;
        }

    }else{
      if(order[0] == 3 && order[3] == 1){ // up
        processCommand(F("move_rot x0.5 f160"));
        enc_diff--;
        beep(1);
      }else
      if(order[0] == 1 && order[3] == 3){ // down
        processCommand(F("move_rot x-0.5 f160"));
        enc_diff++;
        beep(1);
      }else
      if(order[0] == 4 && order[3] == 2){ // left
        if(current_menu != nullptr){
          if(current_menu->has_back()){
            current_menu->go_back();
            beep(1);
          }else{
            beep(50);
          }
        }
      }else
      if(order[0] == 2 && order[3] == 4){ // right
        enc_click = 1;
        beep(1);
      }

      next_index = 0;
      next_order = 0;
      memset(&index, 0, sizeof(index));
      memset(&order, 0, sizeof(order));
    }

    memcpy(&old_state, &state, sizeof(state));
  }

}




void setupCustom(){
  // main_menu.redraw_interval = 50;
  pinModeInput(PIN_IR_1);
  pinModeInput(PIN_IR_2);
  pinModeInput(PIN_IR_3);
  pinModeInput(PIN_IR_4);

  pinModeInput(12); // X_MIN
  pinModeInput(11); // Y_MIN

  // setup comm pin interrupt
  PCICR |= (1 << PCIE0);
  PCMSK0 = 0;
  PCMSK0 |= (1 << PCINT6); // X_MIN
  PCMSK0 |= (1 << PCINT5); // Y_MIN

}


volatile byte comm_buffer = 0;
volatile uint8_t comm_index = 0;
volatile uint32_t comm_last_bang = 0;

ISR(PCINT0_vect){
  const bool dat = PINB & _BV(PINB6);
  const bool clk = PINB & _BV(PINB5);

  if(clk){
    if(dat) comm_buffer |= 1 << comm_index;
    comm_last_bang = millis();

    // Serial.print("i=");
    // Serial.print(comm_index);
    // Serial.print("\t");
    //
    // Serial.print("v=");
    // Serial.print(dat);
    // Serial.print("\t");
    //
    // Serial.print("buf=");
    // Serial.print(comm_buffer);
    // Serial.print("\t");

    if(comm_index++ >= 7){
      // Serial.print("got packet!");
      Serial.println(comm_buffer);
      comm_buffer = 0;
      comm_index = 0;
      comm_last_bang = 0;
    }

    // Serial.println();
  }
}

// void loopCustom(){
//   const uint32_t _millis = millis();
//   if(comm_last_bang > 0 && _millis - comm_last_bang > 100){
//     cli();
//     comm_buffer = 0;
//     comm_index = 0;
//     comm_last_bang = 0;
//     sei();
//     Serial.println("comm reset");
//   }
//
//   Serial.println(us_delta);
// }



void do_xy_up(){
  processCommand(F("move_rot x0.05 y-0.05 f10"));
  processCommand(F("start x y"));
}
const char pgmstr_do_xy_up[] PROGMEM = "XY up";
MenuItemCallable item_do_xy_up(pgmstr_do_xy_up, &do_xy_up, false);


void do_xy_down(){
  processCommand(F("move_rot x-0.05 y0.05 f10"));
  processCommand(F("start x y"));
}
const char pgmstr_do_xy_down[] PROGMEM = "XY down";
MenuItemCallable item_do_xy_down(pgmstr_do_xy_down, &do_xy_down, false);



// main menu
MenuItem* const main_menu_items[] PROGMEM = {
  &item_do_xy_up,
  &item_do_xy_down,
  &motor_x,
  &motor_y,
  &motor_z,
  &motor_e,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);



#endif
