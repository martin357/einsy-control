// #include <stdint.h>
#include "src/LiquidCrystal_Prusa.h"
#include "pins.h"
#include "hardware.h"
#include "menu.h"


// MenuItem* current_menu[] = {nullptr};
// uint8_t current_menu_len = 0;
// MenuItem* const* current_menu;

MenuItem foo1("foobar 1");
MenuItem foo2("derp2");
MenuItem foo3("test 3");
MenuItem foo4("line 4");
MenuItem foo5("five");
MenuItem foo6("6");
MenuItem foo7("baz7");
MenuItem foo8("lipsum8");

MenuItem* main_menu_items[] = {
  &foo1,
  &foo2,
  &foo3,
  &foo4,
  &foo5,
  &foo6,
  &foo7,
  &foo8,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);

MenuItem* menu_3_items[] = {
  &foo2,
  &foo4,
  &foo5,
};
MenuItem* menu_5_items[] = {
  &foo1,
  &foo2,
  &foo3,
  &foo4,
  &foo5,
};
MenuItem* menu_6_items[] = {
  &foo1,
  &foo2,
  &foo3,
  &foo4,
  &foo5,
  &foo6,
};
Menu menu_3(menu_3_items, sizeof(menu_3_items) / 2);
Menu menu_5(menu_5_items, sizeof(menu_5_items) / 2);
Menu menu_6(menu_6_items, sizeof(menu_6_items) / 2);



void setup() {
  setupPins();
  // current_menu[0] = main_menu[0];
  // current_menu = &main_menu;
  // current_menu_len = sizeof(main_menu) / 2;

  Serial.begin(115200);
  // Serial.println(current_menu_len);
  Serial.print("main_menu_items\t");
  Serial.println(sizeof(main_menu_items));

  Serial.print("menu_3_items\t");
  Serial.println(sizeof(menu_3_items));

  Serial.print("menu_5_items\t");
  Serial.println(sizeof(menu_5_items));

  Serial.print("menu_6_items\t");
  Serial.println(sizeof(menu_6_items));

  Serial.print("main_menu items_count\t");
  Serial.println(main_menu.items_count);

  Serial.print("menu_3 items_count\t");
  Serial.println(menu_3.items_count);

  Serial.print("menu_5 items_count\t");
  Serial.println(menu_5.items_count);

  Serial.print("menu_6 items_count\t");
  Serial.println(menu_6.items_count);

  // current_menu = &menu_3;
  current_menu = &main_menu;

  lcd.setBrightness(128);
  lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("START");
  drawMenu();

}



void loop() {
  // static int8_t last = 0;
  // static uint8_t last = digital
  // delay(500);
  readEncoder();
  // if(last != enc_diff){
  //   drawMenu();
  //   lcd.print(" ");
  //   char buf[8] = {0};
  //   itoa(enc_diff, buf, 5);
  //   lcd.print(buf, 3, 1);
  //   // Serial.println(enc_diff);
  //
  //   Serial.println(current_menu->items[0]->title);
  //   Serial.println(current_menu->items[1]->title);
  //   Serial.println(current_menu->items[2]->title);
  //   Serial.println(sizeof(*current_menu->items));
  //
  //
  //   last = enc_diff;
  // }
  if(enc_diff){
    current_menu->move(enc_diff);
    enc_diff = 0;

    drawMenu();
  }
  return;

  // Serial.println("loop");
  // delay(400);
  // return;

  // digitalWrite(BEEPER, digitalRead(BTN_ENC) ? HIGH : LOW);
  Serial.print("loop ");
  Serial.print(digitalRead(BEEPER));
  Serial.print(" ");
  Serial.println(digitalRead(BTN_ENC));

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("fooo bar");
  lcd.setCursor(0, 1);
  lcd.print(foo1.title);
  lcd.setCursor(0, 2);
  lcd.print(foo2.title);

  // digitalWrite(BEEPER, !digitalRead(BEEPER));
  delay(600);

}
