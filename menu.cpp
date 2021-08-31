// #include <avr/pgmspace.h>
#include <Arduino.h>
#include "src/LiquidCrystal_Prusa.h"
#include "menu.h"


Menu* current_menu = nullptr;


void drawMenu(){
  if(current_menu == nullptr) return;

  lcd.clear();
  for (size_t i = 0; i < 4; i++) {
    uint8_t y = i + current_menu->offset;
    if(y >= current_menu->items_count) break;

    bool active = current_menu->current_item == y;
    // lcd.setCursor(active ? 0 : 1, i);
    // if(active) lcd.print(">");
    lcd.setCursor(0, i);
    lcd.print(active ? ">" : " ");
    lcd.print(current_menu->items[y]->title);
  }

}

// MenuItem::MenuItem(){}
MenuItem::MenuItem(const char* title): title(title){}


// const char* MenuItem::get_title(){
//   return title;
// }


// void MenuItem::set_title(const char* value){
//   title = value;
// }


MenuItem* MenuItem::on_press(){
  Serial.print("virtual ");
  Serial.print(title);
  Serial.print(" on_press()");
}


/*
  menu
*/
Menu::Menu(MenuItem* const* items, uint8_t items_count):
  items(items),
  items_count(items_count)
{
  current_item = 0;
  offset = 0;
}


void Menu::move(int8_t amount){
  current_item += amount;
  if(current_item < 0) current_item = 0;
  if(current_item >= items_count - 1) current_item = items_count - 1;
  int8_t y = current_item - offset;
  if(y < 0) offset += y;
  if(y > 3) offset += (y - 3);
}
