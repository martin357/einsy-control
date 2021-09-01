// #include <avr/pgmspace.h>
#include <Arduino.h>
#include "src/LiquidCrystal_Prusa.h"
#include "menu_system.h"


Menu* current_menu = nullptr;


void drawMenu(){
  if(current_menu == nullptr) return;

  lcd.clear();
  for (size_t i = 0; i < 4; i++) {
    uint8_t y = i + current_menu->offset;
    if(y >= current_menu->items_count) break;

    bool active = current_menu->current_item == y;
    lcd.setCursor(0, i);
    lcd.print(active ? "\3" : " ");
    lcd.print(current_menu->items[y]->getTitle());
  }

}


/*
  menu item
*/
MenuItem::MenuItem(const char* title, const Menu* leads_to):
  title(title),
  leads_to(leads_to){}


const char* MenuItem::getTitle(){
  return title;
}


Menu* MenuItem::on_press(){
  return nullptr;
}



/*
  menu item back
*/
MenuItemBack::MenuItemBack() : MenuItem("Back \1"){}


Menu* MenuItemBack::on_press(){
  return current_menu->came_from;
}



/*
  menu item callable
*/
MenuItemCallable::MenuItemCallable(const char* title, void (*callable)(), bool do_return):
  MenuItem(title),
  callable(callable),
  do_return(do_return){}


Menu* MenuItemCallable::on_press(){
  if(callable != nullptr) callable();
  return do_return ? current_menu->came_from : nullptr;
}



/*
  menu item callable arg int8_t
*/
MenuItemCallableArgInt8_t::MenuItemCallableArgInt8_t(const char* title, void (*callable)(int8_t), int8_t value, bool do_return):
  MenuItemCallable(title, nullptr, do_return),
  callable(callable),
  value(value){}


Menu* MenuItemCallableArgInt8_t::on_press(){
  if(callable != nullptr) callable(value);
  return do_return ? current_menu->came_from : nullptr;
}



/*
  menu item callable arg uint8_t
*/
MenuItemCallableArgUint8_t::MenuItemCallableArgUint8_t(const char* title, void (*callable)(uint8_t), uint8_t value, bool do_return):
  MenuItemCallable(title, nullptr, do_return),
  callable(callable),
  value(value){}


Menu* MenuItemCallableArgUint8_t::on_press(){
  Serial.println("[i] MenuItemCallableArgUint8_t");
  if(callable != nullptr) callable(value);
  return do_return ? current_menu->came_from : nullptr;
}



/*
  menu
*/
Menu::Menu(MenuItem* const* items, uint8_t items_count):
  items(items),
  items_count(items_count),
  came_from(nullptr),
  current_item(0),
  offset(0){}


void Menu::on_enter(){
  Serial.println("entered menu");
}


void Menu::move(int8_t amount){
  current_item += amount;
  if(current_item < 0) current_item = 0;
  if(current_item >= items_count - 1) current_item = items_count - 1;
  int8_t y = current_item - offset;
  // if(y < 0) offset += y;
  // if(y > 3) offset += (y - 3);
  if(y < 1 && offset > 0) offset += y - 1;
  if(y > 2 && offset + 4 < items_count) offset += (y - 2);
}
