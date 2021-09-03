// #include <avr/pgmspace.h>
#include <Arduino.h>
#include "src/LiquidCrystal_Prusa.h"
#include "menu_system.h"



/*
  menu item
*/
MenuItem::MenuItem(const char* title, const Menu* leads_to):
  title(title),
  leads_to(leads_to){}


const char* MenuItem::getTitle(){
  return title;
}


Menu* MenuItem::on_press(uint16_t duration){
  return nullptr;
}



/*
  menu item back
*/
MenuItemBack::MenuItemBack() : MenuItem("Back \1"){}


Menu* MenuItemBack::on_press(uint16_t duration){
  if(current_menu->came_from != nullptr){
    current_menu->on_leave();
    current_menu->came_from->on_enter();
    current_menu = current_menu->came_from;
  }
  return nullptr;
}



/*
  menu item toggle
*/
MenuItemToggle::MenuItemToggle(const bool* value, const char* title_true, const char* title_false):
  MenuItem(nullptr),
  value(value),
  title_true(title_true),
  title_false(title_false){}


Menu* MenuItemToggle::on_press(uint16_t duration){
  (*value) = !(*value);
  return nullptr;
}


const char* MenuItemToggle::getTitle(){
  return (*value) ? title_true : title_false;
}



/*
  menu item toggle callable
*/
MenuItemToggleCallable::MenuItemToggleCallable(bool (*value_getter)(), const char* title_true, const char* title_false, void (*call_on_true)(), void (*call_on_false)()):
  MenuItem(nullptr),
  value_getter(value_getter),
  title_true(title_true),
  title_false(title_false),
  call_on_true(call_on_true),
  call_on_false(call_on_false){}


Menu* MenuItemToggleCallable::on_press(uint16_t duration){
  if(value_getter()) call_on_true();
  else call_on_false();
  return nullptr;
}


const char* MenuItemToggleCallable::getTitle(){
  return value_getter() ? title_true : title_false;
}



/*
  menu item callable
*/
MenuItemCallable::MenuItemCallable(const char* title, void (*callable)(), bool do_return):
  MenuItem(title),
  callable(callable),
  do_return(do_return){}


Menu* MenuItemCallable::on_press(uint16_t duration){
  if(callable != nullptr) callable();
  if(do_return && current_menu->came_from != nullptr){
    current_menu->on_leave();
    current_menu->came_from->on_enter();
    current_menu = current_menu->came_from;
  }
  return nullptr;
}



/*
  menu item callable w/ arg
*/
template <typename T>
MenuItemCallableArg<T>::MenuItemCallableArg(const char* title, void (*callable)(T), T value, bool do_return):
  MenuItemCallable(title, nullptr, do_return),
  callable(callable),
  value(value){}


template <typename T>
Menu* MenuItemCallableArg<T>::on_press(uint16_t duration){
  if(callable != nullptr) callable(value);
  if(do_return && current_menu->came_from != nullptr){
    current_menu->on_leave();
    current_menu->came_from->on_enter();
    current_menu = current_menu->came_from;
  }
  return nullptr;
}


template class MenuItemCallableArg<char>;
template class MenuItemCallableArg<int8_t>;
template class MenuItemCallableArg<uint8_t>;
template class MenuItemCallableArg<int16_t>;
template class MenuItemCallableArg<uint16_t>;
template class MenuItemCallableArg<int32_t>;
template class MenuItemCallableArg<uint32_t>;
template class MenuItemCallableArg<double>;
template class MenuItemCallableArg<float>;



/*
  menu item dynamic
*/
template <typename T>
MenuItemDynamic<T>::MenuItemDynamic(const char* title, T& value):
  MenuItem(title, nullptr),
  value(value){}


template <typename T>
const char* MenuItemDynamic<T>::getTitle(){
  return "I AM DYNAMIC";
}


template class MenuItemDynamic<uint8_t>;
template class MenuItemDynamic<uint16_t>;



/*
  menu item dynamic callable
*/
template <typename T>
MenuItemDynamicCallable<T>::MenuItemDynamicCallable(const char* title, T (*value_getter)()):
  MenuItem(title, nullptr),
  title(title),
  value_getter(value_getter){}


template <typename T>
const char* MenuItemDynamicCallable<T>::getTitle(){
  static char buf[20] = {0};
  char buf_num[10] = {0};
  T value = value_getter();
  itoa(value, buf_num, 10);

  memset(buf, ' ', sizeof(buf));
  buf[sizeof(buf) - 1] = 0;

  memcpy(buf, title, strlen(title));
  memcpy(buf + strlen(title), ": ", 2);
  memcpy(buf + 18 - strlen(buf_num), buf_num, strlen(buf_num));

  return buf;
}


template class MenuItemDynamicCallable<uint8_t>;
template class MenuItemDynamicCallable<uint16_t>;



/*
  menu
*/
Menu::Menu(MenuItem* const* items, uint8_t items_count):
  items(items),
  items_count(items_count),
  came_from(nullptr),
  current_item(0),
  offset(0),
  redraw_interval(0){}


void Menu::on_enter(){
}


void Menu::on_leave(){
}


void Menu::on_press(uint16_t duration){
  Menu* new_menu = nullptr;
  if(items[current_item]->leads_to != nullptr){
    new_menu = items[current_item]->leads_to;
  }else{
    new_menu = items[current_item]->on_press(duration);
  }

  if(new_menu != nullptr){
    current_menu->on_leave();
    (*new_menu).came_from = current_menu;
    new_menu->on_enter();
    current_menu = new_menu;
  }

  draw();
}


void Menu::draw(bool clear){
  if(current_menu == nullptr) return;

  if(clear) lcd.clear();
  for (size_t i = 0; i < 4; i++) {
    uint8_t y = i + offset;
    if(y >= items_count) break;

    bool active = current_item == y;
    lcd.setCursor(0, i);
    lcd.print(active ? "\3" : " ");
    lcd.print(items[y]->getTitle());
  }
  last_menu_redraw = millis();
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



/*
  menu motor
*/
MenuMotor::MenuMotor(uint8_t index, MenuItem* const* items, uint8_t items_count):
  Menu(items, items_count),
  index(index){}


void MenuMotor::on_enter(){
  last_entered_motor_menu = index;
}



MenuItemBack back;
Menu* current_menu = nullptr;
uint8_t last_entered_motor_menu = 0;
uint32_t last_menu_redraw = 0;
