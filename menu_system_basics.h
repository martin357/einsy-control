#pragma once
#include "permanent_storage.h"
// #define COUNT_ARRAY_ITEMS(X) (sizeof(X) / sizeof(X[0]))


class Menu;

class MenuItem{
public:
  MenuItem(const char*, const Menu* = nullptr);
  virtual Menu* on_press(uint16_t);
  virtual const char* getTitle();
  const Menu* leads_to;
private:
  const char* title;
};


class MenuItemBack: public MenuItem{
public:
  MenuItemBack();
  Menu* on_press(uint16_t);
};


class MenuItemSeparator: public MenuItem{
public:
  MenuItemSeparator();
};


class MenuItemToggle: public MenuItem{
public:
  MenuItemToggle(bool*, const char*, const char*, bool = false);
  Menu* on_press(uint16_t);
  const char* getTitle();
  bool* value;
  const char* title_true;
  const char* title_false;
  bool update_storage_on_change;
};


class MenuItemToggleCallable: public MenuItem{
public:
  MenuItemToggleCallable(bool (*)(), const char*, const char*, void (*)(), void (*)(), bool = false);
  Menu* on_press(uint16_t);
  const char* getTitle();
  bool (*value_getter)();
  const char* title_true;
  const char* title_false;
  void (*call_on_true)();
  void (*call_on_false)();
  bool update_storage_on_change;
};


class MenuItemCallable: public MenuItem{
public:
  MenuItemCallable(const char*, void (*)(), bool = true);
  Menu* on_press(uint16_t);
  void (*callable)();
  bool do_return;
};


template <typename T>
class MenuItemCallableArg: public MenuItemCallable{
public:
  MenuItemCallableArg(const char*, void (*)(T), T, bool = true);
  Menu* on_press(uint16_t);
  void (*callable)(T);
  T value;
};


template <typename T>
class MenuItemDynamic: public MenuItem{
public:
  MenuItemDynamic(const char*, T&, const Menu* = nullptr);
  const char* getTitle();
  T& value;
  const char* title; // make it public
};


template <typename T>
class MenuItemDynamicCallable: public MenuItem{
public:
  MenuItemDynamicCallable(const char*, T (*)());
  const char* getTitle();
  T (*value_getter)();
  const char* title; // make it public
};


class MenuItemDynamicTime: public MenuItem{
public:
  MenuItemDynamicTime(const char*, const uint32_t*, bool = false);
  const char* getTitle();
  const uint32_t* value;
  const char* title; // make it public
  bool force_show_hours;
};


class Menu{
public:
  Menu(MenuItem* const*, size_t);
  virtual void on_enter();
  virtual void on_leave();
  virtual void on_press(uint16_t);
  virtual void draw(bool = true);
  virtual void move(int8_t);
  virtual void loop();
  void go_back();
  bool has_back();
  MenuItem* const* const items;
  size_t items_count;
  Menu* came_from;
  int8_t current_item;
  uint8_t offset;
  uint16_t redraw_interval;
};


template <typename T>
class MenuRange: public Menu{
public:
  MenuRange(const char*, T&, T, T, T = 1, bool = false);
  void on_enter();
  void on_press(uint16_t);
  void draw(bool = true);
  void move(int8_t);
  const char* title;
  T& value;
  T min_value;
  T max_value;
  T step;
  bool update_storage_on_leave;
};


template <typename T>
class MenuItemRange: public MenuItemDynamic<T>{
public:
  MenuItemRange(const char*, T&, T, T, T = 1, bool = false);
  MenuRange<T> menu_range;
};


template <typename T>
class MenuList: public Menu{
public:
  MenuList(const char*, T* ,T[], size_t, bool = false);
  void on_enter();
  void on_press(uint16_t);
  void draw(bool = true);
  void move(int8_t);
  const char* title;
  T* value;
  size_t items_count;
  uint8_t index;
  T* items;
  bool update_storage_on_leave;
};



extern MenuItemBack back;
extern MenuItemSeparator separator;
extern Menu* current_menu;
extern uint8_t last_entered_motor_menu;
extern uint32_t last_menu_redraw;


extern const char pgmstr_back[]; // "Back \1"
extern const char pgmstr_separator[]; // "------------------"
