#ifndef _menu_system_h_
#define _menu_system_h_



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


class MenuItemToggle: public MenuItem{
public:
  MenuItemToggle(const bool*, const char*, const char*);
  Menu* on_press(uint16_t);
  const char* getTitle();
  bool* value;
  const char* title_true;
  const char* title_false;
};


class MenuItemToggleCallable: public MenuItem{
public:
  MenuItemToggleCallable(bool (*)(), const char*, const char*, void (*)(), void (*)());
  Menu* on_press(uint16_t);
  const char* getTitle();
  bool (*value_getter)();
  const char* title_true;
  const char* title_false;
  void (*call_on_true)();
  void (*call_on_false)();
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
  MenuItemDynamic(const char*, T&);
  const char* getTitle();
  T& value;
};


template <typename T>
class MenuItemDynamicCallable: public MenuItem{
public:
  MenuItemDynamicCallable(const char*, T (*)());
  const char* getTitle();
  T (*value_getter)();
  const char* title; // make it public
};


class Menu{
public:
  Menu(MenuItem* const*, uint8_t);
  virtual void on_enter();
  virtual void on_leave();
  virtual void on_press(uint16_t);
  virtual void draw(bool = true);
  virtual void move(int8_t);
  MenuItem* const* const items;
  uint8_t items_count;
  Menu* came_from;
  int8_t current_item;
  uint8_t offset;
  uint16_t redraw_interval;
};


class MenuMotor: public Menu{
public:
  MenuMotor(uint8_t, MenuItem* const*, uint8_t);
  void on_enter();
  uint8_t index;
};



extern MenuItemBack back;
extern Menu* current_menu;
extern uint8_t last_entered_motor_menu;
extern uint32_t last_menu_redraw;



#endif
