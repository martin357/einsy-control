#ifndef _menu_system_h_
#define _menu_system_h_


void drawMenu();
class Menu;

class MenuItem{
public:
  MenuItem(const char*, const Menu* = nullptr);
  virtual Menu* on_press();
  const char* getTitle();
  const Menu* leads_to;
private:
  const char* title;
};


class MenuItemBack: public MenuItem{
public:
  MenuItemBack();
  Menu* on_press();
};


class MenuItemCallable: public MenuItem{
public:
  MenuItemCallable(const char*, void (*)(), bool = true);
  Menu* on_press();
  void (*callable)();
  bool do_return;
};


class MenuItemCallableArgInt8_t: public MenuItemCallable{
public:
  MenuItemCallableArgInt8_t(const char*, void (*)(int8_t), int8_t, bool = true);
  Menu* on_press();
  void (*callable)(int8_t);
  int8_t value;
};


class MenuItemCallableArgUint8_t: public MenuItemCallable{
public:
  MenuItemCallableArgUint8_t(const char*, void (*)(uint8_t), uint8_t, bool = true);
  Menu* on_press();
  void (*callable)(uint8_t);
  uint8_t value;
};


class Menu{
public:
  Menu(MenuItem* const*, uint8_t);
  void on_enter();
  void move(int8_t);
  MenuItem* const* const items;
  uint8_t items_count;
  Menu* came_from;
  int8_t current_item;
  uint8_t offset;
};


// class MenuMotor: public Menu{
// public:
//   MenuMotor(TMCStepper* const driver);
// }



extern Menu* current_menu;

#endif
