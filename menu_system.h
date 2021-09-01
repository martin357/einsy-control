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


template <typename T>
class MenuItemCallableArg: public MenuItemCallable{
public:
  MenuItemCallableArg(const char*, void (*)(T), T, bool = true);
  Menu* on_press();
  void (*callable)(T);
  T value;
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
