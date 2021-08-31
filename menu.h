#ifndef _menu_h_
#define _menu_h_


void drawMenu();


class MenuItem{

public:
  // MenuItem();
  MenuItem(const char* title);

  virtual MenuItem* on_press();
  // MenuItem* entered_from = nullptr;
  // const char* get_title();
  // void set_title(const char*);
// private:
  const char* title;
};


class Menu{
public:
  Menu(MenuItem* const* items, uint8_t items_count);

  void move(int8_t);
  MenuItem* const* const items;
  uint8_t items_count;
  int8_t current_item;
  uint8_t offset;
};



extern Menu* current_menu;

#endif
