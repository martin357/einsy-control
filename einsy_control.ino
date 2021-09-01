// #include <stdint.h>
#include "src/LiquidCrystal_Prusa.h"
#include "pins.h"
#include "hardware.h"
#include "menu_system.h"
#include "src/TMCStepper.h"


MenuItemBack back;

MenuItem start("Start \3");
MenuItem stop("Stop  \4");

MenuItem* motor1_items[] = {
  &back,
  &start,
  &stop,
};
Menu motor1_menu(motor1_items, sizeof(motor1_items) / 2);

MenuItem* motor2_items[] = {
  &back,
  &start,
  &stop,
  &back,
};
Menu motor2_menu(motor2_items, sizeof(motor2_items) / 2);

void foobar(){ Serial.println("callable clicked"); }
void foobar_arg(int8_t val){ Serial.print("callable(with arg) clicked! "); Serial.println(val); }
void foobar_uarg(uint8_t val){ Serial.print("callable(with unsigned arg) clicked! "); Serial.println(val); }
MenuItemCallable foobar1("Callable!", &foobar, false);

MenuItemCallableArgInt8_t foobar_arg1("ArgCall! -16", &foobar_arg, -16, false);
MenuItemCallableArgInt8_t foobar_arg2("ArgCall! -128", &foobar_arg, -128, false);
MenuItemCallableArgInt8_t foobar_arg3("ArgCall! -255", &foobar_arg, -255, false);

MenuItemCallableArgUint8_t foobar_uarg1("ArgU_Call! 16", &foobar_uarg, 16, false);
MenuItemCallableArgUint8_t foobar_uarg2("ArgU_Call! 128", &foobar_uarg, 128, false);
MenuItemCallableArgUint8_t foobar_uarg3("ArgU_Call! 255", &foobar_uarg, 255, false);

MenuItem motor1("Motor X", &motor1_menu);
MenuItem motor2("Motor Y", &motor2_menu);
MenuItem foo1("foobar 1");
MenuItem foo2("\1derp2");
MenuItem foo3("\2test 3");
MenuItem foo4("\3line 4");
MenuItem foo5("\4five");
MenuItem foo6("6");
MenuItem foo7("baz7");
MenuItem foo8("lipsum8");

MenuItem* main_menu_items[] = {
  &motor1,
  &motor2,
  &foo1,
  &foo2,
  &foo3,
  &foobar1,
  &foo4,
  &foo5,
  &foobar_arg1,
  &foobar_arg2,
  &foobar_arg3,
  &foobar_uarg1,
  &foobar_uarg2,
  &foobar_uarg3,
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

// X_TMC2130_CS           41
// #define X_TMC2130_DIAG         64 // !!! changed from 40 (EINY03)
// #define X_STEP_PIN             37
// #define X_DIR_PIN              49
// #define X_MIN_PIN            0 // 12
// //#define X_MAX_PIN            30
// //#define X_MIN_PIN              X_TMC2130_DIAG
// #define X_MAX_PIN              X_TMC2130_DIAG
// #define X_ENABLE_PIN           29
// #define X_MS1_PIN           -1
// #define X_MS2_PIN

TMC2130Stepper driver = TMC2130Stepper(X_TMC2130_CS, 0.2f);


// struct {
//     uint8_t blank_time = 24;        // [16, 24, 36, 54]
//     uint8_t off_time = 3;           // [1..15]
//     uint8_t hysteresis_start = 1;   // [1..8]
//     int8_t hysteresis_end = 12;     // [-3..12]
// } config;

#define CHOPPER_TIMING { 3, 1, 1 }
// ISR(TIMER1_COMPA_vect){
//   PORTC |= 1 << PINC0; // STEP_PORT |= 1 << STEP_BIT;
//   PORTC &= ~(1 << PINC0); // STEP_PORT &= ~(1 << STEP_BIT);
// }
ISR(TIMER1_COMPA_vect){
  PORTC ^= 1 << PINC0; // STEP_PORT ^= 1 << STEP_BIT_POS;
  // digitalWrite(X_STEP_PIN, !digitalRead(X_STEP_PIN));
}

// void reportCurrentSettings() {
//   Serial.print("Off time = ");
//   Serial.print(config.off_time);
//   Serial.print(" Hysteresis end = ");
//   Serial.print(config.hysteresis_end);
//   Serial.print(" Hysteresis start = ");
//   Serial.println(config.hysteresis_start);
// }

#define STALL_VALUE      0 // [-64..63]
void setup() {
  pinModeOutput(X_ENABLE_PIN);
  pinModeOutput(X_DIR_PIN);
  pinModeOutput(X_STEP_PIN);
  pinModeOutput(X_TMC2130_CS);

  digitalWriteExt(X_ENABLE_PIN, HIGH); //deactivate driver (LOW active)
  digitalWriteExt(X_DIR_PIN, LOW); //LOW or HIGH
  digitalWriteExt(X_STEP_PIN, LOW);
  digitalWriteExt(X_TMC2130_CS, HIGH);

  SPI.begin();
  pinModeInput(MISO, true);

  setupPins();
  setupLcd();
  Serial.begin(115200);

  current_menu = &main_menu;
  drawMenu();


  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // mA
  // driver.microsteps(16);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);

  driver.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver.pwm_autoscale(1);
  driver.microsteps(256);


  // Set stepper interrupt
  {
    cli();//stop interrupts
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    OCR1A = 256;// = (16*10^6) / (1*1024) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS11 bits for 8 prescaler
    TCCR1B |= (1 << CS11);// | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei();//allow interrupts
  }

  digitalWrite( X_ENABLE_PIN,  LOW );
}


#define MAX_SPEED        40 // In timer value
#define MIN_SPEED      1000
void loop() {
  uint32_t _millis = millis();

  driver.step();
  static uint32_t last_time=0;
  uint32_t ms = millis();

  while(Serial.available() > 0) {
    int8_t read_byte = Serial.read();
    #ifdef USING_TMC2660
      if (read_byte == '0')      { TIMSK1 &= ~(1 << OCIE1A); driver.toff(0); }
      else if (read_byte == '1') { TIMSK1 |=  (1 << OCIE1A); driver.toff(driver.savedToff()); }
    #else
      if (read_byte == '0')      { TIMSK1 &= ~(1 << OCIE1A); digitalWrite( X_ENABLE_PIN, HIGH ); }
      else if (read_byte == '1') { TIMSK1 |=  (1 << OCIE1A); digitalWrite( X_ENABLE_PIN,  LOW ); }
    #endif
    else if (read_byte == '+') { if (OCR1A > MAX_SPEED) OCR1A -= 20; }
    else if (read_byte == '-') { if (OCR1A < MIN_SPEED) OCR1A += 20; }
  }

  if((ms-last_time) > 200) { //run every 0.1s
    last_time = ms;

    TMC2130_n::DRV_STATUS_t drv_status{0};
    drv_status.sr = driver.DRV_STATUS();

    Serial.print("0 ");
    Serial.print(drv_status.sg_result, DEC);
    Serial.print(" ");
    Serial.println(driver.cs2rms(drv_status.cs_actual), DEC);
    // lcd.clear();
    // lcd.setCursor(0, 0);
    // lcd.print(drv_status.sg_result);
    // lcd.print("  ");
    // lcd.setCursor(0, 1);
    // lcd.print(driver.cs2rms(drv_status.cs_actual));
    // lcd.print("  ");

  }
  // return;
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

  if(enc_click){
    Menu* new_menu = nullptr;
    if(current_menu->items[current_menu->current_item]->leads_to != nullptr){
      new_menu = current_menu->items[current_menu->current_item]->leads_to;
      Serial.println("  shall go to another menu!");
    }else{
      new_menu = current_menu->items[current_menu->current_item]->on_press();
    }

    if(new_menu != nullptr){
      (*new_menu).came_from = current_menu;
      new_menu->on_enter();
      current_menu = new_menu;
    }

    enc_click = 0;
    drawMenu();

  }

  if(beeper_off_at && _millis > beeper_off_at){
    beeper_off_at = 0;
    digitalWriteExt(BEEPER, LOW);
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
  lcd.print(foo1.getTitle());
  lcd.setCursor(0, 2);
  lcd.print(foo2.getTitle());

  // digitalWrite(BEEPER, !digitalRead(BEEPER));
  delay(600);

}
