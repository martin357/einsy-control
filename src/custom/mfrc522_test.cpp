#include "mfrc522_test.h"
#include <Arduino.h>
#include "../../pins.h"
#include "../../menus.h"
#include "../../hardware.h"
#ifdef CUSTOM_MFRC522_TEST

#include "../MFRC522.h"

const uint8_t ss_pin = 73; // PJ3
const uint8_t rst_pin = 2; // nAC_FAULT

MFRC522 rfid(ss_pin, rst_pin);
MFRC522::MIFARE_Key key;
byte nuidPICC[4];

// custom HEATBED stuff
void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

void loopCustom(){
  const uint32_t _millis = millis();
  const MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);

  if(!rfid.PICC_IsNewCardPresent()) return;
  if(!rfid.PICC_ReadCardSerial()) return;

  lcd.clear();
  switch (piccType) {
		case MFRC522::PICC_TYPE_ISO_14443_4: lcd.print("PICC ISO/IEC 14443-4"); break;
		case MFRC522::PICC_TYPE_ISO_18092: lcd.print("PICC ISO/IEC 18092"); break; // (NFC)
		case MFRC522::PICC_TYPE_MIFARE_MINI: lcd.print("MIFARE Mini 320bytes"); break;
		case MFRC522::PICC_TYPE_MIFARE_1K: lcd.print("MIFARE 1KB"); break;
		case MFRC522::PICC_TYPE_MIFARE_4K: lcd.print("MIFARE 4KB"); break;
		case MFRC522::PICC_TYPE_MIFARE_UL: lcd.print("MIFARE Ultralight/C"); break;
		case MFRC522::PICC_TYPE_MIFARE_PLUS: lcd.print("MIFARE Plus"); break;
		case MFRC522::PICC_TYPE_MIFARE_DESFIRE: lcd.print("MIFARE DESFire"); break;
		case MFRC522::PICC_TYPE_TNP3XXX: lcd.print("MIFARE TNP3XXX"); break;
		case MFRC522::PICC_TYPE_NOT_COMPLETE: lcd.print("UID is not complete!"); break;
		case MFRC522::PICC_TYPE_UNKNOWN:
		default: lcd.print("Unknown type");
	}
  lcd.setCursor(0, 1);

  Serial.print(F(": Card UID:"));
  dump_byte_array(rfid.uid.uidByte, rfid.uid.size);
  Serial.println();

  // rfid.PICC_DumpToSerial(&(rfid.uid));

  last_menu_redraw = _millis;
  last_lcd_reinit = _millis;
  beep(50);
  delay(500);

}



// main menu
const char pgmstr_scan_a_tag[] PROGMEM = "Scan a tag";
MenuItem item_scan_a_tag(pgmstr_scan_a_tag, nullptr);

MenuItem* const main_menu_items[] PROGMEM = {
  &item_scan_a_tag,
};
Menu main_menu(main_menu_items, sizeof(main_menu_items) / 2);



void setupCustom(){
  rfid.PCD_Init();
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }

  // main_menu.redraw_interval = 50;
}


#endif
