#include	<Arduino.h>
#include "pins.h"



uint8_t _mask(const uint8_t pin){
  switch (pin) {
    case 70: return 1 << PING4;
    case 71: return 1 << PING3;
    case 72: return 1 << PINJ2;
    case 73: return 1 << PINJ3;
    case 74: return 1 << PINJ7;
    case 75: return 1 << PINJ4;
    case 76: return 1 << PINJ5;
    case 77: return 1 << PINJ6;
    case 78: return 1 << PINE2;
    case 79: return 1 << PINE6;
    case 80: return 1 << PINE7;
    case 81: return 1 << PIND4;
    case 82: return 1 << PIND5;
    case 83: return 1 << PIND6;
    case 84: return 1 << PINH2;
    case 85: return 1 << PINH7;
  }
}


void pinModeInput(const uint8_t pin, const bool pullup){
  if(pin < 70){
    pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
    return;
  }
  if(pin >= 84) DDRH &= ~_mask(pin);
  else if(pin >= 81) DDRD &= ~_mask(pin);
  else if(pin >= 78) DDRE &= ~_mask(pin);
  else if(pin >= 72) DDRJ &= ~_mask(pin);
  else if(pin >= 70) DDRG &= ~_mask(pin);
  digitalWriteExt(pin, pullup);
}


void pinModeOutput(const uint8_t pin){
  if(pin < 70){
    pinMode(pin, OUTPUT);
    return;
  }
  if(pin >= 84){ DDRH |= _mask(pin); return; }
  if(pin >= 81){ DDRD |= _mask(pin); return; }
  if(pin >= 78){ DDRE |= _mask(pin); return; }
  if(pin >= 72){ DDRJ |= _mask(pin); return; }
  if(pin >= 70){ DDRG |= _mask(pin); return; }
}


bool digitalReadExt(const uint8_t pin){
  if(pin < 70) return digitalRead(pin);
  volatile uint8_t* reg;

  if(pin >= 84) reg = &PINH;
  else if(pin >= 81) reg = &PIND;
  else if(pin >= 78) reg = &PINE;
  else if(pin >= 72) reg = &PINJ;
  else if(pin >= 70) reg = &PING;

  return (bool)(*reg & _mask(pin));
}


void digitalWriteExt(const uint8_t pin, const bool value){
  if(pin < 70){
    digitalWrite(pin, value);
    return;
  }
  volatile uint8_t* reg;

  if(pin >= 84) reg = &PORTH;
  else if(pin >= 81) reg = &PORTD;
  else if(pin >= 78) reg = &PORTE;
  else if(pin >= 72) reg = &PORTJ;
  else if(pin >= 70) reg = &PORTG;

  if(value) *reg |= _mask(pin);
  else *reg &= ~_mask(pin);
}
