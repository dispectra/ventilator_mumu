#include "Button.h"
Button::Button(byte pin) {
  this->pin = pin;
  lastReading = LOW;
  init();
}
void Button::init() {
  pinMode(pin, INPUT);
  update();
}
void Button::update() {
    //handle decode
    byte newReading = digitalRead(pin);
    
    if (newReading != lastReading) {
      lastDebounceTime = millis();
    }
    if (millis() - lastDebounceTime > debounceDelay) {
      // Update the 'state' attribute only if debounce is checked
      state = newReading;
    }
    lastReading = newReading;
}
byte Button::getState() {
  update();
  return state;
}
bool Button::isPressed() {
  return (getState() == HIGH);
}
