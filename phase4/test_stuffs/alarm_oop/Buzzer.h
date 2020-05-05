#ifndef MY_BUZZER_H
#define MY_BUZZER_H

#include <Arduino.h>

class Buzzer{
  private:
    byte pin;
    byte state;
    unsigned long previousMillis = 0;
  public:
    int interval;
    Buzzer(byte pin);
    void init();
    void on(); 
    void off();
  }; 

#endif
