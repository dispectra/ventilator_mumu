#include "Buzzer.h"

Buzzer::Buzzer(byte pin){
      this->pin = pin;
      init();
      }
void Buzzer::init(){
      pinMode(pin, OUTPUT);
      }
void Buzzer::on(){
      unsigned long currentMillis = millis();
      if(currentMillis - previousMillis >= interval){
        previousMillis = currentMillis;

        if (state == LOW){
            state = HIGH;
            tone(pin, 262);
          } else {
            state = LOW;
            noTone(pin);
            }
        }
      }  
void Buzzer::off(){
      noTone(pin);
      }
