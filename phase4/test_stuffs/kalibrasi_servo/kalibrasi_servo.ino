// Kode untuk kalibrasi servo

#include <Servo.h>

Servo servo1;
int sudut = 0;

void setup(){
  Serial.begin(115200);

  servo1.attach(9);
  pinMode(4, INPUT_PULLUP); // Tombol Naik
  pinMode(5, INPUT_PULLUP); // Tombol Turun

  Serial.println("==> READY");
}

void loop(){
  if(!digitalRead(4)){
    sudut += 1;
    if(sudut>65){sudut = 65;}
  } else if (!digitalRead(5)){
    sudut-=1;
    if(sudut<0){sudut = 0;}
  }

  servo1.write(sudut);
  Serial.println(sudut);

  delay(100);
}
