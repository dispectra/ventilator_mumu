#include <Servo.h>

Servo servoO2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
 
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);

  servoO2.attach(3);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  servoO2.write(0);
  delay(5000);
  servoO2.write(65);
  delay(5000);
  
}
