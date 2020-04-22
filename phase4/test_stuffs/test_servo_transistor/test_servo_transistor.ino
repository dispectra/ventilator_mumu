#include <Servo.h>
#define sig 9
#define ena 8

Servo valve1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  valve1.attach(9);
  pinMode(ena, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(ena, HIGH);
  valve1.write(60);
  delay(3000);
  digitalWrite(ena, LOW);
  delay(3000);
  digitalWrite(ena, HIGH);
  valve1.write(0);
  delay(3000);
  digitalWrite(ena, LOW);
  delay(3000);
}
