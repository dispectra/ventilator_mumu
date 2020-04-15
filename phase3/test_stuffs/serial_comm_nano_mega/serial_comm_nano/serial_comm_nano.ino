/* Arduino Nano
 * Serial communication to/from Arduino Nano through SoftSerial
 */
#include <SoftwareSerial.h>

SoftwareSerial SerialM(2,3); //RX, TX

void setup() {
  Serial.begin(115200);
  SerialM.begin(38400);
}

void loop() {
  String received = listeningMega();
  Serial.print("Received: "); Serial.println(received); Serial.flush();

  doStepperThings();
}

//== FUNCTIONS ==============================================

// Listen serial communication from Arduino Mega on port SerialM
String listeningMega(){
  bool quit = false;
  String seriesData = "";
  
  Serial.println(F("Waiting data from Mega...")); Serial.flush();
  while (!quit) {
    if (SerialM.available() > 0) {
      char x = SerialM.read();
      if (x == '>'){
        seriesData += x;
        quit = true;
      } else {
        if (x == '<') {seriesData = "";}
        seriesData += x;
      }
    }
  }
  
  return seriesData;
}

/* 
// listeningMegaOld(), with timeout
String listeningMegaOld(unsigned long timeout){
  bool quit = false;
  String seriesData;
  unsigned long timer;
  Serial.print(F("Waiting data from Mega...")); Serial.flush();
  timer = millis();
  while ((!quit) && (millis() - timer <= timeout)){
    if (millis() % 1000 == 0){
      Serial.print("."); Serial.flush();
      digitalWrite(LED_PIN,!(digitalRead(LED_PIN)));
      delay(1);
    }
    if (ardSerial.available() > 0){
      char x = ardSerial.read();
      if (x == '\n'){
        quit = true;
      } else {
        if (x != ' '){
          seriesData += x;
        }
      }
    }
  }
  return seriesData;
}
//*/

void doStepperThings() {
  Serial.println("Inhale"); Serial.flush();
  delay(500);
  Serial.println("Exhale"); Serial.flush();
  delay(500);
}
