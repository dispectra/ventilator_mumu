#include <SoftwareSerial.h>
SoftwareSerial Serial2Alarm(11,12); // RX, TX

byte alarms[] = {0,0,0,0,0,0,0,0,0};

void setup() {
  Serial.begin(115200);
  Serial2Alarm.begin(115200);
}

void loop() {
//  for(int i = 1; i <= 9; i++) {
//    String keyy = "0"+String(i)+"_ON";
//    Serial.println(keyy); // debugging purpose
//    setAlarm(keyy);
//    delay(5000);
//  }
//  Serial.println("Turning off");  // debugging purpose
//  for(int i = 1; i <= 9; i++) {
//    String keyy = "0"+String(i)+"_OFF";
//    Serial.println(keyy);
//    setAlarm(keyy);
//    delay(5000);
//  }

  String command = readSerial(); 
  Serial.println(command);
  Serial2Alarm.print(command); Serial2Alarm.flush();
}

String readSerial() {
  unsigned int timeout = 1000;
  unsigned long time_begin = millis();
  bool quit = false;
  String packet = "";

  while (!quit){
    if (Serial.available() > 0){
      char x = Serial.read();
      if (x == '>'){
        packet += x;
        quit = true;
      } else {
        if (x == '<'){packet = "";}
        packet += x;
      }
    }
    if (millis() - time_begin > timeout) {quit = true;}
  }

  return packet;
}

void setAlarm(String key) {   // Key example: 01_ON   ;   09_OFF
  int key_index = key.substring(0,2).toInt();
//  Serial.println(key_index);  // debugging
  if (key.substring(3) == "ON") {
    alarms[key_index - 1] = 1;
  } else {alarms[key_index - 1] = 0;}
  
  String msg = "<";
  msg = msg + String(alarms[0]);
  for(int i = 1;i<9;i++) {
    msg = msg + "," + String(alarms[i]);
  }
  msg = msg + ">";
  
  Serial2Alarm.println(msg); Serial2Alarm.flush();
  Serial.println(msg);
}
