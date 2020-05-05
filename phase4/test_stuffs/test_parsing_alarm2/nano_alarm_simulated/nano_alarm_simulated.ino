#include <SoftwareSerial.h>

SoftwareSerial SerialFrMega(11,12); // RX, TX

void setup() {
  Serial.begin(115200);
  SerialFrMega.begin(115200);

/// Debugging purpose
//  String msg = "<";
//  msg = msg + String(alarms[0]);
//  for(int i = 1;i<9;i++) {
//    msg = msg + "," + String(alarms[i]);
//  }
//  msg = msg + ">";
//  Serial.println(msg);
}

void loop() {
/// Code for Nano_Alarm
  updateAllGlobalVars();
//  String received = listeningMega();
//  Serial.print("Received: "); Serial.println(received); Serial.flush();
}

bool updated = false;
byte alarmzz[] = {0,0,0,0,0,0,0,0,0};
String lastData = "<0,0,0,0,0,0,0,0,0>";

void updateAllGlobalVars(){
  String received = listeningMega();
  if(!updated) {
    Serial.print("Received: ");
    Serial.println(received);
    Serial.flush();

    int indexStart = 0;
    int indexEnd = 0;

    for(int i = 0; i<9; i++) {
      indexEnd = received.indexOf(",", indexStart);
      alarmzz[i] = received.substring(indexStart, indexEnd).toInt();
      indexStart = indexEnd+1;
      //    Serial.println(String(i) + ": " + bufferq[i]);
    }

    String msg = "<";
    msg = msg + String(alarmzz[0]);
    for(int i = 1;i<9;i++) {msg = msg + "," + String(alarmzz[i]);}
    msg = msg + ">";
    Serial.println(msg);

    
    updated = true;
  }
}

//// Update Buffer from serial
String listeningMega(){
  bool quit = false;
  String seriesData = "";

//  Serial.println(F("Waiting data from Mega...")); Serial.flush();
  while (!quit) {
    if (SerialFrMega.available() > 0) {
      updated = false;
      char x = SerialFrMega.read();
      if (x == '>') {
        seriesData += x;
        quit = true;
      } else {
        if (x == '<') {seriesData = "";}
        seriesData += x;
      }
    } else {
      seriesData = lastData;
      quit = true;
    }
  }
  lastData = seriesData;

  //!! Dummy Data !!
//  seriesData = "<1,0,0,350,2,14,0>";

//  String seriesData2 = ;

  return seriesData.substring(1,seriesData.length()-1);
}

//String listeningMega(){
//  bool quit = false;
//  String seriesData = "";
//  
//  Serial.println(F("Waiting data from Mega...")); Serial.flush();
//  while (!quit) {
//    if (SerialFrMega.available() > 0) {
//      char x = SerialFrMega.read();
//      if (x == '>'){
//        seriesData += x;
//        quit = true;
//      } else {
//        if (x == '<') {seriesData = "";}
//        seriesData += x;
//      }
//    }
//  }
//  
//  return seriesData;
//}
