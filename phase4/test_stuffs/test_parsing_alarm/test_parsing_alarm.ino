byte alarms[] = {0,0,0,0,0,0,0,0,0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  String msg = "<";
  msg = msg + String(alarms[0]);
  for(int i = 1;i<9;i++) {
    msg = msg + "," + String(alarms[i]);
  }
  msg = msg + ">";
  
  Serial.println(msg);
}

void loop() {
  updateAllGlobalVars();
  
  for(int i = 1; i <= 9; i++) {
    String keyy = "0"+String(i)+"_ON";
    Serial.println(keyy);
    setAlarm(keyy);
    delay(500);
  }

  for(int i = 1; i <= 9; i++) {
    String keyy = "0"+String(i)+"_OFF";
    Serial.println(keyy);
    setAlarm(keyy);
    delay(500);
  }
}


void setAlarm(String key) {   // Key example: 01_ON   ;   09_OFF
  int key_index = key.substring(0,2).toInt();
  Serial.println(key_index);
  if (key.substring(3) == "ON") {
    alarms[key_index - 1] = 1;
  } else {alarms[key_index - 1] = 0;}
  
  String msg = "<";
  msg = msg + String(alarms[0]);
  for(int i = 1;i<9;i++) {
    msg = msg + "," + String(alarms[i]);
  }
  msg = msg + ">";
  
//  Serial3.println(msg); Serial3.flush();
  Serial.println(msg);
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
    for(int i = 1;i<9;i++) {
      msg = msg + "," + String(alarmzz[i]);
    }
    msg = msg + ">";
    
  //  Serial3.println(msg); Serial3.flush();
    Serial.println(msg);
    updated = true;
  }
}

// Update Buffer from serial
String listeningMega(){
  bool quit = false;
  String seriesData = "";

//  Serial.println(F("Waiting data from Mega...")); Serial.flush();
  while (!quit) {
    if (SerialM.available() > 0) {
      updated = false;
      char x = SerialM.read();
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
