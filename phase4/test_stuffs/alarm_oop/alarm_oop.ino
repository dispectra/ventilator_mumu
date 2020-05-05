#include "Led.h"
#include "Button.h"
#include "Buzzer.h"
#include <SoftwareSerial.h>

#define LED_1_PIN 9
#define LED_2_PIN 8
#define LED_3_PIN 7
#define LED_4_PIN 6
#define LED_5_PIN 5
#define LED_6_PIN 4 
#define LED_7_PIN 12
#define LED_8_PIN 14 //D14 = A0

#define BUZZER_1_PIN 10
#define BUTTONSILENT_PIN 2
#define BUTTONRESET_PIN 3

Led led1(LED_1_PIN);
Led led2(LED_2_PIN);
Led led3(LED_3_PIN);
Led led4(LED_4_PIN);
Led led5(LED_5_PIN);
Led led6(LED_6_PIN);
Led led7(LED_7_PIN);
Led led8(LED_8_PIN);

Button buttonSilent (BUTTONSILENT_PIN);
Button buttonReset (BUTTONRESET_PIN);
Buzzer buzzer1 (BUZZER_1_PIN);
Buzzer buzzer2 (BUZZER_1_PIN);
Buzzer buzzer3 (BUZZER_1_PIN);

//Serial communication
SoftwareSerial SerialFrMega(11,12); // RX, TX

//silent button variables
bool previousSilentState = true;
bool currentSilentState = true;

bool isEverOn = false;
bool updated = false;

/*Define array of alarm triggers
alarmzz[0] = High pressure exceeded PIP (HIGH)
alarmzz[1] = Pressure too low (HIGH)
alarmzz[2] = Patient is fighting (HIGH)
alarmzz[3] = Overcurrent fault (HIGH)
alarmzz[4] = Sporious breath (MEDIUM)
alarmzz[5] = Overtidal volume (MEDIUM)
alarmzz[6] = Low PEEP (MEDIUM)
alarmzz[7] = Low/Oversupply of Oxygen (LOW)
*/
byte alarmzz[] = {0,0,0,0,0,0,0,0,0};
String lastData = "<0,0,0,0,0,0,0,0,0>";

void setup() {
  Serial.begin(115200);
  SerialFrMega.begin(115200);
}


void loop() {
  
  updateAllGlobalVars();
  
  buzzer1.interval = 100;
  buzzer2.interval = 300;
  buzzer3.interval = 700;
  bool silentReading = buttonSilent.isPressed();

  
//Silent Button Toggle
  if(silentReading && !previousSilentState){
    if(currentSilentState){
      currentSilentState = false;
      }
    else{
      currentSilentState = true;
      }
    }
  previousSilentState = silentReading;



//Set of alarm decisions of high priority
//  if(alarmzz[0] && currentSilentState){
//      isEverOn = true;
//      led1.on();
//      buzzer1.on();
//  }
//  if(alarmzz[1] && currentSilentState){
//      isEverOn = true;
//      led2.on();
//      buzzer1.on();
//  }
  //else if(alarmzz[2] && currentSilentState){
  //    isEverOn = true;
  //    led3.on();
  //    buzzer1.on();
//  }
  //else if(alarmzz[3] && currentSilentState){
  //    isEverOn = true;
  //    led4.on();
  //    buzzer1.on();
//  }
//  else{
//      buzzer1.off();
//      Serial.println("Alarm Silent");
//      }

//Set of alarm decisions of medium priority
//  if(alarmzz[4] && currentSilentState){
//      isEverOn = true;
//      led3.on();
//      buzzer2.on();
//  }
//  else if(alarmzz[5] && currentSilentState){
//      isEverOn = true;
//      led6.on();
//      buzzer2.on();
//  }
//  else if(alarmzz[6] && currentSilentState){
//      isEverOn = true;
//      led7.on();
//      buzzer2.on();
//  }
//  else{
//      buzzer2.off();
//      Serial.println("Alarm Silent");
//      }

//Set of alarm decisions of low priority
  if(alarmzz[7] && currentSilentState){
      isEverOn = true;
      led4.on();
      buzzer3.on();
  }
  else{
      buzzer3.off();
      Serial.println("Alarm Silent");
      }


      
//save alarm check state ON unless reset button is pressed (or under other unspecified circumstances)
if(isEverOn){
  int i = 0;
  for(i = 0; i <= 7; i++){
  alarmzz[i] = true;}
  }

//Reset the Alarm
  if(buttonReset.isPressed()){
    int i;
      for (i = 0; i <= 7; i++){
        alarmzz[i] = false;
        }
    led1.off();
    led2.off();
    led3.off();
    led4.off();
    led5.off();
    led6.off();
    led7.off();
    led8.off();     
    isEverOn = false;
    Serial.println("Alarm Reset");
    }
}
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
