#include "Led.h"
#include "Button.h"
#include "Buzzer.h"

#define LED_1_PIN 13
#define LED_2_PIN 4
#define BUTTONSILENT_PIN 2
#define BUTTONRESET_PIN 3
#define BUZZER_PIN 8

Led led1(LED_1_PIN);
Led led2(LED_2_PIN);
Button buttonSilent (BUTTONSILENT_PIN);
Button buttonReset (BUTTONRESET_PIN);
Buzzer buzzer1 (BUZZER_PIN);
Buzzer buzzer2 (BUZZER_PIN);
Buzzer buzzer3 (BUZZER_PIN);

bool alarmCheckHigh = true;
bool alarmCheckMedium = false;


bool previousSilentState = true;
bool currentSilentState = true;

unsigned long nextUpdate;
unsigned long timeoutAmount = 1000;

void setup() {
  Serial.begin(9600);
}



void loop() {
  buzzer1.interval = 100;
  buzzer2.interval = 300;
  buzzer3.interval = 700;
  bool silentReading = buttonSilent.isPressed();
  bool isEverOn = false;
  
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



//Alarm decisions
  if(alarmCheckHigh && currentSilentState){
      isEverOn = true;
      led1.on();
      buzzer1.on();
      //Serial.println("Alarm");
  }
  else if(alarmCheckMedium && currentSilentState){
      isEverOn = true;
      led2.on();
      buzzer2.on();
      //Serial.println("Alarm");
  }
  else{
      //led1.off();
      buzzer1.off();
      buzzer2.off();
      //Serial.println("Alarm Silent");
      }

      
//save alarm check state ON unless reset button is pressed (or under other unspecified circumstances)
if(isEverOn){
  alarmCheckHigh = true;
  alarmCheckMedium = true;
  }

//Reset the Alarm
  if(buttonReset.isPressed()){
    alarmCheckHigh = false;
    alarmCheckMedium = false;
    isEverOn = false;
    //led1.off();
    Serial.println("Alarm Reset");
    }
  else{

    }
}
