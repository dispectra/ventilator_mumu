#include <Wire.h>

//Button & Buzzer Variables

const int inPin = 2;         // the number of the input pin
const int outPin = 13;       // the number of the output pin
const int buzzer = 9;        // the number of the buzzer pin

//Button State Variables
int buttonState = HIGH;      // the current state of the output pin
int buttonReading;           // the current reading from the input pin
int buttonPreviousState = LOW;    // the previous reading from the input pin

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers


int buzzerState = LOW; 

// 0 = high pressure
// 1 = low pressure
// 2 = fighting
// 3 = oxygen & pressure
// 4 = oxygen & fighting
// 5 = pressure & fighting
// 6 = pressure, oxygen, & fighting

//Period
// 0 = HIGH
// 1 = MEDIUM
// 2 = LOW
int period_alarm[] = {100, 400, 750};
int frequency_alarm[] = {};
unsigned long previousMillis[] = {0, 0, 0};

//unsigned long previousPressureMillis = 0;
//unsigned long previousOxygen1Millis = 0;
//unsigned long previousFightingMillis = 0;

//mock variable
//HIGH PRIORITY: PRESSURE
bool overPressure = true;
bool underPressure = false;
bool isFighting = false;
//HIGH PRIORITY: CURRENT
bool overCurrent = false;
//MEDIUM PRIORITY
bool sporiousBreath = false;
bool overTidalVolume = false;
//LOW PRIORITY
bool oxygenDifference = false;


//float setPoint  = 30;
//float oxygenPercentage = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(inPin, INPUT);
  pinMode(outPin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  Serial.begin(115200); 

}

void loop() {
  buttonReading = digitalRead(inPin);

  // if the input just went from LOW and HIGH and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
  
  if (buttonReading && !buttonPreviousState && millis() - time > debounce) {
    if (buttonState)
      buttonState = LOW;
    else
      buttonState = HIGH;

    time = millis();    
  }

//float oxygenDifference = setPoint-oxygenPercentage;
unsigned long currentMillis = millis();


  if(buttonState){

  //High Priority Pressure Case: Overpressure
    if(overPressure || underPressure || isFighting){
      if(currentMillis - previousMillis[0] >= period_alarm[0] ){
        previousMillis[0] = currentMillis;
        buzzerSound(300);
        if(overPressure){
          Serial.println("Pressure is too high! System is shutting down...");
          }
         else if(underPressure){
          Serial.println("Pressure is too low! System is shutting down...");
          }
         else if(isFighting){
          Serial.println("Patient is fighting! System is shutting down...");
          }
        return;
        }
     }
    else{
     Serial.println("No Kknown pressure Problems");
     }   
   //High Priority Current Case: Overcurrent
    if(overCurrent){
      if(currentMillis - previousMillis[0] >= period_alarm[0] ){
        previousMillis[0] = currentMillis;
        buzzerSound(300);
       Serial.println("Overcurrent detected! System is shutting down..."); 
        }
      }  
   else{
      Serial.println("No known current problem");
     }
   //Medium Priority Case: 
    if(sporiousBreath || overTidalVolume){
       if(currentMillis - previousMillis[1] >= period_alarm[1] ){
        previousMillis[1] = currentMillis;
        buzzerSound(300);
      if(sporiousBreath){ 
       Serial.println("Sporious Breath");}
      else if(overTidalVolume){
       Serial.println("Over Tidal Volume");}
        } 
      }  
   //Low Priority Gas Case: Low/Over oxygen   
    if(oxygenDifference){
      if(currentMillis - previousMillis[2] >= period_alarm[2] ){
        previousMillis[2] = currentMillis;
        buzzerSound(200);
       Serial.println("Readjust Oxygen"); 
        }
      }
     else{
      Serial.println("Oxygen Safe");
      }            
  }
  else{
  digitalWrite(outPin, LOW);
  noTone(buzzer);
  }
  buttonPreviousState = buttonReading;

}

void buzzerSound(int freq){
  //unsigned long currentAlarmMillis = millis();
  //unsigned long previousAlarmMillis = 0;
  //if(power && currentAlarmMillis - previousAlarmMillis <= 5000){
  if(!buzzerState){
    buzzerState = HIGH;
    tone(buzzer, freq);
    }
  else{
    buzzerState = LOW;
    noTone(buzzer);
    }
   
}

//fungsi buat ngitung 120 detik (120000 ms)
//tiap fungsi ini dipanggil langsung ngitung sampe 120 detik (120000 ms)
