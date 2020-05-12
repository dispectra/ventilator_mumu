#include <ArduinoSort.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;
float offset;
int buffsize = 50;
unsigned long now;
float pres_raw;
float pres_val;
float pres_val2;
float last_val;
float peak = 0;
int peakCount = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(3, OUTPUT);
  pinMode(2,INPUT_PULLUP);
  ads.begin();
//  ads.setGain(GAIN_SIXTEEN);
//  zeroFlowSensor();
}
bool asd = true;

void loop() {
  // put your main code here, to run repeatedly:
  pres_raw = ads.readADC_SingleEnded(0);
  pres_val = calcFlow(pres_raw) + offset;

  Serial.println(pres_val);

  if(pres_val>peak) {
    peak = pres_val;
  } else {
    if(peak-pres_val > 10){// turun
      peakCount ++;
      peak = 0;
    }
  }

  if(peakCount>2) {
    Serial.println("FIGHTING!!");
    while(digitalRead(2) == HIGH){
      if(asd){
        Serial.print(".");
        asd = false;
      }
    }
    Serial.println("");
    peak = 0;
    peakCount = 0;
  }

  if(digitalRead(2) == LOW){//reset, perlu direset setiap satu cycle
    peak = 0;
    peakCount = 0;
    zeroFlowSensor();
    Serial.println("RESET -------------");
  }
  delay(11);
}


void zeroFlowSensor(){
  now = micros();
  //0. Create buffer for value and histogram
  float val[buffsize];
  float lastVal;
  int index_terpilih = 0;
  int mode_count = 0;
  int valcount = 0;

  //1. Ambil x data
  for(int i=0; i<buffsize; i++){
    val[i] = calcFlow(ads.readADC_SingleEnded(0))+offset;
    Serial.println(val[i]);
  }

  sortArray(val, buffsize);

  //2. create histogram
  for (int i=0; i<buffsize; i++){
    if(lastVal != val[i]){
      lastVal = val[i];
      valcount = countOccurances(val, val[i]);
      if(valcount>=mode_count){
        mode_count = valcount;
        index_terpilih = i;
      }
    }
  }

  //3. Return Mode
  offset += -1*val[index_terpilih];
  Serial.println("OFFSET : " + String(offset));
}

int countOccurances(float val[], float q){
  int count = 0;
  for(int i=0; i<buffsize; i++){
    if(val[i] == q){
      count++;
    }
  }
  return count;
}

//- Calc Flow from Callibration
float calcFlow(float pres_rawq){
  float calc =  0.3135*pres_rawq-1316.0693-860.24 +947.70;

  return calc;
}


//Define array of alarm triggers
//alarmzz[0] = High pressure exceeded PIP (HIGH)
//alarmzz[1] = Pressure too low (HIGH)
//alarmzz[2] = Patient is fighting (HIGH)
//alarmzz[3] = Overcurrent fault (HIGH)
//alarmzz[4] = Sporious breath (MEDIUM)
//alarmzz[5] = Overtidal volume (MEDIUM)
//alarmzz[6] = Low PEEP (MEDIUM)
//alarmzz[7] =
//alarmzz[8] = Low/Oversupply of Oxygen (LOW)
