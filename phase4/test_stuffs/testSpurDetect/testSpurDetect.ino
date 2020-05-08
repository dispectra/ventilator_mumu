#include <ArduinoSort.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;
float offset;
int buffsize = 5;
unsigned long now;
float flow_raw;
float flow_val;
float flow_val2;
float last_val;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(3, OUTPUT);
  pinMode(2,INPUT_PULLUP);
  ads.begin();
  ads.setGain(GAIN_SIXTEEN);
  zeroFlowSensor();
}
bool asd=false;
bool asd2 = true;

void loop() {
  // put your main code here, to run repeatedly:
flow_raw = ads.readADC_Differential_0_1();
   flow_val = calcFlow(flow_raw) + offset;
      if(abs(flow_val) <= 1
       || abs(roundf(flow_val*100.0)/100.0) == 3.24
       || abs(roundf(flow_val*100.0)/100.0) == 0.81
       || abs(roundf(flow_val*100.0)/100.0) == 4.06
       || abs(roundf(flow_val*100.0)/100.0) == 4.87
       || abs(roundf(flow_val*100.0)/100.0) == 5.68
       || abs(roundf(flow_val*100.0)/100.0) == 2.43
       || abs(roundf(flow_val*100.0)/100.0) == 1.62
       || abs(roundf(flow_val*100.0)/100.0) == 1.63
       ){flow_val=0;}

 ///  Serial.println(flow_val);
   flow_val2 = 0.5*flow_val + 0.5*last_val;
   last_val = flow_val;
   
  if(digitalRead(2) == LOW){ // Inhale
//    Serial.println("INHALE");/
    if (asd) {
    zeroFlowSensor();
    asd = false;
    }
    Serial.println(flow_val2);
//    Serial.println("BEFORE =");
//    zeroFlowSensor();/
//    Serial.println("AFTER");
    // Serial.println(micros()-now);
    // for(int i=0; i<buffsize; i++){
    //   Serial.println(calcFlow(ads.readADC_Differential_0_1())+offset);
    // }
 } 
 // Exhale
 else {
  asd = true;
 // Serial.println("EXHALE");
   

   //0b. Remove Noise Values


   Serial.println(flow_val2);
   if(flow_val2>10){
     Serial.println("SPURIOUS");
     while(digitalRead(2) == HIGH){
      if(asd2){
        Serial.print(".");
        asd2 = false;
      }
     }
   }

  }
  // delay(10);
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
    val[i] = calcFlow(ads.readADC_Differential_0_1())+offset;
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
float calcFlow(float flow_rawq){
  float calc = (90.1479*sqrt(flow_rawq)-5011.9318+35.80);

  return calc;
}
