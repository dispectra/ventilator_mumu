#include <ArduinoSort.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;

float offset = 0;
int buffsize = 50;
unsigned long now;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
//  pinMode(6, INPUT_PULLUP);
  ads.begin();
  zeroFlowSensor();
}

void loop() {
  // put your main code here, to run repeatedly:
  
//  if(digitalRead(6) == LOW){
////    Serial.println("BEFORE =");
//    zeroFlowSensor();
////    Serial.println("AFTER");
//    Serial.println(micros()-now);
//    for(int i=0; i<buffsize; i++){
//      Serial.println(calcFlow(ads.readADC_Differential_0_1())+offset);
//    }
////  } else {
////   Serial.println(calcFlow(ads.readADC_Differential_0_1())+offset);
//  }

  int flow_read = ads.readADC_SingleEnded(0);
//  Serial.print(flow_read);
//  Serial.print("\t");
    //0b. Remove Noise Values
  double flow_val = calcFlow(flow_read)+offset;
    if(abs(flow_val) <= 1
        || abs(roundf(flow_val*100.0)/100.0) == 0.11
        || abs(roundf(flow_val*100.0)/100.0) == 0.22
        || abs(roundf(flow_val*100.0)/100.0) == 0.32
        || abs(roundf(flow_val*100.0)/100.0) == 0.43
        || abs(roundf(flow_val*100.0)/100.0) == 0.54
        || abs(roundf(flow_val*100.0)/100.0) == 0.65
        || abs(roundf(flow_val*100.0)/100.0) == 0.75
        ){flow_val=0;}
  Serial.println(flow_val);
  delay(10);
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
float calcFlow(float flow_rawq){
  float calc = (25.188*sqrt(flow_rawq)-2915.7);

  return calc;
}
