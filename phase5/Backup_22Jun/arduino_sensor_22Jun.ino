#include <Adafruit_ADS1015.h>
#include <ArduinoSort.h>
Adafruit_ADS1115 ads;

float oxy_val, flow_val, pres_val = 0;
float offset_p = 0;
float offset_f = 0;
int buffsize = 100;

unsigned long now;

void setup(){
  Serial.begin(115200);
  Serial.println("S");
  ads.begin();
//  ads.setGain(GAIN_SIXTEEN);
  delay(10);
//  zeroPresSensor();
//  zeroFlowSensor();
}

void loop() {
  now = micros();
  ads.setGain(GAIN_SIXTEEN);
  oxy_val = calcOxy(ads.readADC_Differential_0_1());
  ads.setGain(GAIN_TWOTHIRDS);
  flow_val = calcFlow(ads.readADC_SingleEnded(2));// + offset_f;
  ads.setGain(GAIN_TWOTHIRDS);
  pres_val = calcPres(ads.readADC_SingleEnded(3));// + offset_p;
//  if(
//  Serial.print("-100");
//  Serial.print("\t" + String(peakq));
//  Serial.print("\t100");
//  Serial.println("\t" + String(flow_val));
  Serial.println('{' + String(oxy_val) + ',' + String(flow_val) + ',' + String(pres_val) + ',' + String(micros()-now) + '}');
//  Serial.flush();
//  delayMicroseconds(10);
//  delay(1000);
}

void zeroFlowSensor(){
  //0. Create buffer for value and histogram
  ads.setGain(GAIN_TWOTHIRDS);
  float val[buffsize];
  float lastVal;
  int index_terpilih = 0;
  int mode_count = 0;
  int valcount = 0;

  //1. Ambil x data
  for(int i=0; i<buffsize; i++) {
    val[i] = calcFlow(ads.readADC_SingleEnded(2))+offset_f;
  }

  sortArray(val, buffsize);

  //2. create histogram
  for (int i=0; i<buffsize; i++) {
    if(lastVal != val[i]) {
      lastVal = val[i];
      valcount = countOccurances(val, val[i]);
      if(valcount>=mode_count) {
        mode_count = valcount;
        index_terpilih = i;
      }
    }
  }

  //3. Return Mode
  offset_f += -1*val[index_terpilih];
//  Serial.println("OFFSET : " + String(offset));
}

void zeroPresSensor(){
  //0. Create buffer for value and histogram
  ads.setGain(GAIN_TWOTHIRDS);
  float val[buffsize];
  float lastVal;
  int index_terpilih = 0;
  int mode_count = 0;
  int valcount = 0;

  //1. Ambil x data
  float sumq = 0;
  for(int i=0; i<buffsize; i++) {
    val[i] = calcPres(ads.readADC_SingleEnded(3))+offset_p;
    sumq+=val[i];
  }
  
  sortArray(val, buffsize);

  //2. create histogram
  for (int i=0; i<buffsize; i++) {
    if(lastVal != val[i]) {
      lastVal = val[i];
      valcount = countOccurances(val, val[i]);
      if(valcount>=mode_count) {
        mode_count = valcount;
        index_terpilih = i;
      }
    }
  }

  //3. Return Mode
  offset_p += -1*val[index_terpilih];
//  offset_p += -1*sumq/buffsize;
//  Serial.println("OFFSET : " + String(offset));
}

int countOccurances(float val[], float q){
  int count = 0;
  for(int i=0; i<buffsize; i++) {
    if(val[i] == q) {
      count++;
    }
  }
  return count;
}


//---- Press
float calcPres(float pres_rawq){
  float calc = 0.3135*pres_rawq-1316.0693-3.14;

  return calc;
}

//---- Flow
float calcFlow(float flow_rawq){
  float calc = 25.188*sqrt(flow_rawq)-2915.7;

  return calc;
}

//---- Oxy
float calcOxy(float oxy_rawq){
  float calc = oxy_rawq *0.256 / 32768*1000*10/6;

  return calc;
}