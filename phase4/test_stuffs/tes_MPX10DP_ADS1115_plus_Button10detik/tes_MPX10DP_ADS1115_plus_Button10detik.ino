#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  // Declare an instance of the ADS1115

int16_t rawADCvalue01, rawADCvalue2;  // The is where we store the value we receive from the ADS1115
//float scalefactor = 0.1875F; // This is the scale factor for the default +/- 6.144 Volt Range we will use
float scalefactor = 0.0078125F;
float volts = 0.0; // The result of applying the scale factor to the raw value
float pressure;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(7, INPUT_PULLUP);
  ads.begin();
  ads.setGain(GAIN_SIXTEEN); // 16x gain +/- 0.256V 1 bit = 0.125mV
//  Serial.println("Raw01,Volt01(mV),Pressure(cmH2O)");
}
bool test = false;
void loop() {
  int stepq = 0;
  unsigned long now = millis();
  if(!digitalRead(7)){
    test = true;
  }
  
  if(test==true){
    while(millis()-now<=10000){
  //  int raw_tekanan=analogRead(A1);
//      float raw_flow=analogRead(A0);
  //  Serial.print(raw_tekanan);Serial.print("\t");
//      Serial.println(analogRead(A0));  

      rawADCvalue01 = ads.readADC_Differential_0_1();   
//      volts = (rawADCvalue01 * scalefactor);
//      pressure = mapFloat(volts,0,35,0,101.972); //in cmH2O
      Serial.println(rawADCvalue01);
      stepq+=1;
      delay(20);  
    }
    Serial.println("step:" + String(stepq));
    test = false;
  }
  
}

float mapFloat(float rawX, float rawA, float rawB, float realA, float realB) {
  float realX = ( (realB - realA) * ((rawX - rawA) / (rawB - rawA)) ) + realA;
  return realX;
}
