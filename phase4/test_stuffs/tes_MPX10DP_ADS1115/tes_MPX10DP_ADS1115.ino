#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  // Declare an instance of the ADS1115

int16_t rawADCvalue01, rawADCvalue2;  // The is where we store the value we receive from the ADS1115
//float scalefactor = 0.1875F; // This is the scale factor for the default +/- 6.144 Volt Range we will use
float scalefactor = 0.0078125F;
float volts = 0.0; // The result of applying the scale factor to the raw value
float pressure;

void setup(void)
{
  Serial.begin(115200);
  ads.begin();
  ads.setGain(GAIN_SIXTEEN); // 16x gain +/- 0.256V 1 bit = 0.125mV
//  Serial.println("Raw01,Volt01(mV),Pressure(cmH2O)");
}

void loop(void)
{  

  rawADCvalue01 = ads.readADC_Differential_0_1(); 
  rawADCvalue2 = ads.readADC_SingleEnded(2);
  volts = (rawADCvalue01 * scalefactor);
//  pressure = mapFloat(volts,0,35,0,10000);  //in Pa
  pressure = mapFloat(volts,0,35,0,101.972); //in cmH2O

  Serial.print(rawADCvalue01);
//  Serial.print(" ");
//  Serial.print(rawADCvalue2);
//  Serial.print(" ");
//  Serial.print(volts);
//  Serial.print(" ");
//  Serial.print(pressure);
  Serial.println();  

  delay(10);
}

float mapFloat(float rawX, float rawA, float rawB, float realA, float realB) {
  float realX = ( (realB - realA) * ((rawX - rawA) / (rawB - rawA)) ) + realA;
  return realX;
}
