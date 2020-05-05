#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  // Declare an instance of the ADS1115

int16_t rawADCvalue01, rawADCvalue2;  // The is where we store the value we receive from the ADS1115
//float scalefactor = 0.1875F; // This is the scale factor for the default +/- 6.144 Volt Range we will use
float scalefactor = 0.0078125F;
float volts = 0.0; // The result of applying the scale factor to the raw value
float pressure;

// Filter pressure
float p_impulse[] = {0.0, 0.0};
float p_response() {
  float response;
  response = 0.5*p_impulse[0] + 0.5*p_impulse[1];
  return response;
}

void setup(void)
{
  Serial.begin(38400);
  ads.begin();
  ads.setGain(GAIN_SIXTEEN); // 16x gain +/- 0.256V 1 bit = 0.125mV
//  Serial.println("Raw01,Volt01(mV),Pressure(cmH2O)");

  Serial.println("CLEARDATA");
  Serial.println("LABEL,Time,Timer,RawADC,RawVolt,Pressure(cmH2O");
  Serial.println("RESETTIMER");
}

void loop(void)
{  

  rawADCvalue01 = ads.readADC_Differential_0_1(); 
  rawADCvalue2 = ads.readADC_SingleEnded(2);
  volts = (rawADCvalue01 * scalefactor);
//  pressure = mapFloat(volts,0,35,0,10000);  //in Pa
//  pressure = mapFloat(volts,0,35,0,101.972); //in cmH2O
  pressure = mapFloat(sqrt(rawADCvalue01),0,100,-4852.5,4162.4) - 109; //sebenernya flow

//  Serial.print(rawADCvalue01);
//  Serial.print(" ");
//  Serial.print(rawADCvalue2);
//  Serial.print(" ");
//  Serial.print(volts);
//  Serial.print(" ");
//  Serial.print(pressure);
//  Serial.println();  

//- PLX-DAQ format
//  Serial.print("DATA,TIME,TIMER,");
//  Serial.print(rawADCvalue01); Serial.print(",");
//  Serial.print(volts); Serial.print(",");
  Serial.println(pressure);

  delay(1);
}

float mapFloat(float rawX, float rawA, float rawB, float realA, float realB) {
  float realX = ( (realB - realA) * ((rawX - rawA) / (rawB - rawA)) ) + realA;
  return realX;
}
