#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  // Declare an instance of the ADS1115

int16_t rawADCvalue;  // The is where we store the value we receive from the ADS1115
float scalefactor = 0.1875F; // This is the scale factor for the default +/- 6.144 Volt Range we will use
float volts = 0.0; // The result of applying the scale factor to the raw value
float oxygenpercentage = 0.0;

void setup(void)
{
  Serial.begin(9600); 
  ads.begin();
}

void loop(void)
{  

  rawADCvalue = ads.readADC_Differential_0_1(); 
  volts = (rawADCvalue * scalefactor);
  oxygenpercentage = 10*volts/6;
  
  //Serial.print("Raw ADC Value = "); 
  //Serial.print(rawADCvalue); 
  //Serial.print("\tVoltage Measured = ");
  Serial.print(volts);
  Serial.print("\t");
  Serial.println(oxygenpercentage);
  //Serial.println();
  

  delay(10);
}
