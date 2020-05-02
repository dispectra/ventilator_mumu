#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads;  // Declare an instance of the ADS1115
int16_t rawADCPressureValue;  // The is where we store the value we receive from the ADS1115
float scalefactor = 0.1875F; // This is the scale factor for the default +/- 6.144 Volt Range we will use
float voltsPressure = 0.0;
//float voltsOxygen = 0.0;
const int button = 5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(button, INPUT_PULLUP);
  ads.begin();
}

bool test = false;
void loop() {

int stepq = 0;
  unsigned long now = millis();
  if(!digitalRead(button)){
    test = true;
  }
  
//  rawADCPressureValue = ads.readADC_SingleEnded(0);
//  voltsPressure = (rawADCPressureValue * scalefactor);

  //Serial.println(volts);

  if(test==true){
    while(millis()-now<=10000){
  //  int raw_tekanan=analogRead(A1);
  //  float raw_flow=analogRead(A0);
      rawADCPressureValue = ads.readADC_SingleEnded(0);
      voltsPressure = (rawADCPressureValue * scalefactor);
      Serial.println(rawADCPressureValue);
      stepq+=1;
      delay(20);  
    }
    Serial.println("step:" + String(stepq));
    test = false;
  }

}
