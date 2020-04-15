/* Low Cost Ventilator - ITB
 * This is the program for the main microcontroller (Arduino  \
 * Mega). Read sensors, signaling motor controller (Arduino    \
 * Nano), and communicate with HMI (Nextion).
 */

//== LIBRARIES =============================================
#include <SoftwareSerial.h>


//== GLOBAL VARIABLES ======================================
boolean status4Nano = 1;
boolean warningVolume = 0;
boolean warningPressure = 0;
int volumeTidal = 500;
float ERat = 0.2;
int respirationRate = 12;
boolean triggerInhale = 0;

SoftwareSerial Serial2(2,3);


//== MAIN SETUP ============================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(38400);
}


//== MAIN LOOP =============================================
void loop() {
  update2Nano();
}


//== FUNCTIONS =============================================

//-- Sending necessary information to Arduino Nano ---------
//-- (motor controller) through Serial2 port ---------------
void update2Nano() {
  String message = '<' + String(status4Nano) + ','
                   + String(warningVolume) + ','
                   + String(warningPressure) + ','
                   + String(volumeTidal) + ','
                   + String(ERat) + ','
                   + String(respirationRate) + ','
                   + String(triggerInhale) + '>';
  Serial2.print(message); Serial2.flush();

  // Debug message
//  Serial.print(F("Message sent to Nano:\n\t")); Serial.print(message); Serial.flush();
}

//-- Digital Filter -----------------------------------------
float signalFilter(float newImpulse) {
// Variable declaration
  byte filterOrder = 5;
  float respons = 0.0;

// Updating impulses
  for (int i = 0; i<(filterOrder-1); i++) {
    impulse[i] = impulse[i+1];
  }
  impulse[filterOrder]=newImpulse;

// Calculating response
  float weight[] = [0.1 0.2 0.3 0.2 0.1];
  for (int i=0; i<ordeFilter; i++) {
    respons += (blok[i] * weight[i]);
  }

  return respons;
}
