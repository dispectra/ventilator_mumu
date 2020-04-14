// Low-Cost Ventilator
// ----
// Updated 7 April 2020 11.38 PM
// Based on nextion_baru and RevisiStepper4


#include <Nextion.h>

/////////////////////////////////// BREATHING PART //////////////////////////////////

//-- KONFIGURASI PIN DRIVER =========================================================
// ENA- NC
// ENA+ NC
// DIR- GND
// DIR+ dirPin
// PUL- GND
// PUL+ stepPin

//-- TWEAKABLES =====================================================================
// dirPin = pin DIR+
// stepPin = pin PUL+
// microstepping = settingan microstepping (1 / 2 / 4 / 8 / 16)
// dirInhale = arah untuk inhale (HIGH / LOW)
#define microstepping 4
#define dirInhale LOW

//-- PIN FISIK =======================================================================
#define dirPin 2
#define stepPin 3
#define limitSwitchIn 4
#define limitSwitchEx 5
#define calManMaju 6
#define calManMundur 7
#define LEDBreath 10
#define LEDCallibrate 11

bool callibrated = false;
/////////////////////////////////////////////////////////////////////////////////////

int CurrentPage = 1;
uint32_t state = 0;  // Create variable to store Running state
uint32_t IE = 0;  // Create variable to store value of IE
uint32_t RR = 10;  // Create variable to store value of IE
uint32_t PEEP = 0;  // Create variable to store value of IE
uint32_t Vtj = 100;  // Create variable to store value of IE
//char buffer[100] = {0};
// Format: <type of object> <object name> = <type of object>(<page id>, <object id>, "<object name>");

//Page 1
NexButton b0 = NexButton(1, 2, "b0");  // Button added
NexButton b1 = NexButton(1, 1, "b1");  // Button added
NexDSButton bt0 = NexDSButton(1, 3, "bt0");  // Dual state button added
NexText t1 = NexText(1, 4, "t1");  // Text box added, so we can read it
//Page 2
NexButton b3 = NexButton(2, 11, "b3");  // Button added
NexButton b5 = NexButton(2, 12, "b5");  // Button added
NexDSButton bt1 = NexDSButton(2, 9, "bt1");  // Dual state button added
//NexText t1 = NexText(2, 14, "t1");  // Text box added, so we can read it
NexButton b10 = NexButton(2, 13, "b10");  // Button added
NexButton b11 = NexButton(2, 14, "b11");  // Button added
NexButton b2 = NexButton(2, 15, "b2");  // Button added
NexButton b4 = NexButton(2, 16, "b4");  // Button added
NexButton b6 = NexButton(2, 18, "b6");  // Button added
NexButton b7 = NexButton(2, 19, "b7");  // Button added
NexButton b8 = NexButton(2, 22, "b8");  // Button added
NexButton b9 = NexButton(2, 23, "b9");  // Button added
//
NexPage page0 = NexPage(0, 0, "page0");  // Page added as a touch event
NexPage page1 = NexPage(1, 0, "page1");  // Page added as a touch event
NexPage page2 = NexPage(2, 0, "page2");  // Page added as a touch event
//
char buffer[100] = {0};
//
NexTouch *nex_listen_list[] = 
{
  &b0,&b1,&b2,&b3,&b4,&b5,&b6,
  &b7,&b8,&b9,&b10,&b11,
  &bt0,&bt1,
  //&h0,&h1,&h2,&h3,  // Slider added
  &page0,&page1,&page2,  // Page added as a touch event
  NULL  // String terminated
}; 
//
void bt0PushCallback(void *ptr)  // Using dual state button as Running State
{
  //uint32_t state;
  bt0.getValue(&state);
  if(state == 1) {digitalWrite(LEDBreath, HIGH);}  // Control Action when ON}
  else {digitalWrite(LEDBreath, LOW);}  // Control Action when OFF}
} 
//
void bt1PushCallback(void *ptr)  // Using dual state button as Running State
{
  //uint32_t state; 
  bt1.getValue(&state);  // Read value of dual state button to know the state (0 or 1)
  if(state == 1) {digitalWrite(LEDBreath, HIGH);}  // Control Action when ON}
  else {digitalWrite(LEDBreath, LOW);}  // Control Action when OFF}
}
//
void b10PushCallback(void *ptr)  // Press event for button b0
{
  IE++;
  if(IE>=40)
  {IE=40;}
} 
void b11PushCallback(void *ptr)  // Press event for button b0
{
  IE--; 
  if(IE==4294967295){IE=0;}
}
void b2PushCallback(void *ptr)  // Press event for button b0
{
  RR++;
  if(RR>=60)
  {RR=60;}
} 
void b4PushCallback(void *ptr)  // Press event for button b0
{
  RR--; 
  if(RR<=10)
  {RR=10;}
}
void b6PushCallback(void *ptr)  // Press event for button b0
{
  PEEP++;
  if(PEEP>=80)
  {PEEP=80;}
} 
void b7PushCallback(void *ptr)  // Press event for button b0
{
  PEEP--; 
  if(PEEP==4294967295)
  {PEEP=0;}
}
void b8PushCallback(void *ptr)  // Press event for button b0
{
  Vtj=Vtj+10;
  if(Vtj>=900)
  {Vtj=900;}
} 
void b9PushCallback(void *ptr)  // Press event for button b0
{
  Vtj=Vtj-10; 
  if(Vtj<=100)
  {Vtj=100;}
}
//
void page0PushCallback(void *ptr)  // If page 0 is loaded on the display, the following is going to execute:
{
  CurrentPage = 0;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
  dbSerialPrintln(CurrentPage);
}
//
void page1PushCallback(void *ptr)  // If page 1 is loaded on the display, the following is going to execute:
{
  CurrentPage = 1;  // Set variable as 1 so from now on arduino knows page 1 is loaded on the display
  dbSerialPrintln(CurrentPage);
}
//
void page2PushCallback(void *ptr)  // If page 2 is loaded on the display, the following is going to execute:
{
  CurrentPage = 2;  // Set variable as 2 so from now on arduino knows page 2 is loaded on the display
  dbSerialPrintln(CurrentPage);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {  // Put your setup code here, to run once:
  
  //Serial.begin(9600);  // Start serial comunication at baud=9600
  //Serial2.begin(9600);
  nexInit();
  // Register the event callback functions of each touch event:
  // You need to register press events and release events seperatly.
  // Format for press events: <object name>.attachPush(<object name>PushCallback);
  // Format for release events: <object name>.attachPop(<object name>PopCallback);
  bt0.attachPush(bt0PushCallback, &bt0);  // Dual state button bt0 press
  bt1.attachPush(bt1PushCallback, &bt1);  // Dual state button bt1 press
  b2.attachPush(b2PushCallback, &b2);
  b4.attachPush(b4PushCallback, &b4);
  b6.attachPush(b6PushCallback, &b6);
  b7.attachPush(b7PushCallback, &b7);
  b8.attachPush(b8PushCallback, &b8);
  b9.attachPush(b9PushCallback, &b9);
  b10.attachPush(b10PushCallback, &b10);
  b11.attachPush(b11PushCallback, &b11);
  page0.attachPush(page0PushCallback, &page0);  // Page press event
  page1.attachPush(page1PushCallback, &page1);  // Page press event
  page2.attachPush(page2PushCallback, &page2);  // Page press event
  // End of registering the event callback functions

  pinMode(LEDBreath, OUTPUT);
  pinMode(LEDCallibrate, OUTPUT);
  //digitalWrite(2, HIGH);
  
  //////////// BREATHING PART //////////////////
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(limitSwitchIn, INPUT_PULLUP);
  pinMode(limitSwitchEx, INPUT_PULLUP);

  pinMode(calManMaju, INPUT_PULLUP);
  pinMode(calManMundur, INPUT_PULLUP);

  Serial.println("==> CALLIBRATING");
  Callibrate();
  Serial.println("==> CALLIBRATION DONE");
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{  
  int mode = 0;

  if(digitalRead(calManMaju) == LOW){
    digitalWrite(dirPin, dirInhale);
    delayMicroseconds(5);
    
    if(digitalRead(limitSwitchIn)){
      digitalWrite(stepPin, HIGH);
    }
    delayMicroseconds(1000);
    if(digitalRead(limitSwitchIn)){
      digitalWrite(stepPin, LOW);
    }
    delayMicroseconds(1000);
  } else if(digitalRead(calManMundur) == LOW){
    digitalWrite(dirPin, !dirInhale);
    delayMicroseconds(5);
    
    if(digitalRead(limitSwitchEx)){
      digitalWrite(stepPin, HIGH);
    }
    delayMicroseconds(1000);
    if(digitalRead(limitSwitchEx)){
      digitalWrite(stepPin, LOW);
    }
    delayMicroseconds(1000);
  }
  
  if(state==0&&CurrentPage==1){mode=0;}
  if(state==0&&CurrentPage==2){mode=1;}
  if(state==1&&CurrentPage==1){mode=2;}
  if(state==1&&CurrentPage==2){mode=3;}

  dbSerialPrintln("IE = 1:" + String(IE));
  dbSerialPrintln("RR" + String(RR));
  dbSerialPrintln("Vtidal" + String(Vtj));
  dbSerialPrintln("PEEP" + String(PEEP));
  
  while(mode==0)
  {
    nexLoop(nex_listen_list);
    //bt0.getValue(&state);
    if(state==1){break;}
    if(CurrentPage==2){break;}
    dbSerialPrintln("mode 0");

    if (!callibrated) {
      Callibrate();
    }
  }
  while(mode==1)
  {
    nexLoop(nex_listen_list);
    //bt1.getValue(&state);
    if(state==1){break;}
    if(CurrentPage==1){break;}
    dbSerialPrintln("mode 1");

    if (!callibrated) {
      Callibrate();
    }
    
  }
  while(mode==2)
  {
    nexLoop(nex_listen_list);
   
    //bt0.getValue(&state);
    if(state==0){callibrated = false; break;}
    if(CurrentPage==2){break;}
    dbSerialPrintln("mode 2");
    
    oneBreathCycle(Vtj, IE, RR);
  }
  while(mode==3)
  {
    nexLoop(nex_listen_list);
    
    //bt1.getValue(&state);
    if(state==0){callibrated = false; break;}
    if(CurrentPage==1){break;}
    dbSerialPrintln("mode 3");

    oneBreathCycle(Vtj, IE, RR);
  }
}



//////////////////////////////////// BREATHING PART ///////////////////////////////////

//-- Fungsi loop nafas
void oneBreathCycle(float volTidal, float IERat, int RR){
  unsigned long stepTidal, delayInhale, delayExhale, timeInEx;
  float timeInhale, timeExhale, IERatio, timeBreath, slopeFactor;
  
  int IRat = 1;
  float ERat = IERat/10;
  float initDelay = 500;

  slopeFactor = 0.5;
  stepTidal = cekTidal(volTidal);
  timeBreath = (60000 / float(RR)) * 1000;
  timeInhale = (60000 / float(RR)) * (float(IRat) / float(IRat + ERat)) * 1000; // dalam miliseconds
  timeExhale = (60000 / float(RR)) * (float(ERat) / float(IRat + ERat)) * 1000; // dalam miliseconds
  delayInhale = 220; // dalam microseconds
  delayExhale = delayInhale; // dalam microseconds

  Serial.println("==================");
  Serial.println("Vol Tidal = " + String(volTidal));
  Serial.println("Step Tidal = " + String(stepTidal));
  Serial.println("Slope Tidal = " + String(slopeFactor));
  Serial.println("----");
  Serial.println("DELAY Awal = " + String(initDelay));
  Serial.println("DELAY Inhale = " + String(delayInhale));
  Serial.println("DELAY Exhale = " + String(delayExhale));
  Serial.println("----");
  Serial.println("WAKTU BREATH = " + String(timeBreath));
  Serial.println("WAKTU IDEAL Inhale = " + String(timeInhale));
  Serial.println("WAKTU IDEAL Exhale = " + String(timeExhale));
  Serial.println("----");

  if (stepTidal > 0) {
    unsigned long now = micros();
    
    Inhale(stepTidal, delayInhale, slopeFactor, initDelay);
    
    while((micros()-now) < timeInhale){delayMicroseconds(1);}
    unsigned long timeInhaleReal = micros()-now;
    Serial.println("==> TIME INHALE : " + String(timeInhaleReal));
      
    Exhale(stepTidal, delayExhale, slopeFactor, initDelay);

    while((micros()-now) < timeBreath){delayMicroseconds(1);}
    Serial.println("==> TIME EXHALE : " + String(micros()-now - timeInhaleReal));
    
    Serial.println("TIME TAKEN : " + String(micros() - now));
    Serial.println("----");
    
  } else {
    // Error Message
  }
}


//-- Fungsi Kalibrasi
void Callibrate() {
  digitalWrite(LEDCallibrate, HIGH);
  digitalWrite(dirPin, !dirInhale); 
  unsigned long now = millis();
  while(digitalRead(limitSwitchEx)){
    if (millis() - now > 2000){break;} // 2 detik maks kalibrasi
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  callibrated = true;
  digitalWrite(LEDCallibrate, LOW);
}

//-- Lookup Table Volume Tidal vs Step yang diperlukan ================================
float cekTidal(float vol_Tidal){
  float lookup_vol[] = {500, 750, 1000, 1250};
  float lookup_step[] = {480, 550, 660, 700};

  float stepTidal = 0;
  int arraySize = sizeof(lookup_vol) / sizeof(lookup_vol[0]);

  // Extrapolasi Bawah
  if(vol_Tidal < cariMin(lookup_vol, arraySize)){
    float m = float(lookup_step[1] - lookup_step[0]) / (lookup_vol[1] - lookup_vol[0]);
    float c = float(lookup_step[0]) - lookup_vol[0] * m;
    stepTidal = m * vol_Tidal + c;
  } 
  // Extrapolasi Atas
  else if(vol_Tidal > cariMax(lookup_vol, arraySize)){
    float m = float(lookup_step[arraySize-1] - lookup_step[arraySize-2]) / (lookup_vol[arraySize-1] - lookup_vol[arraySize-2]);
    float c = float(lookup_step[arraySize-1]) - lookup_vol[arraySize-1] * m;
    stepTidal = m * vol_Tidal + c;
  } 
  // Normal + Interpolasi 
  else {
    for(int i = 0; i< arraySize; i++) {
      if (vol_Tidal == lookup_vol[i]) {
        stepTidal = lookup_step[i];
      } else {
        if(vol_Tidal >= lookup_vol[i] && vol_Tidal < lookup_vol[i+1]) {
          stepTidal = lookup_step[i] + float(lookup_step[i+1] - lookup_step[i]) * float(vol_Tidal - lookup_vol[i]) / float(lookup_vol[i+1]-lookup_vol[i]);
          break;
        }
      }
    }
  }
  return stepTidal;
}

//-- fungsi tambahan lookup table
float cariMin(float list[], int arraySize){
  float mini = 99999;
  for(int i=0; i< arraySize; i++){
    if(list[i] <= mini) {
      mini = list[i];
    }
  }
  return mini;
}

float cariMax(float list[], int arraySize){
  float maxi = 0;
  for(int i=0; i< arraySize; i++){
    if(list[i] >= maxi) {
      maxi = list[i];
    }
  }
  return maxi;
}


//-- Sekuens Inhale ====================================================================
void Inhale(int stepTidal, unsigned long delayInhale, float slopeFactor, float initDelay) {
  // 0. Init Variables
  unsigned long now = micros();
  float delayInhale2 = initDelay;
  
  // 1. Set Arah
  digitalWrite(dirPin, dirInhale); 
  delayMicroseconds(5);

  // 2. Set Gerakan Stepper
  if (delayInhale2 > 16383){
    for(int i = 0; i < stepTidal; i++) {
      if(i < slopeFactor*stepTidal){
        delayInhale2 -= (initDelay-delayInhale) / (slopeFactor*stepTidal);
      }
      if(i>(1-slopeFactor)*stepTidal){
        delayInhale2 += (initDelay-delayInhale) / (slopeFactor*stepTidal);
      }
      
      if(digitalRead(limitSwitchIn)){
        digitalWrite(stepPin, HIGH);
      }
      
      for(int j=0; j < delayInhale2/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(int(delayInhale2) % 16383);

      if(digitalRead(limitSwitchIn)){
        digitalWrite(stepPin, LOW);
      }
      
      for(int j=0; j < delayInhale2/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(int(delayInhale2) % 16383);
    }
  } else {
    for(int i = 0; i < stepTidal; i++) {
      if(i < slopeFactor*stepTidal){
        delayInhale2 -= (initDelay-delayInhale) / (slopeFactor*stepTidal);
      }
      if(i>(1-slopeFactor)*stepTidal){
        delayInhale2 += (initDelay-delayInhale) / (slopeFactor*stepTidal);
      }  
    
      if(digitalRead(limitSwitchIn)){
        digitalWrite(stepPin, HIGH);
      }
      
      delayMicroseconds(delayInhale2);
      
      if(digitalRead(limitSwitchIn)){
        digitalWrite(stepPin, LOW);
      }
      
      delayMicroseconds(delayInhale2);
    }
  }

  // 3. Tampil Waktu
  Serial.print("Waktu Inhale = ");
  Serial.println(micros() - now);
}

//-- Sekuens Exhale ====================================================================
void Exhale(int stepTidal, unsigned long delayExhale, float slopeFactor, float initDelay) {
  // 0. Init Variables
  unsigned long now = micros();
  float delayExhale2 = initDelay;
  
  // 1. Set Arah
  digitalWrite(dirPin, !dirInhale); 
  delayMicroseconds(5);

  // 2. Set Gerakan Stepper
  if (delayExhale2 > 16383){
    for(int i = 0; i < stepTidal; i++) {
      if(i < slopeFactor*stepTidal){
        delayExhale2 -= (initDelay-delayExhale) / (slopeFactor*stepTidal);
      }
      if(i>(1-slopeFactor)*stepTidal){
        delayExhale2 += (initDelay-delayExhale) / (slopeFactor*stepTidal);
      }
      
      if(digitalRead(limitSwitchEx)){
        digitalWrite(stepPin, HIGH);
      }
      
      for(int j=0; j < delayExhale2/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(int(delayExhale2) % 16383);

      if(digitalRead(limitSwitchEx)){
        digitalWrite(stepPin, LOW);
      }
      
      for(int j=0; j < delayExhale2/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(int(delayExhale2) % 16383);
    }
  } else {
    for(int i = 0; i < stepTidal; i++) {
      if(i < slopeFactor*stepTidal){
        delayExhale2 -= (initDelay-delayExhale) / (slopeFactor*stepTidal);
      }
      if(i>(1-slopeFactor)*stepTidal){
        delayExhale2 += (initDelay-delayExhale) / (slopeFactor*stepTidal);
      }  
    
      if(digitalRead(limitSwitchEx)){
        digitalWrite(stepPin, HIGH);
      }
      
      delayMicroseconds(delayExhale2);
      
      if(digitalRead(limitSwitchEx)){
        digitalWrite(stepPin, LOW);
      }
      
      delayMicroseconds(delayExhale2);
    }
  }

  // 3. Tampil Waktu
  Serial.print("Waktu Exhale = ");
  Serial.println(micros() - now);
}
