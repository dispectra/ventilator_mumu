//Low-Cost Multi-User Ventilator
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
#define dirPin 2
#define stepPin 3
#define microstepping 4
#define dirInhale HIGH

//-- PIN FISIK =======================================================================
#define limitSwitchIn 4
#define limitSwitchEx 5
#define LEDBreath 10
#define LEDCallibrate 11

bool callibrated = false;
/////////////////////////////////////////////////////////////////////////////////////

int CurrentPage = 1;
uint32_t state = 0;  // Create variable to store Running state
uint32_t IE = 0;  // Create variable to store value of IE
uint32_t RR = 0;  // Create variable to store value of IE
uint32_t PEEP = 0;  // Create variable to store value of IE
uint32_t Vtj = 0;  // Create variable to store value of IE
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
NexDSButton bt1 = NexDSButton(2, 13, "bt1");  // Dual state button added
//NexText t1 = NexText(2, 14, "t1");  // Text box added, so we can read it
NexSlider h0 = NexSlider(2, 1, "h0");  // Slider added
NexSlider h1 = NexSlider(2, 2, "h1");  // Slider added
NexSlider h2 = NexSlider(2, 3, "h2");  // Slider added
NexSlider h3 = NexSlider(2, 4, "h3");  // Slider added
NexNumber n0 = NexNumber(2, 5, "n0");  // Number
NexNumber n2 = NexNumber(2, 8, "n2");  // Number
NexNumber n3 = NexNumber(2, 9, "n3");  // Number
NexNumber n4 = NexNumber(2, 10, "n4");  // Number
//
NexPage page0 = NexPage(0, 0, "page0");  // Page added as a touch event
NexPage page1 = NexPage(1, 0, "page1");  // Page added as a touch event
NexPage page2 = NexPage(2, 0, "page2");  // Page added as a touch event
//
char buffer[100] = {0};
//
NexTouch *nex_listen_list[] = 
{
  &b0,  // Button added
  &b1,  // Button added
  &b3,  // Button added
  &b5,  // Button added
  &bt0,  // Dual state button added
  &bt1,  // Dual state button added
  &h0,  // Slider added
  &h1,  // Slider added
  &h2,  // Slider added
  &h3,  // Slider added
  &page0,  // Page added as a touch event
  &page1,  // Page added as a touch event
  &page2,  // Page added as a touch event
  NULL  // String terminated
}; 
//
void bt0PushCallback(void *ptr)  // Using dual state button as Running State
{
  //uint32_t state;  // Create variable to store Running state
  //NexDSButton *btn = (NexDSButton *)ptr;
  //dbSerialPrintln("Callback");
  //dbSerialPrint("ptr=");
  //bt0.getValue(&state);  // Read value of dual state button to know the state (0 or 1)
  //dbSerialPrintln((uint32_t)ptr);
  //memset(buffer, 0, sizeof(buffer));
  bt0.getValue(&state);
  if(state == 1)
  {  
    digitalWrite(LEDBreath, HIGH);  // Control Action when ON
  }
  else
  {  
    digitalWrite(LEDBreath, LOW);  // Control Action when OFF
  }
} 
//
void bt1PushCallback(void *ptr)  // Using dual state button as Running State
{
  //uint32_t state; 
  bt1.getValue(&state);  // Read value of dual state button to know the state (0 or 1)

  if(state == 1)
  {  
    digitalWrite(LEDBreath, HIGH);  // Control Action when ON
  }
  else
  {  
    digitalWrite(LEDBreath, LOW);  // Control Action when OFF
  }
}
//
void h0PopCallback(void *ptr)  // Press event for IE slider
{
  //uint32_t IE; 
  h0.getValue(&IE);  // Read the value of the IE
  //dbSerialPrintln(IE);
  // The "Are you sure is 0?" begins:
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  if(IE==0){  // If I got a 0, then recheck:
    h0.getValue(&IE);  // Read the value of the slider
  }
  dbSerialPrintln(IE);
  //return IE;
}
//
void h1PopCallback(void *ptr)  // Press event for RR slider
{
  //uint32_t RR; 
  h1.getValue(&RR);  // Read the value of the RR
  dbSerialPrintln(RR);
}
//
void h2PopCallback(void *ptr)  // Press event for PEEP slider
{
  //uint32_t PEEP; 
  h2.getValue(&PEEP);  // Read the value of the PEEP
  dbSerialPrintln(PEEP);
}
//
void h3PopCallback(void *ptr)  // Press event for Vtj slider
{
  //uint32_t Vtj; 
  h3.getValue(&Vtj);  // Read the value of the Vtj
  dbSerialPrintln(Vtj);
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
  
//  Serial.begin(9600);  // Start serial comunication at baud=9600
//  Serial2.begin(9600);//
  nexInit();
  // Register the event callback functions of each touch event:
  // You need to register press events and release events seperatly.
  // Format for press events: <object name>.attachPush(<object name>PushCallback);
  // Format for release events: <object name>.attachPop(<object name>PopCallback);
  bt0.attachPush(bt0PushCallback, &bt0);  // Dual state button bt0 press
  bt1.attachPush(bt1PushCallback, &bt1);  // Dual state button bt1 press
  h0.attachPop(h0PopCallback, &h0);  // Slider press
  h1.attachPop(h1PopCallback, &h1);  // Slider press
  h2.attachPop(h2PopCallback, &h2);  // Slider press
  h3.attachPop(h3PopCallback, &h3);  // Slider press
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

  Serial.println("==> CALLIBRATING");
  Callibrate();
  Serial.println("==> CALLIBRATION DONE");
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{  
  int mode = 0;
  
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
  unsigned long stepTidal, delayInhale, delayExhale;
  float timeInhale, timeExhale, IERatio;
  int IRat = 1;
  float ERat = IERat/10;
  stepTidal = cekTidal(volTidal);
  timeInhale = (60000 / float(RR)) * (float(IRat) / float(IRat + ERat)); // dalam miliseconds
  timeExhale = (60000 / float(RR)) * (float(ERat) / float(IRat + ERat)); // dalam miliseconds
  delayInhale = float(timeInhale*1000) / float(stepTidal) / 2; // dalam microseconds
  delayExhale = float(timeExhale*1000) / float(stepTidal) / 2; // dalam microseconds

  if (stepTidal > 0) {
    Inhale(stepTidal, delayInhale);
    Exhale(stepTidal, delayExhale);
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
    delayMicroseconds(200);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(200);
  }
  callibrated = true;
  digitalWrite(LEDCallibrate, LOW);
}
//-- Lookup Table Volume Tidal vs Step yang diperlukan ================================
int cekTidal(float vol_Tidal){
  float lookup_vol[] = {6, 7, 8};
  int lookup_step[] = {2500, 8000, 10000};

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
void Inhale(int stepTidal, int delayInhale) {
  // 0. Hitung Waktu
  unsigned long now = micros();
  
  // 1. Set Arah
  digitalWrite(dirPin, dirInhale); 

  // 2. Set Gerakan Stepper
  if (delayInhale > 16383){
    for(int i = 0; i < stepTidal; i++) {
      if(digitalRead(limitSwitchIn)){
        digitalWrite(stepPin, HIGH);
      }
      
      for(int j=0; j < delayInhale/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(delayInhale % 16383);

      if(digitalRead(limitSwitchIn)){
        digitalWrite(stepPin, LOW);
      }
      
      for(int j=0; j < delayInhale/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(delayInhale % 16383);
    }
  } else {
    for(int i = 0; i < stepTidal; i++) {
      if(digitalRead(limitSwitchIn)){
        digitalWrite(stepPin, HIGH);
      }
      
      delayMicroseconds(delayInhale);
      
      if(digitalRead(limitSwitchIn)){
        digitalWrite(stepPin, LOW);
      }
      
      delayMicroseconds(delayInhale);
    }
  }

  // 3. Tampil Waktu
  Serial.print("Waktu Inhale = ");
  Serial.println(micros() - now);
}

//-- Sekuens Exhale ====================================================================
void Exhale(int stepTidal, int delayExhale) {
  // 0. Hitung Waktu
  unsigned long now = micros();
  
  // 1. Set Arah
  digitalWrite(dirPin, !dirInhale); 

  // 2. Set Gerakan Stepper
  if (delayExhale > 16383){
    for(int i = 0; i < stepTidal; i++) {
      if(digitalRead(limitSwitchEx)){
        digitalWrite(stepPin, HIGH);
      }
      
      for(int j=0; j < delayExhale/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(delayExhale % 16383);

      if(digitalRead(limitSwitchEx)){
        digitalWrite(stepPin, LOW);
      }
      
      for(int j=0; j < delayExhale/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(delayExhale % 16383);
    }
  } else {
    for(int i = 0; i < stepTidal; i++) {
      if(digitalRead(limitSwitchEx)){
        digitalWrite(stepPin, HIGH);
      }
      
      delayMicroseconds(delayExhale);
      
      if(digitalRead(limitSwitchEx)){
        digitalWrite(stepPin, LOW);
      }
      
      delayMicroseconds(delayExhale);
    }
  }
  
  // 3. Tampil Waktu
  Serial.print("Waktu Exhale = ");
  Serial.println(micros() - now);
}
