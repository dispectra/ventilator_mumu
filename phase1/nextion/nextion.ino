//Low-Cost Multi-User Ventilator
#include <Nextion.h>
 
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
    digitalWrite(2, HIGH);  // Control Action when ON
  }
  else
  {  
    digitalWrite(2, LOW);  // Control Action when OFF
  }
} 
//
void bt1PushCallback(void *ptr)  // Using dual state button as Running State
{
  //uint32_t state; 
  bt1.getValue(&state);  // Read value of dual state button to know the state (0 or 1)

  if(state == 1)
  {  
    digitalWrite(2, HIGH);  // Control Action when ON
  }
  else
  {  
    digitalWrite(2, LOW);  // Control Action when OFF
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
  
  //Serial.begin(9600);  // Start serial comunication at baud=9600
  //Serial2.begin(9600);
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

  pinMode(2, OUTPUT);
  //digitalWrite(2, HIGH);
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{  // Put your main code here, to run repeatedly:
  int mode = 0;
  if(state==0&&CurrentPage==1){mode=0;}
  if(state==0&&CurrentPage==2){mode=1;}
  if(state==1&&CurrentPage==1){mode=2;}
  if(state==1&&CurrentPage==2){mode=3;}
  while(mode==0)
  {
    nexLoop(nex_listen_list);
    //bt0.getValue(&state);
    if(state==1){break;}
    if(CurrentPage==2){break;}
    dbSerialPrintln("mode 0");
    dbSerialPrintln(IE);
    //delay(1000);
  }
  while(mode==1)
  {
    nexLoop(nex_listen_list);
    //bt1.getValue(&state);
    if(state==1){break;}
    if(CurrentPage==1){break;}
    dbSerialPrintln("mode 1");
    dbSerialPrintln(IE);
    //delay(1000);
  }
  while(mode==2)
  {
    nexLoop(nex_listen_list);
    //bt0.getValue(&state);
    if(state==0){break;}
    if(CurrentPage==2){break;}
    dbSerialPrintln("mode 2");
    dbSerialPrintln(IE);
  }
  while(mode==3)
  {
    nexLoop(nex_listen_list);
    //bt1.getValue(&state);
    if(state==0){break;}
    if(CurrentPage==1){break;}
    dbSerialPrintln("mode 3");
    dbSerialPrintln(IE);
  }
}
