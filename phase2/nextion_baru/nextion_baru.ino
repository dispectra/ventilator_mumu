//Low-Cost Multi-User Ventilator
#include <Nextion.h>
 
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
} 
//
void bt1PushCallback(void *ptr)  // Using dual state button as Running State
{
  //uint32_t state; 
  bt1.getValue(&state);  // Read value of dual state button to know the state (0 or 1)
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
    dbSerialPrintln(PEEP);
    //delay(1000);
  }
  while(mode==2)
  {
    nexLoop(nex_listen_list);
    //bt0.getValue(&state);
    if(state==0){break;}
    if(CurrentPage==2){break;}
    dbSerialPrintln("mode 2");
    dbSerialPrintln(Vtj);
  }
  while(mode==3)
  {
    nexLoop(nex_listen_list);
    //bt1.getValue(&state);
    if(state==0){break;}
    if(CurrentPage==1){break;}
    dbSerialPrintln("mode 3");
    dbSerialPrintln(RR);
  }
}
