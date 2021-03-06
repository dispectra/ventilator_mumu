#include <Nextion.h>

uint32_t IE = 0;
uint32_t RR = 10;
uint32_t PEEP = 5;
uint32_t Vti = 100;
uint32_t Ox = 20;
int CurrentPage;
int mode = 0;
uint32_t state = 0;
NexDSButton bt0 = NexDSButton(1, 3, "bt0");
NexButton b7 = NexButton(4, 20, "b7");
NexButton b8 = NexButton(4, 19, "b8");
NexButton b9 = NexButton(4, 21, "b9");
NexButton b10 = NexButton(4, 22, "b10");
NexButton b11 = NexButton(4, 26, "b11");
NexButton b12 = NexButton(4, 25, "b12");
NexButton b13 = NexButton(4, 24, "b13");
NexButton b14 = NexButton(4, 23, "b14");
NexButton b15 = NexButton(4, 27, "b15");
NexButton b16 = NexButton(4, 28, "b16");
NexPage page0 = NexPage(0, 0, "page0");
NexPage page1 = NexPage(1, 0, "page1");
NexPage page2 = NexPage(2, 0, "page2");
NexPage page4 = NexPage(4, 0, "page4");
NexPage page6 = NexPage(6, 0, "page6");

NexTouch *nex_listen_list[] =
{
  &b7,&b8,&b9,&b10,&b11,&b12,&b13,&b14,&b15,&b16,
  &bt0, &page0, &page1, &page2, &page4, &page6, NULL
};
void b8PushCallback(void *ptr)  // Press event for button b0
{
  IE++;
  if(IE>=60)
  {IE=60;}
} 
void b7PushCallback(void *ptr)  // Press event for button b0
{
  IE--; 
  if(IE==4294967295){IE=0;}
}
void b10PushCallback(void *ptr)  // Press event for button b0
{
  RR++;
  if(RR>=60)
  {RR=60;}
} 
void b9PushCallback(void *ptr)  // Press event for button b0
{
  RR--; 
  if(RR<=10)
  {RR=10;}
}
void b14PushCallback(void *ptr)  // Press event for button b0
{
  PEEP++;
  if(PEEP>=80)
  {PEEP=80;}
} 
void b13PushCallback(void *ptr)  // Press event for button b0
{
  PEEP--; 
  if(PEEP==4294967295)
  {PEEP=0;}
}
void b16PushCallback(void *ptr)  // Press event for button b0
{
  Vti=Vti+10;
  if(Vti>=800)
  {Vti=800;}
} 
void b15PushCallback(void *ptr)  // Press event for button b0
{
  Vti=Vti-10; 
  if(Vti<=100)
  {Vti=100;}
}
void b12PushCallback(void *ptr)  // Press event for button b0
{
  Ox++;
  if(Ox>=100)
  {Ox=100;}
  Serial.println(Ox);
} 
void b11PushCallback(void *ptr)  // Press event for button b0
{
  Ox--; 
  if(Ox<=20)
  {Ox=20;}
  Serial.println(Ox);
}

void bt0PushCallback(void *ptr) {
  //uint32_t state;
  bt0.getValue(&state);
  dbSerialPrintln(state);
}
void page0PushCallback(void *ptr)
{
  CurrentPage = 0;
  //  dbSerialPrintln(CurrentPage);
}
void page1PushCallback(void *ptr)
{
  CurrentPage = 1;
  //  dbSerialPrintln(CurrentPage);
}
void page2PushCallback(void *ptr)
{
  CurrentPage = 2;
  //  dbSerialPrintln(CurrentPage);
}
void page4PushCallback(void *ptr)
{
  CurrentPage = 4;
  //  dbSerialPrintln(CurrentPage);
}
void page6PushCallback(void *ptr)
{
  CurrentPage = 6;
  //  dbSerialPrintln(CurrentPage);
}

uint8_t tekanan;
uint8_t flow;
const int PIN_MPX5010DP = A1;
const int PIN_MPX5010F = A0;
int MPX5010DP_rawValue;
int MPX5010F_rawValue;
float MPX5010DP_pressureValue;
float MPX5010F_flowValue;
float calcDatasheetPressure(int x_adc) {
  /* Function to return differential pressure value from raw ADC value by
     following typical characteristic in the datasheet
     ( http://contrails.free.fr/temp/MPX5010.pdf )
  */
  const int abr = 1023; // ADC bit range
  const int avr = 5;    // ADC voltage range (in volt)
  const float dvo = 0.2;  // Datasheet voltage offset (in volt)
  const float dpg = 2222.22;  // Datasheet pressure gradient (in pascal/volt)
  const float pa_cmh2o = 0.0102; // 1 Pa = 0.0102 cmH2O
  float pds = (((float(x_adc) / abr * avr) - dvo) * dpg); // Pressure based on datasheet (in pascal)
  float pds_cmh2o = pds * pa_cmh2o; // Pressure (in cmH2O)
  return pds_cmh2o;
}

void baca_flow()
{
  MPX5010F_rawValue = analogRead(PIN_MPX5010F);
  MPX5010F_flowValue = calcDatasheetPressure(MPX5010F_rawValue);
  MPX5010F_flowValue = sqrt((MPX5010F_flowValue * 2 * 98.06) / 123000) * 0.064 * 60 * 1000;
  MPX5010F_flowValue = (MPX5010F_flowValue - 100) * 1.6;
  flow = int(MPX5010F_flowValue);
  Serial2.print("add 2,0,");
  Serial2.print(flow);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
}
void baca_tekanan()
{
  MPX5010DP_rawValue = analogRead(PIN_MPX5010DP);
  MPX5010DP_pressureValue = calcDatasheetPressure(MPX5010DP_rawValue);
  MPX5010DP_pressureValue = MPX5010DP_pressureValue * 100 * 1.062;
  tekanan = (int)MPX5010DP_pressureValue;
  Serial2.print("add 1,0,");
  Serial2.print(tekanan);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
}

void setup() {
  // put your setup code here, to run once:
  nexInit();
  bt0.attachPush(bt0PushCallback, &bt0);
  b7.attachPush(b7PushCallback, &b7);
  b8.attachPush(b8PushCallback, &b8);
  b9.attachPush(b9PushCallback, &b9);
  b10.attachPush(b10PushCallback, &b10);
  b11.attachPush(b11PushCallback, &b11);
  b12.attachPush(b12PushCallback, &b12);
  b13.attachPush(b13PushCallback, &b13);
  b14.attachPush(b14PushCallback, &b14);
  b15.attachPush(b15PushCallback, &b15);
  b16.attachPush(b16PushCallback, &b16);
  page0.attachPush(page0PushCallback, &page0);
  page1.attachPush(page1PushCallback, &page1);
  page2.attachPush(page2PushCallback, &page2);
  page4.attachPush(page4PushCallback, &page4);
  page6.attachPush(page6PushCallback, &page6);
  //  dbSerialPrintln(CurrentPage);
  delay(500);
}

void loop() {
  // put your main code here, to run repeatedly:
  //  nexLoop(nex_listen_list);
  if (state == 0 && CurrentPage == 1) {
    mode = 0;
  }
  if (state == 1 && CurrentPage == 1) {
    mode = 1;
  }
  if (CurrentPage == 2) {
    mode = 2;
  }
  if (CurrentPage == 4) {
    mode = 3;
  }
  if (CurrentPage == 6) {
    mode = 4;
  }
  while (mode == 0)
  {
    nexLoop(nex_listen_list);
    //    dbSerialPrintln(mode);
    baca_flow();
    baca_tekanan();
    if (CurrentPage != 1) {
      break;
    }
    if (state == 1) {
      break;
    }
  }
  while (mode == 1)
  {
    nexLoop(nex_listen_list);
    //    dbSerialPrintln(mode);
    baca_flow();
    baca_tekanan();
    if (CurrentPage != 1) {
      break;
    }
    if (state == 0) {
      break;
    }
  }
  while (mode == 2)
  {
    nexLoop(nex_listen_list);
    //    dbSerialPrintln(mode);
    if (CurrentPage != 2) {
      break;
    }
  }
  while (mode == 3)
  {
    nexLoop(nex_listen_list);
    //    dbSerialPrintln(mode);
//    Serial.print(IE);Serial.print("\t");Serial.print(RR);Serial.print("\t");
//    Serial.print(PEEP);Serial.print("\t");Serial.print(Ox);Serial.print("\t");
//    Serial.println(Vti);
    if (CurrentPage != 4) {
      break;
    }
  }
  while (mode == 4)
  {
    nexLoop(nex_listen_list);
    //    dbSerialPrintln(mode);
    if (CurrentPage != 6) {
      break;
    }
  }
}
