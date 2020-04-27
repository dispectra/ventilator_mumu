/* Low Cost Ventilator - ITB
 * This is the program for the main microcontroller (Arduino  \
 * Mega). Read sensors, signaling motor controller (Arduino   \
 * Nano), and communicate with HMI (Nextion).
 */

//== LIBRARIES =============================================
#include <SoftwareSerial.h>
#include <Nextion.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

//== GLOBAL VARIABLES ======================================

// Transmitted variables to Nano
//boolean state = 1;p
boolean warningVolume = 0;
boolean warningPressure = 0;
//int Vti = 500;
float ERat = 2;
//int RR = 12;
boolean triggerInhale = 0;

// Nextion variables
uint32_t state = 0;
uint32_t IE = 20;
uint32_t RR = 10;
uint32_t PEEP = 5;
uint32_t Vti = 300;

uint32_t Ox = 20;

int CurrentPage;
int mode = 0;

NexNumber n7 = NexNumber(1, 19, "n7");
NexNumber n8 = NexNumber(1, 20, "n8");
NexNumber n9 = NexNumber(1, 21, "n9");
////update///
NexNumber n10 = NexNumber(1, 22, "n10"); //Preserved for IPP
NexNumber n11 = NexNumber(1, 23, "n11"); //Preserved for PEEP
NexText t1 = NexText(1, 9, "t1");//Preserved for Alarm (send t1.pco = 63488), normally 12678
////////////
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
NexTouch *nex_listen_list[] = {
///Update///
	&n7,&n8,&n9,&n10,&n11,
	&b7,&b8,&b9,&b10,&b11,&b12,&b13,&b14,&b15,&b16,
	&bt0, &page0, &page1, &page2, &page4, &page6, NULL
///Update///
};

// Pressure and Flow variables
uint8_t pressure_int8;
uint8_t flow_int8;
const int PIN_MPX5010DP_pressure = A1;
const int PIN_MPX5010DP_flow = A0;
int pressure_raw;
int flow_raw;
float pressure_float;
float pressure_float2;
float flow_float;

// KE-25 Oxygen Sensor + ADS1115 ADC board
Adafruit_ADS1115 ads;
float scalefactor = 0.1875F;
float volts = 0.0;
int16_t oxygen_raw = 0;
float oxygen_float;

bool readPEEP = false;
bool readIPP = false;

//== MAIN SETUP ============================================
void setup() {
	Serial.begin(115200);   // for debugging
	Serial1.begin(57600 );   // from/to Nano
	Serial2.begin(115200);   // from/to Nextion #Updated to 115200
	ads.begin();      // from/to ADS115 + Oxygen

	pinMode(3, INPUT_PULLUP);
	pinMode(9, OUTPUT);
	attachInterrupt(digitalPinToInterrupt(3), readPEEPQ, FALLING);
	attachInterrupt(digitalPinToInterrupt(2), readIPPQ, FALLING);

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
}

void readPEEPQ(){
	readPEEP = true;
}

void readIPPQ(){
	readIPP = true;
}

float pip_value = 0;
float ipp_value = 0;
bool exhaleStage = false;
#define triggerInhalePin 31

//== MAIN LOOP =============================================
void loop() {
	update2Nano();
//  nexLoop(nex_listen_list);
	Serial.println("state" + String(state));
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
	while (mode == 0) {
		nexLoop(nex_listen_list);
		//dbSerialPrintln(mode);
		flowUpdate();
		pressureUpdate();
		oxygenUpdate();
		if (CurrentPage != 1) {
			break;
		}
		if (state == 1) {
			break;
		}
	}
	while (mode == 1) {
		if(readPEEP){
			PEEPUpdate();
			pip_value = 0;
			readPEEP = false;
			digitalWrite(triggerInhalePin, HIGH); //reset trigger inhale
			exhaleStage = false;
		}
		if (readIPP) {
			IPPUpdate();
			exhaleStage = true;
			readIPP = false;
		}
		if(exhaleStage){
			if(fighting()){
				digitalWrite(triggerInhalePin,LOW);
			}
		}
		nexLoop(nex_listen_list);
		//dbSerialPrintln(mode);
		flowUpdate();
		pressureUpdate1();
		oxygenUpdate();
		if (CurrentPage != 1) {
			break;
		}
		if (state == 0) {
			break;
		}
	}
	while (mode == 3) {
		nexLoop(nex_listen_list);
//    dbSerialPrintln(mode);
//    Serial.print(IE);Serial.print("\t");Serial.print(RR);Serial.print("\t");
//    Serial.print(PEEP);Serial.print("\t");Serial.print(Ox);Serial.print("\t");
//    Serial.println(Vti);
		if (CurrentPage != 4) {
			break;
		}
	}
	while (mode == 4) {
		nexLoop(nex_listen_list);
//    dbSerialPrintln(mode);
		if (CurrentPage != 6) {
			break;
		}
	}
}


//== FUNCTIONS =============================================

//-- Sending necessary information to Arduino Nano ---------
//-- (motor controller) through Serial2 port ---------------
void update2Nano() {
	String message = '<' + String(state) + ','
	                 + String(warningVolume) + ','
	                 + String(warningPressure) + ','
	                 + String(Vti) + ','
	                 + String(ERat) + ','
	                 + String(RR) + ','
	                 + String(triggerInhale) + '>';
	Serial1.print(message); Serial1.flush();

// Debug message
//  Serial.print(F("Message sent to Nano:\n\t")); Serial.print(message); Serial.flush();
}

//-- Digital Filter ----------------------------------------
float digitalFilter(float newImpulse) {
// Variable declaration
	byte filterOrder = 5;
	float impulse[] = {0.0, 0.0, 0.0, 0.0, 0.0};
	float weight[] = {0.1, 0.2, 0.3, 0.2, 0.1};
	float respons = 0.0;

// Updating impulses
	for (int i = 0; i<(filterOrder-1); i++) {
		impulse[i] = impulse[i+1];
	}
	impulse[filterOrder]=newImpulse;

// Calculating response
	for (int i=0; i<filterOrder; i++) {
		respons += (impulse[i] * weight[i]);
	}

	return respons;
}

//-- Float type mapper, for linear regression calibration --
float mapFloat(int rawX, int rawA, int rawB, float realA, float realB) {
	float realX = ( (realB - realA) * float((rawX - rawA) / (rawB - rawA)) ) + realA;
	return realX;
}
// function overloading of mapFloat()
float mapFloat(int rawX, float rawA, float rawB, float realA, float realB) {
	float realX = ( (realB - realA) * float((rawX - rawA) / (rawB - rawA)) ) + realA;
	return realX;
}

//-- Nextion button callback -------------------------------
void b8PushCallback(void *ptr) {  // Press event for button b8
	IE++;
	if(IE>=60)
	{IE=60;}
	ERat = float(IE)/10;
	update2Nano();
}

void b7PushCallback(void *ptr) {  // Press event for button b7
	IE--;
	if(IE==4294967295) {IE=0;}
	ERat = IE/10;
	update2Nano();
}

void b10PushCallback(void *ptr) {  // Press event for button b10
	RR++;
	if(RR>=60)
	{RR=60;}
	update2Nano();
}

void b9PushCallback(void *ptr) {  // Press event for button b9
	RR--;
	if(RR<=10)
	{RR=10;}
	update2Nano();
}

void b14PushCallback(void *ptr) {  // Press event for button b14
	PEEP++;
	if(PEEP>=80)
	{PEEP=80;}
	update2Nano();
}

void b13PushCallback(void *ptr) {  // Press event for button b13
	PEEP--;
	if(PEEP==4294967295)
	{PEEP=0;}
	update2Nano();
}

void b16PushCallback(void *ptr) {  // Press event for button b16
	Vti=Vti+10;
	if(Vti>=800)
	{Vti=800;}
	update2Nano();
}

void b15PushCallback(void *ptr) {  // Press event for button b15
	Vti=Vti-10;
	if(Vti<=100)
	{Vti=100;}
	update2Nano();
}

void b12PushCallback(void *ptr) {  // Press event for button b12
	Ox++;
	if(Ox>=100)
	{Ox=100;}
//  Serial.println(Ox);
}

void b11PushCallback(void *ptr) {  // Press event for button b11
	Ox--;
	if(Ox<=20)
	{Ox=20;}
//  Serial.println(Ox);
}

void bt0PushCallback(void *ptr) {
//uint32_t state;
	bt0.getValue(&state);
	dbSerialPrintln(state);
	update2Nano();
}

void page0PushCallback(void *ptr) {
	CurrentPage = 0;
//  dbSerialPrintln(CurrentPage);
}

void page1PushCallback(void *ptr){
	CurrentPage = 1;
//  dbSerialPrintln(CurrentPage);
}

void page2PushCallback(void *ptr) {
	CurrentPage = 2;
//  dbSerialPrintln(CurrentPage);
}

void page4PushCallback(void *ptr) {
	CurrentPage = 4;
//  dbSerialPrintln(CurrentPage);
}

void page6PushCallback(void *ptr) {
	CurrentPage = 6;
//  dbSerialPrintln(CurrentPage);
}

//-- MPX5010DP pressure sensor ----------------
float calcDatasheetPressure(int x_adc) {
/* Function to return differential pressure value (in cmH2O)
 * from raw ADC value by
 * following typical characteristic in the datasheet
 * ( http://contrails.free.fr/temp/MPX5010.pdf )
 */
	const int abr = 1023;   // ADC bit range
	const int avr = 5;   // ADC voltage range (in volt)
	const float dvo = 0.2;   // Datasheet voltage offset (in volt)
	const float dpg = 2222.22;   // Datasheet pressure gradient (in pascal/volt)
	const float pa_cmh2o = 0.0102;   // 1 Pa = 0.0102 cmH2O
	float pds = (((float(x_adc) / abr * avr) - dvo) * dpg);   // Pressure based on datasheet (in pascal)
	float pds_cmh2o = pds * pa_cmh2o;   // Pressure (in cmH2O)
	return 0.1095*x_adc-4.6591;
}

void flowUpdate() {
// Read sensor output
	flow_raw = analogRead(PIN_MPX5010DP_flow);
//
// Conversion to cmH2O based on datasheet
	flow_float = calcDatasheetPressure(flow_raw+41);

// Convert value from DP (cmH20) to flow (LPM | litre per minute) (ver. mas husnul)
	flow_float = sqrt((flow_float * 2 * 98.06) / 123000) * 0.064 * 60 * 1000;
	flow_float = (flow_float - 100) * 1.6;

//  // Calibration using raw value (already square-rooted) as the formula of differential pressure -> flow
//  int adcA = 111; float flowA = 11; //Measurement #1
//  int adcB = 222; float flowB = 22; //Measurement #2
//  flow_float = mapFloat(flow_raw, float(sqrt(adcA)), float(sqrt(adcB)), flowA, flowB);

// Convert to Nextion waveform graph scale
//  flow_int8 = map(int(flow_float),-140,2500,0,255);
	flow_int8 = map(int(flow_float),-140,500,0,255);

// Update to Nextion waveform graph
	Serial2.print("add 2,0,");
	Serial2.print(flow_int8 - 200);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);

///update///
	Serial2.print("n8.val=");
	Serial2.print(int(flow_float - 380));
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
////////////

	Serial.print(flow_raw); Serial.print(",");
	Serial.print(flow_int8); Serial.print(",");

}

float lastPressure = 0;

void pressureUpdate() {
// Read sensor output
	pressure_raw = analogRead(PIN_MPX5010DP_pressure);

// Conversion to cmH2O based on datasheet
	pressure_float2 = 0.2*calcDatasheetPressure(pressure_raw) + 0.8*lastPressure;
	pressure_float = calcDatasheetPressure(pressure_raw);   //+ 0.2*lastPressure;
	lastPressure = pressure_float;
//  // Calibration using raw value
//  int adcA = 111; float pressureA = 11; //Measurement #1
//  int adcB = 222; float pressureB = 22; //Measurement #2
//  pressure_float = mapFloat(pressure_raw, adcA, adcB, pressureA, pressureB);

//  // Convert to Nextion waveform graph scale (ver. mas husnul)
//  pressure_float = pressure_float * 100 * 1.062;
//  pressure_int8 = int(pressure_float);

// Convert to Nextion waveform graph scale
//  pressure_int8 = map(int(pressure_float), -10, 120, 0, 255);
	pressure_int8 = map(int(pressure_float2), -10, 20, 0, 255);

// Update to Nextion waveform graph
	Serial2.print("add 1,0,");
	Serial2.print(pressure_int8 - 80);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);

///update///
 Serial2.print("n9.val=");
 Serial2.print(round(pressure_float));
 Serial2.write(0xff);
 Serial2.write(0xff);
 Serial2.write(0xff);

//////////

	Serial.print(pressure_raw); Serial.print(",");
	Serial.print(pressure_int8); Serial.println();
//
}

void pressureUpdate1() {
// Read sensor output
	pressure_raw = analogRead(PIN_MPX5010DP_pressure);

// Conversion to cmH2O based on datasheet
	pressure_float2 = 0.2*calcDatasheetPressure(pressure_raw) + 0.8*lastPressure;
	pressure_float = calcDatasheetPressure(pressure_raw);   //+ 0.2*lastPressure;
	lastPressure = pressure_float;

	if(pressure_float>pip_value) {
		pip_value = pressure_float;
	}
//
//  // Calibration using raw value
//  int adcA = 111; float pressureA = 11; //Measurement #1
//  int adcB = 222; float pressureB = 22; //Measurement #2
//  pressure_float = mapFloat(pressure_raw, adcA, adcB, pressureA, pressureB);

//  // Convert to Nextion waveform graph scale (ver. mas husnul)
//  pressure_float = pressure_float * 100 * 1.062;
//  pressure_int8 = int(pressure_float);

// Convert to Nextion waveform graph scale
//  pressure_int8 = map(int(pressure_float), -10, 120, 0, 255);
	pressure_int8 = map(int(pressure_float2), -10, 20, 0, 255);

// Update to Nextion waveform graph
	Serial2.print("add 1,0,");
	Serial2.print(pressure_int8 - 80);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);

	Serial.print(pressure_raw); Serial.print(",");
	Serial.print(pressure_int8); Serial.println();

}

void PEEPUpdate() {
// Read sensor output
	pressure_raw = analogRead(PIN_MPX5010DP_pressure);

// Conversion to cmH2O based on datasheet
	pressure_float2 = 0.2*calcDatasheetPressure(pressure_raw) + 0.8*lastPressure;
	pressure_float = calcDatasheetPressure(pressure_raw);   //+ 0.2*lastPressure;
	lastPressure = pressure_float;

///update///
	Serial2.print("n11.val=");
	Serial2.print(round(pressure_float));
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);

	Serial2.print("n9.val=");
	Serial2.print(round(pip_value));
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);

	Serial2.print("n12.val=");
	Serial2.print(round(ipp_value));
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);

	Serial.print("PEEP VALUE : ");
	Serial.println(pressure_float);
}

void IPPUpdate() {
// Read sensor output
	pressure_raw = analogRead(PIN_MPX5010DP_pressure);

// Conversion to cmH2O based on datasheet
	pressure_float = calcDatasheetPressure(pressure_raw);   //+ 0.2*lastPressure;
	lastPressure = pressure_float;

	ipp_value = pressure_float;
}

void oxygenUpdate() {
	oxygen_raw = ads.readADC_Differential_0_1();
	volts = (oxygen_raw * scalefactor);
	oxygen_float = 10*volts/6;
	int oxygen_int = int(oxygen_float);
	if (oxygen_int==0) oxygen_int = 100;

///update///
	Serial2.print("n7.val=");
	Serial2.print(oxygen_int);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
////////////
}

bool fighting(){
	bool fight = false;
	// if(pressure_float < 0 && flow_float > 0){
	// 	fight = true;
	// }
	return fight;
}
