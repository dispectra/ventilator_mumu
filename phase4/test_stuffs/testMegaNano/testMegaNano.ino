// Low-Cost Ventilator
// ----
// Code including serial comm with mega
// Cek2an di sela2 step dihapus
// inhale dibikin constant speed
// ehale max speed

#include <SoftwareSerial.h>
#include <Nextion.h>
#include <Adafruit_ADS1015.h>

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
#define enaPin 10
#define dirPin 4
#define stepPin 11
#define limitSwitchIn 5
#define limitSwitchEx 6
#define calManMaju 7
#define calManMundur 8

#define pinPEEP A5
#define pinIPP A6

#define pinWarnVol 2 //31
#define pinWarnPres 3 //30
#define pinSpurious A2 //29
#define pinPresHold A3

#define PIN_MPX5010DP_pressure A1
#define PIN_MPX5010DP_flow A0

Adafruit_ADS1115 ads;

float scalefactor = 0.1875F;
float volts = 0.0;

bool callibrated = false;
bool updated = false;
/////////////////////////////////////////////////////////////////////////////////////

//-- Global Variables ===============================================================
bool spontaneousPrev = false;

unsigned long stepTidal, delayInhale, delayExhale, timeInEx;
float timeInhale, timeExhale, IERatio, timeBreath, slopeFactor, initDelay;

//////////////////////////////////////////////////////////////////////////////////////
//-- Variabel pressure
double PIP_raw, PIP_float, pip_value;
double IPP_raw, IPP_float;
double PEEP_raw, PEEP_float;
double PIP_limit = 35;
double flow_raw, flow_float;
double volume_acc;
double oxygen_raw, oxygen_float;

//-- NEXTION ======================================================================
// Nextion variables
uint32_t state = 0;
uint32_t IE = 20;
uint32_t RR = 10;
uint32_t PEEP = 5;
uint32_t Vti = 300;
uint32_t Ox = 20;
float IRat = 1;
int stateNow = 9;

// buffers
uint32_t prev_Vti = 0;
uint32_t prev_Ox = 0;
float ERat = 2;

int CurrentPage;
int mode = 0;

NexNumber n7 = NexNumber(1, 19, "n7");
NexNumber n8 = NexNumber(1, 20, "n8");
NexNumber n9 = NexNumber(1, 21, "n9");
NexNumber n10 = NexNumber(1, 22, "n10"); //Preserved for IPP
NexNumber n11 = NexNumber(1, 23, "n11"); //Preserved for PEEP
NexText t1 = NexText(1, 9, "t1");//Preserved for Alarm (send t1.pco = 63488), normally 12678
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
	&n7,&n8,&n9,&n10,&n11,
	&b7,&b8,&b9,&b10,&b11,&b12,&b13,&b14,&b15,&b16,
	&bt0, &page0, &page1, &page2, &page4, &page6, NULL
};

//-- Nextion button callback
void b8PushCallback(void *ptr) {  // Press event for button b8
	IE++;
	if(IE>=60)
	{IE=60;}
	ERat = float(IE)/10;
	// update2Nano();
}

void b7PushCallback(void *ptr) {  // Press event for button b7
	IE--;
	if(IE==4294967295) {IE=0;}
	ERat = IE/10;
	// update2Nano();
}

void b10PushCallback(void *ptr) {  // Press event for button b10
	RR++;
	if(RR>=60)
	{RR=60;}
	// update2Nano();
}

void b9PushCallback(void *ptr) {  // Press event for button b9
	RR--;
	if(RR<=10)
	{RR=10;}
	// update2Nano();
}

void b14PushCallback(void *ptr) {  // Press event for button b14
	PEEP++;
	if(PEEP>=80)
	{PEEP=80;}
	// update2Nano();
}

void b13PushCallback(void *ptr) {  // Press event for button b13
	PEEP--;
	if(PEEP==4294967295)
	{PEEP=0;}
	// update2Nano();
}

void b16PushCallback(void *ptr) {  // Press event for button b16
	Vti=Vti+10;
	if(Vti>=800)
	{Vti=800;}
	// update2Nano();
}

void b15PushCallback(void *ptr) {  // Press event for button b15
	Vti=Vti-10;
	if(Vti<=100)
	{Vti=100;}
	// update2Nano();
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

//-- SETUP =========================================================================
void setup() {
	Serial.begin(115200);
	Serial2.begin(115200);
	ads.begin();

	slopeFactor = 0.35;
	initDelay = 1200;

	//////////// BREATHING PART //////////////////
	pinMode(dirPin, OUTPUT);
	pinMode(stepPin, OUTPUT);
	pinMode(limitSwitchIn, INPUT_PULLUP);
	pinMode(limitSwitchEx, INPUT_PULLUP);

	pinMode(pinPEEP, OUTPUT);
	pinMode(pinIPP, OUTPUT);

	pinMode(calManMaju, INPUT_PULLUP);
	pinMode(calManMundur, INPUT_PULLUP);
	pinMode(pinSpurious, INPUT_PULLUP);

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

	Serial.println("==> CALLIBRATING"); Serial.flush();
	Callibrate();
	Serial.println("==> CALLIBRATION DONE"); Serial.flush();
}

double lastVol = 0;
void loop() {
	unsigned long timeInhaleReal;

	nexLoop(nex_listen_list);

	if (state == 1) { //on
		if(stateNow == 0) {
			Serial.println("==> STATUS: ON");
		} else {
			readPEEP();
      pip_value = 0;
      volume_acc = 0;
      lastVol = 0;
		}

		stepTidal = round(cekTidal(Vti));
		timeBreath = (60000 / float(RR)) * 1000;
		timeInhale = (60000 / float(RR)) * (float(IRat) / float(IRat + ERat)) * 1000; // dalam microseconds
		timeExhale = (60000 / float(RR)) * (float(ERat) / float(IRat + ERat)) * 1000; // dalam microseconds
		delayInhale = float(timeInhale) / float(stepTidal) / 2 - 400; // dalam microseconds
		delayExhale = 600; // dalam microseconds

		if(spontaneousPrev) { stateNow = 2; Serial.println("==> STATE 2");}
		else{stateNow = 1; Serial.println("==> STATE 1");}

		spontaneousPrev = false;

		Serial.println("==================");
		Serial.println("Vol Tidal = " + String(Vti));
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

		unsigned long now = micros();

		if(stateNow == 1) {
			Serial.println("==> INHALE SEQUENCE");
			int stepTidal2 = Inhale();

			readIPP();
			while((micros()-now) < timeInhale) {delayMicroseconds(1);}
			readOx();

			timeInhaleReal = micros()-now;
			Serial.println("==> TIME INHALE : " + String(timeInhaleReal));

			Serial.println("==> EXHALE SEQUENCE");
			Exhale(stepTidal2);
		} else if(stateNow == 2) {
			Serial.println("==> INHALE SEQUENCE");
			int stepTidal2 = Inhale2();

			readIPP();
			while((micros()-now) < timeInhale) {delayMicroseconds(1);}
			readOx();

			timeInhaleReal = micros()-now;
			Serial.println("==> TIME INHALE : " + String(timeInhaleReal));

			Serial.println("==> EXHALE SEQUENCE");
			Exhale(stepTidal2);
		}

		while((micros()-now) < timeBreath) {
      if(digitalRead(limitSwitchEx)){
          digitalWrite(dirPin, !dirInhale);
          digitalWrite(stepPin,HIGH);
          delayMicroseconds(1000);
          digitalWrite(stepPin,LOW);
          delayMicroseconds(1000);
      }

			if(checkSpurious()) {
				spontaneousPrev = true;
			}

			if (spontaneousPrev) {
				if(!checkPEEP()){ // kalau PEEP blm melewati batas
					digitalWrite(dirPin, dirInhale);
					digitalWrite(stepPin,HIGH);
					delayMicroseconds(1000);
					digitalWrite(stepPin,LOW);
					delayMicroseconds(1000);
				} else {
					digitalWrite(dirPin, !dirInhale);
					digitalWrite(stepPin,HIGH);
					delayMicroseconds(1000);
					digitalWrite(stepPin,LOW);
					delayMicroseconds(1000);
				}
				// while(digitalRead(limitSwitchEx)){
				// 	digitalWrite(dirPin, !dirInhale);
				// 	digitalWrite(stepPin,HIGH);
				// 	delayMicroseconds(1000);
				// 	digitalWrite(stepPin,LOW);
				// 	delayMicroseconds(1000);
				// }
			}
		}
		Serial.println("==> TIME EXHALE : " + String(micros()-now - timeInhaleReal));
		readOx();
		Serial.println("TIME TAKEN : " + String(micros() - now));
		Serial.println("----");

	} else {
		if(stateNow !=0) {
			Serial.println("==> STATUS: OFF"); Serial.flush();
			stateNow = 0;
		}

		if(digitalRead(7) == LOW) {
			digitalWrite(dirPin, LOW);
			if(digitalRead(limitSwitchIn)) {
				Serial.println("Cal In");
				digitalWrite(stepPin, HIGH);
			}
			delayMicroseconds(2000);
			if(digitalRead(limitSwitchIn)) {
				digitalWrite(stepPin, LOW);
			}
			delayMicroseconds(2000);
		} else if(digitalRead(8) == LOW) {

			digitalWrite(dirPin, HIGH);
			if(digitalRead(limitSwitchEx)) {
				Serial.println("Cal Out");
				digitalWrite(stepPin, HIGH);
			}
			delayMicroseconds(2000);
			if(digitalRead(limitSwitchEx)) {
				digitalWrite(stepPin, LOW);
			}
			delayMicroseconds(2000);
		}
		if (!callibrated) {
			Callibrate();
		}
	}
}

//== FUNCTION STATES ==================================================
//-- Sekuens Inhale ====================================================================
int Inhale() {
	// 0. Init Variables
//  unsigned long now = micros();
	float delayInhale2 = delayInhale;
	int stepCount = 0;

	// 1. Set Arah
	digitalWrite(dirPin, dirInhale);
	delayMicroseconds(5);

	unsigned long now = micros();
	// 2. Set Gerakan Stepper
	for(int i = 0; i < stepTidal; i++) {
    readPIP();
		if(digitalRead(limitSwitchIn)) {
			digitalWrite(stepPin, HIGH);
		}

		delayMicroseconds(delayInhale2);

		if(checkPressure()) {
			break;
		}

		if(digitalRead(limitSwitchIn)) {
			digitalWrite(stepPin, LOW);
		}

		delayMicroseconds(delayInhale2);

		if(checkPressure()) {
			break;
		}
		readVolAcc();
		stepCount += 1;
	}

	// 3. Tampil Waktu
	Serial.print("Waktu Inhale = ");
	Serial.println(micros() - now);
	return stepCount;
}

//-- Sekuens Inhale Assisted ====================================================================
int Inhale2() {
	// 0. Init Variables
	float delayInhale2 = delayInhale;
	int stepCount = 0;

	// 1. Set Arah
	digitalWrite(dirPin, dirInhale);
	delayMicroseconds(5);

	unsigned long now = micros();
	// 2. Set Gerakan Stepper
	for(int i = 0; i < stepTidal; i++) {
    readPIP();
		if(digitalRead(limitSwitchIn)) {
			digitalWrite(stepPin, HIGH);
		}

		delayMicroseconds(delayInhale2);

		if(checkVolumePres()) {
			break;
		}

		if(digitalRead(limitSwitchIn)) {
			digitalWrite(stepPin, LOW);
		}

		delayMicroseconds(delayInhale2);

		if(checkVolumePres()) {
			break;
		}
   
		readVolAcc();
		stepCount += 1;
	}

	// 3. Tampil Waktu
	Serial.print("Waktu Inhale = ");
	Serial.println(micros() - now);
	return stepCount;
}

//-- Sekuens Exhale ====================================================================
void Exhale(int stepTidalE) {
	// 0. Init Variables
	float delayExhale2 = initDelay;

	// 1. Set Arah
	digitalWrite(dirPin, !dirInhale);
	delayMicroseconds(5);

	unsigned long now = micros();

	// 2. Set Gerakan Stepper
	for(int i = 0; i < stepTidalE; i++) {
		if(i < slopeFactor*stepTidalE) {
			delayExhale2 -= (initDelay-delayExhale) / (slopeFactor*stepTidalE);
		}
		if(i>(1-slopeFactor)*stepTidal) {
			delayExhale2 += (initDelay-delayExhale) / (slopeFactor*stepTidalE);
		}

		if(digitalRead(limitSwitchEx)) {
			digitalWrite(stepPin, HIGH);
		}

		delayMicroseconds(delayExhale2);
		if(checkSpurious()) {
			spontaneousPrev = true;
		}

		if(digitalRead(limitSwitchEx)) {
			digitalWrite(stepPin, LOW);
		}

		delayMicroseconds(delayExhale2);
//
		if(checkSpurious()) {
			spontaneousPrev = true;
		}
	}

	// 3. Tampil Waktu
	Serial.print("Waktu Exhale = ");
	Serial.println(micros() - now);
}


//-- FUNCTION PLUS PLUS =============================================
//-- Fungsi Kalibrasi
void Callibrate() {
//  digitalWrite(LEDCallibrate, HIGH);
	digitalWrite(dirPin, !dirInhale);
	unsigned long now = millis();
	while(digitalRead(limitSwitchEx)) {
		if (millis() - now > 2000) {break;} // 2 detik maks kalibrasi
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(1000);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(1000);
	}
	callibrated = true;
//  digitalWrite(LEDCallibrate, LOW);
}

//-- Lookup Table Volume Tidal vs Step yang diperlukan ================================
float cekTidal(float vol_Tidal){
	float lookup_vol[] = {223.75, 289.53, 355.72, 410.89, 475.67, 550.83, 606.44, 653.11, 704.17, 748.33, 771.95};
	float lookup_step[] = {450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950};

	float stepTidal = 0;
	int arraySize = sizeof(lookup_vol) / sizeof(lookup_vol[0]);

	// Extrapolasi Bawah
	if(vol_Tidal < cariMin(lookup_vol, arraySize)) {
		float m = float(lookup_step[1] - lookup_step[0]) / (lookup_vol[1] - lookup_vol[0]);
		float c = float(lookup_step[0]) - lookup_vol[0] * m;
		stepTidal = m * vol_Tidal + c;
	}
	// Extrapolasi Atas
	else if(vol_Tidal > cariMax(lookup_vol, arraySize)) {
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
	for(int i=0; i< arraySize; i++) {
		if(list[i] <= mini) {
			mini = list[i];
		}
	}
	return mini;
}

float cariMax(float list[], int arraySize){
	float maxi = 0;
	for(int i=0; i< arraySize; i++) {
		if(list[i] >= maxi) {
			maxi = list[i];
		}
	}
	return maxi;
}


//== FUNCTIONS SERIAL ==============================================
bool checkSpurious(){ //!! BELUM
	bool spur = false;
	// if(pressure_float < 0 && flow_float > 0){
	// 	fight = true;
	// }
	return spur;
}

bool checkPEEP(){
	PEEP_raw = analogRead(PIN_MPX5010DP_pressure);
	PEEP_float = 0.1095*PEEP_raw-4.6591;
	if(PEEP_float>PEEP){
		return true;
	} else {
		return false;
	}
}

bool checkPressure(){
	if(PIP_float>(0.1095*analogRead(PIN_MPX5010DP_pressure)-4.6591)){
//		return true;
	} else {
//		return false;
	}
  return false;
}

bool checkVolume(){
	if(volume_acc> Vti){
//		return true;
	} else {
//		return false;
	}
 return false;
}

bool checkVolumePres(){
	return checkVolume() || checkPressure();
}

void readPEEP(){
	PEEP_raw = analogRead(PIN_MPX5010DP_pressure);
	PEEP_float = 0.1095*PEEP_raw-4.6591;

	Serial2.print("n11.val=");
	Serial2.print(round(PEEP_float));
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);

	Serial2.print("n10.val=");
	Serial2.print(round(IPP_float));
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);

	Serial2.print("n9.val=");
	Serial2.print(round(pip_value));
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
}

void readIPP(){
	IPP_raw = analogRead(PIN_MPX5010DP_pressure);
	IPP_float = 0.1095*IPP_raw-4.6591;
}

void readPIP(){
	PIP_float = 0.1095*analogRead(PIN_MPX5010DP_pressure)-4.6591;
	if(pip_value < PIP_float){
		pip_value = PIP_float;
	}
}
double inc;

void readVolAcc(){
//  inc = 1*((5.7871*analogRead(PIN_MPX5010DP_flow)-295.14)/60 * delayInhale/500) + 0*lastVol;
	volume_acc += ((5.7871*analogRead(PIN_MPX5010DP_flow)-295.14)/60 * (delayInhale+100)/500);
//  lastVol = inc;
  Serial.println(volume_acc);
}

void readOx(){
	oxygen_raw = ads.readADC_Differential_0_1();
	volts = (oxygen_raw * scalefactor);
	oxygen_float = 10*volts/6;
	int oxygen_int = round(oxygen_float);
	if (oxygen_int==0) oxygen_int = 100;

	///update///
	Serial2.print("n7.val=");
	Serial2.print(oxygen_int);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
	////////////
}
