// Low-Cost Ventilator
// ----
// Code including serial comm with mega
// Cek2an di sela2 step dihapus
// inhale dibikin constant speed
// ehale max speed

#include <SoftwareSerial.h>

SoftwareSerial SerialM(11,12); //RX, TX

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
// low dm860, high leadshine

//-- PIN FISIK =======================================================================
#define enaPin 10
#define dirPin 4
#define stepPin 9
#define limitSwitchIn 5
#define limitSwitchEx 6
#define calManMaju 7
#define calManMundur 8

#define pinPEEP A1
#define pinIPP A0

#define pinWarnVol 2 //NANO_SENSE D6
#define pinWarnPres 3 //MEGA D30
#define pinSpurious A2 //NANO_SENSE D7
#define pinPresHold A3 //MEGA D39
#define pinFight A4 //MEGA D33
#define pinStartMotor A5 //MEGA D25

bool callibrated = false;
bool updated = false;
/////////////////////////////////////////////////////////////////////////////////////

//-- Global Variables ===============================================================
String bufferq[4];
int runningState = 0;
bool warnVol = 0;
bool warnPres = 0;
int Vtidal = 0;
float IRat = 1;
float ERat = 0;
int RR = 0;
bool triggerInhale = 0;
int stateNow = 9;

String lastData = "<0,0,0,0>";
bool spuriousPrev = false;

unsigned long now, stepTidal, delayInhale, delayExhale, timeInEx;
double timeInhale, timeExhale, IERatio, timeBreath, slopeFactor, initDelay, timeIPP;

float msq = 1; // x/4
float msq1 = 1.5;
double offsetTimeBreath = 0;

//-- SETUP =========================================================================
void setup() { 
	Serial.begin(115200);
	SerialM.begin(57600);

	slopeFactor = 0.35;
	initDelay = 600/  (msq1);

	//////////// BREATHING PART //////////////////
	pinMode(enaPin, OUTPUT);
	pinMode(dirPin, OUTPUT);
	pinMode(stepPin, OUTPUT);
	pinMode(limitSwitchIn, INPUT_PULLUP);
	pinMode(limitSwitchEx, INPUT_PULLUP);

  pinMode(pinStartMotor,INPUT_PULLUP);

	pinMode(pinPEEP, OUTPUT);
	pinMode(pinIPP, OUTPUT);

	pinMode(calManMaju, INPUT_PULLUP);
	pinMode(calManMundur, INPUT_PULLUP);

	pinMode(pinSpurious, INPUT_PULLUP);
  pinMode(pinPresHold, INPUT_PULLUP);
  pinMode(pinFight, INPUT_PULLUP);
	pinMode(pinWarnVol, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(pinWarnVol), warnVolQ, FALLING);
	pinMode(pinWarnPres, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(pinWarnPres), warnPresQ, FALLING);

	Serial.println("==> CALLIBRATING"); Serial.flush();
//	Callibrate();
	Serial.println("==> CALLIBRATION DONE"); Serial.flush();

  Serial.println("==> READY NANO MOTOR");
}

void loop() {
	unsigned long timeInhaleReal;

	// 1. TERIMA DATA DARI MEGA: STATE, VTi, IE, RR
	updateAllGlobalVars();

	// 2. IF RUNNING
	if (digitalRead(pinStartMotor) == LOW && runningState != 0) {
//		digitalWrite(enaPin, LOW);

		if(stateNow == 0) {
			Serial.println("==> STATUS: ON");
     digitalWrite(enaPin, LOW);
		}

		// 0. INDICATE INHALE START
		readPEEP(1);

		// 1. UPDATE VARIABLES
		stepTidal = round(cekTidal(Vtidal, ERat, RR))* msq;
//    if(Vt/idal <= 420){
//    // NUJUM
//    offsetTimeBreath = -0.027798107*RR-0.001000943*Vtidal-0.179555106*ERat+1.248937329;
//    } /
		timeBreath = (60000 / float(RR)) * 1000;// - offsetTimeBreath* 500000;
		timeIPP = 00000;
		timeInhale = (60000 / float(RR)) * (float(IRat) / float(IRat + ERat)) * 1000 + offsetTimeBreath* ((1000000/(ERat+1))-300000); // dalam microseconds
		timeExhale = (60000 / float(RR)) * (float(ERat) / float(IRat + ERat)) * 1000 ; // dalam microseconds
		delayInhale = float(timeInhale) / float(stepTidal) / 2; // dalam microseconds
		delayExhale = 300 / (msq1); // dalam microseconds

    unsigned long timeInhaleq = timeInhale + timeIPP;
		Serial.println("==================");
		Serial.println("Vol Tidal = " + String(Vtidal));
		Serial.println("Step Tidal = " + String(stepTidal));
		Serial.println("Slope Tidal = " + String(slopeFactor));
		Serial.println("----");
		Serial.println("DELAY Awal = " + String(initDelay));
		Serial.println("DELAY Inhale = " + String(delayInhale));
		Serial.println("DELAY Exhale = " + String(delayExhale));
		Serial.println("----");
    Serial.println(String(offsetTimeBreath* 1000000));
		Serial.println("WAKTU BREATH = " + String(timeBreath));
		Serial.println("WAKTU IDEAL Inhale = " + String(timeInhale));
		Serial.println("WAKTU IDEAL Exhale = " + String(timeExhale));
		Serial.println("----");


		readPEEP(0);
		spuriousPrev = false;
		now = micros();

		// MODE MANDATORY VOLUME
		if(runningState == 1) {
      Serial.println("-> MOD 1");
			//0. INHALE SEQ
			Serial.println("==> INHALE SEQUENCE");
			int stepTidal2 = Inhale();

			//1. Inspiratory Pause Period
			readIPP(1);
//			while((micros()-now) < timeInhale) {delayMicroseconds(1);}

			timeInhaleReal = micros()-now;
			Serial.println("==> TIME INHALE : " + String(timeInhaleReal));


			while((micros()-now) < timeInhale+timeIPP) {delayMicroseconds(1);}
			readIPP(0);

			//2. EXHALE SEG
			Serial.println("==> EXHALE SEQUENCE");
			Exhale(stepTidal2);
		}
		// MODE VOLUME ASSIST + CPAP
		else if(runningState == 2) {
      Serial.println("-> MOD 2");
			spuriousPrev = true;
			//0. INHALE SEQ
			Serial.println("==> INHALE SEQUENCE");
			int stepTidal2 = Inhale2();

			//1. Inspiratory Pause Period
			readIPP(1);
			while((micros()-now) < timeInhale) {delayMicroseconds(1);}

			timeInhaleReal = micros()-now;
			Serial.println("==> TIME INHALE : " + String(timeInhaleReal));

			while((micros()-now) < timeInhaleq) {delayMicroseconds(1);}
			readIPP(0);

			//2. EXHALE SEQ
			Serial.println("==> EXHALE SEQUENCE");
			Exhale(stepTidal2);
		}


		// Sisa waktu exhale
		while((micros()-now) < timeBreath) {
			// 1. Geser sampai mentok (ALL)
      if(digitalRead(limitSwitchEx)){
          digitalWrite(dirPin, !dirInhale);
          digitalWrite(stepPin,HIGH);
          delayMicroseconds(2000);
          digitalWrite(stepPin,LOW);
          delayMicroseconds(2000);
      }

			// 2. cek apakah terjadi spurious breath
			// In case pas exhale mandatory ada spurious, kan ttp harus disupport pressure
			if(checkSpurious()) {
				spuriousPrev = true;
			}

      if (spuriousPrev) {
        runningState = 2;
      }

			// 3. CPAP If spurious
//			if (spuriousPrev) {
//        runningState = 2;
////      Serial.println(checkPEEP());/
//				while(checkPEEP()){ // kalau PEEP blm melewati batas
////        Serial.println(checkPEEP());/
//          if(digitalRead(limitSwitchIn)){
//  					digitalWrite(dirPin, dirInhale);
//  					digitalWrite(stepPin,HIGH);
//  					delayMicroseconds(1000);
//  					digitalWrite(stepPin,LOW);
//  					delayMicroseconds(1000);
//          }
//				}
//			}
		}

		Serial.println("==> TIME EXHALE : " + String(micros()-now - timeInhaleReal - timeIPP));

		Serial.println("TIME TAKEN : " + String(micros() - now));
		Serial.println("----");
		warnPres = false;
		warnVol = false;
    readPEEP(1);
	} else {
		// STATUS OFF
//		readPEEP(0);
//		readIPP(0);
		digitalWrite(enaPin, HIGH);
		if(stateNow !=0) {
			Serial.println("==> STATUS: OFF"); Serial.flush();
			stateNow = 0;
		}

		// Callibrate Maju & Mundur Button
		if(digitalRead(7) == LOW) {
      digitalWrite(enaPin, LOW);
			digitalWrite(dirPin, dirInhale);
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
      digitalWrite(enaPin, LOW);
			digitalWrite(dirPin, !dirInhale);
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
//
		if (!callibrated) {
			Callibrate();
		}
	}
}

//== FUNCTION STATES ==================================================
//-- Sekuens Inhale ====================================================================
int Inhale() { // Mandatory Volume Inhale
	// 0. Init Variables
//  unsigned long now = micros();
	float delayInhale2 = 0.7*delayInhale;
	int stepCount = 0;

	// 1. Set Arah
	digitalWrite(dirPin, dirInhale);
	delayMicroseconds(5);

	unsigned long now = micros();
	// 2. Set Gerakan Stepper
	for(int i = 0; i < stepTidal; i++) { // UNTUK DIPERIKSA
    if(i<0.7*stepTidal){
      delayInhale2 += 1/(0.7*stepTidal) * (0.461538)*delayInhale;
    }
    
		if(digitalRead(limitSwitchIn)) {
			digitalWrite(stepPin, HIGH);
		}
//		else {break;}

		delayMicroseconds(delayInhale2);

		if(checkPressure()) {
			break;
		}

		if(digitalRead(limitSwitchIn)) {
			digitalWrite(stepPin, LOW);
		}
//		else {break;}

		delayMicroseconds(delayInhale2);
//
		if(checkPressure()) {
			break;
		}

		stepCount += 1;

//    Serial.println("TIME 1 STEP IDEAL = " + String(2*delayInhale2));
//    Serial.println("TIME 1 STEP = " + String(micros()-now2));
	}

//  Serial.println(stepCount);

	// 3. Tampil Waktu
	Serial.print("Waktu Inhale = ");
	Serial.println(micros() - now);
	return stepCount;
}

//-- Sekuens Inhale ====================================================================
int Inhale2() { // Assited Volume Inhale
	// 0. Init Variables
	float delayInhale2 = delayInhale;
	int stepCount = 0;

	// 1. Set Arah
	digitalWrite(dirPin, dirInhale);
	delayMicroseconds(5);

	unsigned long now = micros();
	// 2. Set Gerakan Stepper
	for(int i = 0; i < stepTidal; i++) {
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

		stepCount += 1;
	}

//  Serial.println(stepCount);

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
			spuriousPrev = true;
      Serial.println("spurious----------!!!!!!!!!!!!!!!!111!!");
		}

		if(digitalRead(limitSwitchEx)) {
			digitalWrite(stepPin, LOW);
		}

		delayMicroseconds(delayExhale2);
//
		if(checkSpurious()) {
			spuriousPrev = true;
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
float cekTidal(float vol_Tidal, float Eratq, int RR){
//  float lookup_vol[];
//  float lookup_step[];
//  int arraySize;
////  

//  // NUJUM
//
//  if(Eratq < 1.8){
//    if(vol_Tidal < 350){
//      vol_Tidal +=10;
//    }
//  } else if (Eratq == 2){
//    if(vol_Tidal>=590 && RR>=14){
//      vol_Tidal +=40;
//      if(RR>=16){vol_Tidal+=10;};
//      
//    }
//  }
//  
//  if(RR>=17){
//    if(vol_Tidal < 350){
//      vol_Tidal += 10;
//    } else {
//      vol_Tidal += 30;
//    }
//    if(vol_Tidal > 550){
//      vol_Tidal += 20;  
//    }
//  };
//  

  //Eratq = 2
	float lookup_vol[] = {288, 295, 308, 340, 400, 460, 510, 560, 600, 640}; //microstep 1/4
	float lookup_step[] = {774, 788, 803, 849, 911, 977, 1040, 1097, 1134, 1176}; //konfigurasi plus
  int arraySize = sizeof(lookup_vol) / sizeof(lookup_vol[0]);
  
////  
//float lookup_vol[] = {229, 270, 312, 355, 395, 485, 535, 583,/
//float lookup_vol[] = {255, 314, 344, 390, 454, 500, 567, 624}; //microstep 1/4
//  float lookup_step[] = {680, 746, 782, 848, 908, 976, 1039, 1107};
//  if (Eratq == 1){
//    Serial.println("HALOHALO");
//    float lookup_volq[] = {183, 278, 324, 381, 450, 512, 577, 633}; //microstep 1/4
//    float lookup_stepq[] = {730, 789, 845, 903, 960, 1019, 1078, 1138};
//    for(int i= 0; i < arraySize; i++){
//      lookup_vol[i] = lookup_volq[i];
//      lookup_step[i] = lookup_stepq[i];
//    }
//  }

//  float lookup_vol[] = {267, 294, 362, 400, 473, 487, 506, 600, 628}; //microstep 1/4
//  float lookup_step[] = {704, 742, 791, 848, 936, 971, 979, 1107, 1150}; //konfigurasi plus

//  float lookup_vol[] = {147, 220, 255, 315, 441, 500, 600}; //microstep 1/4
//  float lookup_step[] = {551, 624, 694, 707, 832, 936, 1150}; //flat (penekan lama)

//  float lookup_vol[] = {165, 229, 299, 368, 435, 492, 548, 586, 608, 615};
//  float lookup_step[] = {450, 500, 550, 600, 650, 700, 750, 800, 850, 900};
  
	float stepTidal = 0;
	

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
// Update all Global Variable
void updateAllGlobalVars(){
	String received = listeningMega();
	if(!updated) {
		Serial.print("Received: ");
		Serial.println(received);
		Serial.flush();


		int indexStart = 0;
		int indexEnd = 0;

		for(int i = 0; i<4; i++) {
			indexEnd = received.indexOf(",", indexStart);
			bufferq[i] = received.substring(indexStart, indexEnd);
			indexStart = indexEnd+1;
			//    Serial.println(String(i) + ": " + bufferq[i]);
		}

		runningState = bufferq[0].toInt();
		Vtidal = bufferq[1].toInt();
		ERat = bufferq[2].toFloat();
		RR = bufferq[3].toInt();

		updated = true;
	}
//  delay(1000);
}

// Update Buffer from serial
String listeningMega(){
	bool quit = false;
	String seriesData = "";

//  Serial.println(F("Waiting data from Mega...")); Serial.flush();
	while (!quit) {
		if (SerialM.available() > 0) {
			updated = false;
			char x = SerialM.read();
			if (x == '>') {
				seriesData += x;
				quit = true;
			} else {
				if (x == '<') {seriesData = "";}
				seriesData += x;
			}
		} else {
			seriesData = lastData;
			quit = true;
		}
	}

  if(seriesData == lastData) {
    updated = true;
  }

	lastData = seriesData;

	//!! Dummy Data !!
//	seriesData = "<1,0,0,350,2,14,0>";

	return seriesData.substring(1,seriesData.length()-1);
}

bool checkSpurious(){
	triggerInhale = !digitalRead(pinSpurious);
//  Serial.println(triggerInhale);
	return triggerInhale;
}

//!REMEBER!!
bool checkPEEP(){
	bool check = !digitalRead(pinPresHold);
	return check;
}

void warnVolQ(){
	warnVol = true;
 Serial.println("WARNING VOLUME !!!!!!!!!!!!!!!!!!!!!!");
 // delay(1000);
}

void warnPresQ(){
	warnPres = true;
  Serial.println("WARNING PRESSURE !!!!!!!!!!!!!!!!!!!!!!");
}

bool checkPressure(){
//  Serial.println("WarnPres = " + String(warnPres)); Serial.flush();
	return warnPres;
}

bool checkVolume(){
//  Serial.println("Cek Vol = " + String(warnVol)); Serial.flush();
	return warnVol;
}

bool checkVolumePres(){
//  Serial.println("WarnCekPres = " + String(warnVol || warnPres)); Serial.flush();
	return warnVol || warnPres;
}

void readPEEP(bool on){
	if(on) {
		digitalWrite(pinPEEP, LOW);
	}else {
		digitalWrite(pinPEEP, HIGH);
	}
}

void readIPP(bool on){
	if(on) {
		digitalWrite(pinIPP, LOW);
	} else {
		digitalWrite(pinIPP, HIGH);
	}
}
