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
#include <Servo.h>
#include <ArduinoSort.h>

SoftwareSerial SerialFl(62, 63); //A8 A9
//== GLOBAL VARIABLES ======================================
// Servos
Servo servoPEEP;
Servo servoOxigen;
#define enaServoPEEP 38
#define sigServoPEEP 36
#define enaServoOx 42
#define sigServoOx 40

// Warning-Warning
#define warningPressure_PIN 30
#define pinSpurious 32
#define pinFight 33
#define warningPEEP_PIN 34

#define pinStartMotor 58 //A4

boolean warningPressure = 0;
boolean triggerInhale = 0;

// Pressure States
float pip_value = 0;
float ipp_value = 0;
bool exhaleStage = false;

// Nextion variables
uint32_t setupq = 0;
uint32_t runningState = 0;
uint32_t IE = 20;
uint32_t RR = 10;
uint32_t PEEP_LIMIT = 5;
uint32_t Vti = 300;
uint32_t Ox = 20;

// buffers
uint32_t prev_Vti = 0;
uint32_t prev_Ox = 0;
float ERat = 2;

int CurrentPage;
int lastPage;
int mode = 0;

// new variables
int PIP_LIMIT = 20;
int peakCount = 0;

#define ButtonResetAlarm_PIN 36
bool alarms[9] = {0,0,0,0,0,0,0,0,0};

//NexNumber(page, ID, Nama)
//!!HMI!!
// SPLASH
NexPage page0 = NexPage(0, 0, "page0");

// SETUP
NexPage page1 = NexPage(1, 0, "page1");
NexDSButton bt5 = NexDSButton(1, 2, "bt5");//mandatory
NexDSButton bt3 = NexDSButton(1, 1, "bt3");//assisted

// MAIN DISPLAY
NexPage page2 = NexPage(2, 0, "page2");
NexDSButton bt0 = NexDSButton(2, 2, "bt0"); //running state toggle
NexNumber n7 = NexNumber(2, 16, "n7"); //display FiO2
NexNumber n9 = NexNumber(2, 18, "n9"); //display PIP
NexNumber n10 = NexNumber(2, 19, "n10"); //Preserved for IPP
NexNumber n11 = NexNumber(2, 20, "n11"); //Preserved for PEEP
NexText t0 = NexText(2, 5, "t0");//Preserved for send setup mode (mandatory/assisted)
NexText t1 = NexText(2, 8, "t1");//Preserved for Alarm (send t1.pco = 63488), normally 12678

// Init Values
NexPage page3 = NexPage(3, 0, "page3");
NexButton b7 = NexButton(3, 20, "b7"); // IE --
NexButton b8 = NexButton(3, 19, "b8"); // IE ++
NexButton b9 = NexButton(3, 21, "b9"); // RR--
NexButton b10 = NexButton(3, 22, "b10"); // RR ++
NexButton b11 = NexButton(3, 26, "b11"); // FiO2--
NexButton b12 = NexButton(3, 25, "b12"); // FiO2++
NexButton b13 = NexButton(3, 24, "b13"); // PEEP--
NexButton b14 = NexButton(3, 23, "b14"); // PEEP++
NexButton b15 = NexButton(3, 27, "b15"); // Vti--
NexButton b16 = NexButton(3, 28, "b16"); // Vti++

//Alarm set tolerance
NexPage page4 = NexPage(4, 0, "page4");
NexButton b17 = NexButton(4, 16, "b17"); // High pressure limit--
NexButton b18 = NexButton(4, 15, "b18"); // HPL++
NexButton b19 = NexButton(4, 17, "b19"); // Low Pressure limit--
NexButton b20 = NexButton(4, 18, "b20"); // LPL++
NexButton b21 = NexButton(4, 20, "b21"); // O2 tol--
NexButton b22 = NexButton(4, 19, "b22"); // O2 tol++

//!!HMI!!
NexTouch *nex_listen_list[] = {
	&b7,&b8,&b9,&b10,&b11,&b12,&b13,&b14,&b15,&b16,&b17,&b18,&b19,&b20,&b21,&b22, //button
	&bt0,&bt5,&bt3, //dual state button
	&page0, &page1, &page2, &page3, &page4, NULL //page
};

// Pressure and Flow variables
uint8_t pressure_int8;
int pressure_raw;
float pressure_float;

// KE-25 Oxygen Sensor + ADS1115 ADC board
Adafruit_ADS1115 ads;
float scalefactor = 0.1875F;
float volts = 0.0;
int16_t oxygen_raw = 0;
float oxygen_float;

bool readPEEP = false;
bool readIPP = false;

float offset = 0;
int buffsize = 50;

//== MAIN SETUP ============================================
void setup() {
	Serial.begin(115200);   // for debugging
	Serial1.begin(57600);   // from/to Nano Motor
	Serial2.begin(115200);   // from/to Nextion #Updated to 115200
	Serial3.begin(115200); // NANO alarm
	SerialFl.begin(57600); // NANO Sensor

	ads.begin();      // from/to ADS115 + Oxygen
//  ads.setGain(GAIN_SIXTEEN);

	pinMode(3, INPUT_PULLUP);
	pinMode(2, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(3), readPEEPQ, FALLING);
	attachInterrupt(digitalPinToInterrupt(2), readIPPQ, FALLING);

  pinMode(warningPressure_PIN, OUTPUT);
  pinMode(pinFight, OUTPUT);
  pinMode(warningPEEP_PIN, OUTPUT);
  pinMode(pinSpurious, INPUT_PULLUP);
  pinMode(pinStartMotor, OUTPUT);
  digitalWrite(pinStartMotor, HIGH);
  
//	servoPEEP.attach(sigServoPEEP);
//	servoOxigen.attach(sigServoOx);
//	pinMode(enaServoPEEP, OUTPUT);
//	pinMode(enaServoOx, OUTPUT);

	// Tombol
	pinMode(ButtonResetAlarm_PIN, INPUT_PULLUP);

	nexInit();
  page0.attachPush(page0PushCallback, &page0);

  page1.attachPush(page1PushCallback, &page1);
  bt3.attachPush(bt3PushCallback, &bt3);
  bt5.attachPush(bt5PushCallback, &bt5);

  page2.attachPush(page2PushCallback, &page2);
	bt0.attachPush(bt0PushCallback, &bt0);

  page3.attachPush(page3PushCallback, &page3);
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

	page4.attachPush(page4PushCallback, &page4);
  b17.attachPush(b17PushCallback, &b17);
  b18.attachPush(b18PushCallback, &b18);
  b19.attachPush(b19PushCallback, &b19);
  b20.attachPush(b20PushCallback, &b20);
  b21.attachPush(b21PushCallback, &b21);
  b22.attachPush(b22PushCallback, &b22);

//  dbSerialPrintln(CurrentPage);
	zeroPresSensor();
  Serial.println("==> READY MEGA");
}


//== MAIN LOOP =============================================
void loop() {
	update2Nano();
	Serial.println("runningState" + String(runningState));

	// runningState = 0, artinya mesin berhenti
	// runningState = 1, mesin jalan

	// setup 1 Mandatory Volume
	// setup 2 Assited + CPAP
	// setup 0 STOP

	// mode untuk loop nya
	// mode 0 = Page selain page main
	// mode 1 = Page Main, OFF
	// mode 2 = Page Main, ON
	// mode 5 = routine stuck ketika alarm level = HIGH

	// Page 0 splash
	// Page 1 pemilihan mode (pemilihan assist/mandatory)
	// Page 2 main display (yg ada grafik)
	// Page 3 Setting/config (IE, RR, Vti, PEEP, O2)
	// Page 4 Setting alarm (over PIP, under IPP, O2_tolerance)

	if(mode == 5) {
    setupq = 0;
    runningState = 0;
    update2Nano();
    Serial.println("WOKE"); Serial.flush();
//    delay(100000);
	} else {
  	if(CurrentPage == 2){ // Untuk page 1
      if(runningState == 0) { // kalau lagi off
        mode = 1;
      } else if(runningState == 1 && setupq != 0) { // kalau lagi on
        mode = 2;
      }
    } else { //untuk page selain page main
  		mode = 0;
  		lastPage = CurrentPage;
  	}
	}

	Serial.println("CURRENTPAGE" + String(CurrentPage));
	Serial.println("MODE" + String(mode));

  // mode utk routine stuck setelah alarm
  while (mode == 5) {
    sendSetupToHMI('C');
    setupq = 0;
   Serial.println("------------------ mode5"); delay(10000); 

    if (digitalRead(ButtonResetAlarm_PIN) == LOW) {
      Serial.println("Reset system button pressed"); Serial.flush(); delay(10000);
      mode = 0;
      setAlarm("99_LOW"); // Reset and send all alarms[] off
      break;
    }
//    Serial.println("!!! ALARM HIGH. MACHINE STOPPED. FIX THE SETUP THEN PRESS ALARM RESET BUTTON !!!"); Serial.fulsh();
  }

	// NOT PAGE 1
	while (mode == 0) {
		zeroPresSensor();

		//0. Update bacaan nextion
		nexLoop(nex_listen_list);

		//1. Cek kalau ganti halaman/state
		if (CurrentPage != lastPage) {break;}
	}

	// PAGE 1, RUNNING STATE OFF
	while (mode == 1) {
		// zeroPresSensor();

		//0. Update bacaan nextion
		nexLoop(nex_listen_list);

		//1. Update nilai Oksigen
		oxygenUpdate();
   pressureUpdate1();
    readPEEP = false;
    readIPP = false;

		//2. Cek kalau ganti halaman/state
		if (CurrentPage != 2) {break;}
		if (runningState != 0) {break;}
	}

	// PAGE 1, RUNNING STATE ON
	while (mode == 2) {

		// // Set Servos ------
		// if(prev_Vti != Vti){
		//  setServoPEEP(Vti);
		//  prev_Vti = Vti;
		// }
		// if(prev_Ox != Ox){
		//  setServoOx(Ox);
		//  prev_Ox = Ox;
		// }

		//0. Update Bacaan Nextion ---
		nexLoop(nex_listen_list);

		//1. TIMING INHALE/EXHALE CHECK ---
		if(readPEEP) {
			PEEPUpdate();
			pip_value = 0;
	    peakCount = 0;
			readPEEP = false;
			exhaleStage = false;
		}
		if (readIPP) {
			IPPUpdate();
			exhaleStage = true;
			readIPP = false;
		}

		//2. Update Value PIP dan Oksigen ---
		pressureUpdate1();
		oxygenUpdate();

    Serial.println("SETUPQ : " + String(setupq));
    
		//3. ROUTINE EXHALE ---
		if(exhaleStage) {
      digitalWrite(warningPressure_PIN,HIGH);
      digitalWrite(warningPEEP_PIN, HIGH);
			Serial.println("EXHALING");

			if(setupq==2) {
        setAlarm("04_OFF");
				//1. PEEP Pressure HOLD (CPAP) if mode B
//        pressure_float = 1;/
				if(pressure_float < PEEP_LIMIT) {
//        Serial.println("MEH");/
					digitalWrite(warningPEEP_PIN, LOW);
					setAlarm("06_ON");
				} else {
					digitalWrite(warningPEEP_PIN, HIGH);
					setAlarm("06_OFF");
				}
			} else if(setupq==1) {
				// Cek Spurious Trigger dari Arduino Sensor
				if (digitalRead(pinSpurious)== LOW) {
					setupq = 2;
					sendSetupToHMI('B');
					setAlarm("04_ON");
				}
			}

		}

		//4. ROUTINE INHALE ---
		else {
      setAlarm("04_OFF");
			Serial.println("INHALING");

			//0. Cek Fighting
//			if(fighting()) {
//				digitalWrite(pinFight,LOW);
//				setAlarm("02_ON");
//				mode = 5; break;
//			}

//			if (pressure_float > PIP_LIMIT) { // warning pressure to nano through digital pin
//				Serial.println("WARN!!");
//				digitalWrite(warningPressure_PIN,LOW);
//				delay(1);
//				digitalWrite(warningPressure_PIN,HIGH);
//				setAlarm("00_ON");
//				mode = 5; break;
//			}
		}
//		nexLoop(nex_listen_list);

		if (CurrentPage != 2) {break;}
		if (runningState == 0) {break;}
		if (setupq == 0) {break;}
	}

	
}


//== FUNCTIONS =============================================

void sendSetupToHMI(char setupHMI){
	// !!HOMEWORK!!
	//koding untuk mengirimkan teks ke display HMI
	String textbuffer;

	if(setupHMI = 'A') {
		textbuffer = "Mandatory Volume Control";
	} else if (setupHMI = 'B'){
		textbuffer = "Assisted Volume Control";
	} else if (setupHMI = 'C'){
		textbuffer = "Stop";
	}
	Serial2.print("t0.txt=");
	Serial2.print("\"");
	Serial2.print(textbuffer); //buffer teks
	Serial2.print("\"");
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
}

//-- Interrupts
void readPEEPQ(){
	readPEEP = true;
}
void readIPPQ(){
	readIPP = true;
}

//-- Zero-ing Pressure Sensor
float calcPres(float pres_rawq){
  float calc = 0.3135*pres_rawq-1316.0693-3.14;

  return calc;
}

void zeroPresSensor(){
  //0. Create buffer for value and histogram
  float val[buffsize];
  float lastVal;
  int index_terpilih = 0;
  int mode_count = 0;
  int valcount = 0;
	peakCount = 0;

  //1. Ambil x data
  for(int i=0; i<buffsize; i++){
    val[i] = calcPres(ads.readADC_SingleEnded(2))+offset;
  }

  sortArray(val, buffsize);

  //2. create histogram
  for (int i=0; i<buffsize; i++){
    if(lastVal != val[i]){
      lastVal = val[i];
      valcount = countOccurances(val, val[i]);
      if(valcount>=mode_count){
        mode_count = valcount;
        index_terpilih = i;
      }
    }
  }

  //3. Return Mode
  offset += -1*val[index_terpilih];
//  Serial.println("OFFSET : " + String(offset));
}

int countOccurances(float val[], float q){
  int count = 0;
  for(int i=0; i<buffsize; i++){
    if(val[i] == q){
      count++;
    }
  }
  return count;
}

//-- Sending necessary information to Arduino Nano ---------
//-- (motor controller) through Serial2 port ---------------
void update2Nano() {
  int stateq;
  if(runningState==0 || setupq == 0){stateq = 0; digitalWrite(pinStartMotor, HIGH);} //off
  else{stateq = setupq; digitalWrite(pinStartMotor, LOW);} //on
	String message = '<' + String(stateq) + ','
	                 + String(Vti) + ','
	                 + String(ERat) + ','
	                 + String(RR) + '>';
	Serial1.print(message); Serial1.flush();
	Serial.println(message);
	String message2 = '<' + String(stateq) + ','
	                  + String(Vti) + '>';
	SerialFl.print(message2); SerialFl.flush();

// Debug message
//  Serial.print(F("Message sent to Nano:\n\t")); Serial.print(message); Serial.flush();
}

//-- Float type mapper, for linear regression calibration --
float mapFloat(float rawX, float rawA, float rawB, float realA, float realB) {
	float realX = ( (realB - realA) * float((rawX - rawA) / (rawB - rawA)) ) + realA;
	return realX;
}

//!!HMI!!
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
	PEEP_LIMIT++;
	if(PEEP_LIMIT>=80)
	{PEEP_LIMIT=80;}
	update2Nano();
}

void b13PushCallback(void *ptr) {  // Press event for button b13
	PEEP_LIMIT--;
	if(PEEP_LIMIT==4294967295)
	{PEEP_LIMIT=0;}
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

//#UPDATE HMI
void b17PushCallback(void *ptr) {
	//HIGH pressure limit --
	//default di min 20 max 40
	//asumsi variabel = HPL
	PIP_LIMIT--;
	if(PIP_LIMIT<=20){PIP_LIMIT=20;}
  dbSerialPrintln("PIP" + String(PIP_LIMIT));
}

void b18PushCallback(void *ptr) {
	//HIGH pressure limit ++
	//default di 20 max 40
	//asumsi variabel = HPL
	PIP_LIMIT++;
	if(PIP_LIMIT>=40){PIP_LIMIT=40;}
  dbSerialPrintln("PIP" + String(PIP_LIMIT));
}

void b19PushCallback(void *ptr) {
	//Low pressure limit --
	//default di min 20 max 35
	//asumsi variabel = LPL
	/*
	   LPL--;
	   if(LPL<=20){LPL=20;}
	 */
  dbSerialPrintln("LPL-");
}

void b20PushCallback(void *ptr) {
	//Low pressure limit ++
	//default di min 20 max 35
	//asumsi variabel = LPL
	/*
	   LPL++;
	   if(LPL>=35){LPL=35;}
	 */
  dbSerialPrintln("LPL+");
}

void b21PushCallback(void *ptr) {
	//O2 tolerance --
	//default di min 5 max 30
	//asumsi variabel = O2tol
	/*
	   O2tol--;
	   if(HPL<=5){O2tol=5;}
	 */
  dbSerialPrintln("O2-");
}

void b22PushCallback(void *ptr) {
	//O2 tolerance ++
	//default di min 5 max 30
	//asumsi variabel = O2tol
	/*
	   O2tol++;
	   if(HPL>=30){O2tol=30;}
	 */
  dbSerialPrintln("O2+");
}

void bt0PushCallback(void *ptr) {
//uint32_t runningState;
	bt0.getValue(&runningState);
	dbSerialPrintln(runningState);
	update2Nano();
}

void bt5PushCallback(void *ptr) { //mandatory toggle
//uint32_t runningState;
// asumsi disimpan di varlokal
	uint32_t varlokal;
	bt5.getValue(&varlokal);
  Serial.println("VARLOKAL" + String(varlokal));
	if(varlokal==1) {setupq=1;}
	else{setupq=0;}
	dbSerialPrintln("SETUP" + String(setupq));
	update2Nano();
}

void bt3PushCallback(void *ptr) { //assisted
//uint32_t runningState;
	uint32_t varlokal;
	bt3.getValue(&varlokal);
  Serial.println("VARLOKAL" + String(varlokal));
	if(varlokal==1) {setupq=2;}
	else{setupq=0;}
	dbSerialPrintln("SETUP" + String(setupq));
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

void page3PushCallback(void *ptr) {
	CurrentPage = 3;
//  dbSerialPrintln(CurrentPage);
}

void page4PushCallback(void *ptr) {
	CurrentPage = 4;
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

void pressureUpdate() {
// Read sensor output
	pressure_float = calcPres(ads.readADC_SingleEnded(2)) + offset;
	pressure_int8 = map(int(pressure_float), -10, 20, 0, 255);

// Update to Nextion waveform graph
	Serial2.print("add 1,0,");
	Serial2.print(pressure_int8 - 80);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
}

double PIP_raw, PIP_float;
double IPP_raw, IPP_float;
double PEEP_raw, PEEP_float;

void pressureUpdate1() {
	pressure_float = calcPres(ads.readADC_SingleEnded(2)) + offset;
  Serial.println("Pres: " + String(pressure_float));
	pressure_int8 = map(int(pressure_float), -10, 20, 0, 255);

	if(pip_value < pressure_float) {
		pip_value = pressure_float;
	} else {
		if(pip_value - pressure_float > 5){
			peakCount++;
			pip_value = 0;
		}
	}

// Update to Nextion waveform graph
	Serial2.print("add 1,0,");
	Serial2.print(pressure_int8 - 80);
	Serial2.write(0xff);
	Serial2.write(0xff);
	Serial2.write(0xff);
}

void PEEPUpdate() {
//  PEEP_raw = 0.3135*ads.readADC_SingleEnded(0)-1163.1143-153.43;
	pressure_float = calcPres(ads.readADC_SingleEnded(2)) + offset;

	Serial2.print("n11.val=");
	Serial2.print(round(pressure_float));
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

void IPPUpdate() {
	pressure_float = calcPres(ads.readADC_SingleEnded(2)) + offset;
	IPP_float = pressure_float;
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

//-- Check if patient fight (spurious breath)
// !!HOMEWORK!!
bool fighting(){
	// cek fighting pakai multiple Peak
	bool fight = false;

	if(peakCount > 2){
		fight = true;
		peakCount = 0;
	}

	return fight;
}

//-- Send alarm mode to buzzer/alarm microcontroller ----
void setAlarm(String key) {   // Key example: 01_ON   ;   09_OFF
	//Define array of alarm triggers
	//alarmzz[0] = High pressure exceeded PIP (HIGH)
	//alarmzz[1] = Pressure too low (HIGH)
	//alarmzz[2] = Patient is fighting (HIGH)
	//alarmzz[3] = Overcurrent fault (HIGH)
	//alarmzz[4] = Sporious breath (MEDIUM)
	//alarmzz[5] = Overtidal volume (MEDIUM)
	//alarmzz[6] = Low PEEP (MEDIUM)
	//alarmzz[7] =
	//alarmzz[8] = Low/Oversupply of Oxygen (LOW)

	int key_index = key.substring(0,2).toInt();
//  Serial.println(key_index);  // debugging
	if (key.substring(3) == "ON") {alarms[key_index] = 1;}
	else if (key_index == 99) {
		for (int j = 0; j<9; j++) {alarms[j] = 0;}
	}
	else {alarms[key_index] = 0;}

	String msg = "<";
	msg = msg + String(alarms[0]);
	for(int i = 1; i<9; i++) {
		msg = msg + "," + String(alarms[i]);
	}
	msg = msg + ">";

	Serial3.println(msg); Serial3.flush();
	Serial.println(String(key) + " | To alarm: " + String(msg)); Serial.flush();
}

//-- Servo to set Oxygen
void setServoOx(uint32_t Oxq){
	int sudut = map(Oxq, 0, 100, 0, 65);

	digitalWrite(enaServoOx, HIGH);
	servoOxigen.write(sudut);
	digitalWrite(enaServoOx, LOW);
}

//-- Servo to set PEEP opening
void setServoPEEP(uint32_t Vol){
	int sudut = round(cariBukaanPEEP(Vol));

	digitalWrite(enaServoPEEP, HIGH);
	servoPEEP.write(sudut);
	digitalWrite(enaServoPEEP, LOW);
}

//-- Lookup Table for Servo PEEP vs VOLUME
float cariBukaanPEEP(float vol_Tidal){
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
