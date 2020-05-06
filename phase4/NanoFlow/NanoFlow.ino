#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>

SoftwareSerial SerialM(11,12);
Adafruit_ADS1115 ads;

// PIN LIST
#define pinPEEP 2
#define pinIPP 3

#define pinVolWarn 6
#define pinSpur 7
#define pinFight 8

// GLOBAL VARIABLES
float flow_raw, flow_val;
int vol_lim;
unsigned long now;
double vol_acc = 0;

bool readPEEP = false;
bool readIPP = false;
bool exhaleStage = false;
bool updated = false;
String bufferq[2];
String lastData = "<0, 0>";
bool runningState;

unsigned long nowq = 0;
unsigned long dt = 0;
bool lastState = 0; //0 Inhale, 1 Exhale

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialM.begin(57600);

  ads.begin();
  ads.setGain(GAIN_SIXTEEN);

  pinMode(pinPEEP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinPEEP), readPEEPQ, FALLING);
  pinMode(pinIPP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinIPP), readIPPQ, FALLING);

  nowq = micros();
}

void loop() {
  //0. Read Serial from Mega
  readDataFromMega();

  //1. Read Flow Value
  flow_raw = ads.readADC_Differential_0_1();
  flow_val = calcFlow(flow_raw);

  //2. Remove Noise Values
  if(abs(flow_val) <= 1
      || abs(roundf(flow_val*100.0)/100.0) == 3.24
      || abs(roundf(flow_val*100.0)/100.0) == 0.81
      || abs(roundf(flow_val*100.0)/100.0) == 4.06
      || abs(roundf(flow_val*100.0)/100.0) == 4.87
      || abs(roundf(flow_val*100.0)/100.0) == 5.68
      || abs(roundf(flow_val*100.0)/100.0) == 2.43
      || abs(roundf(flow_val*100.0)/100.0) == 1.62
      ){flow_val=0;}
///  Serial.println(flow_val);

  //3. Check State
  if(runningState != 0){
    //0. Check for Inhale/Exhale Timing
    if(readPEEP){
      //reset stuffs
      digitalWrite(pinSpur, HIGH);
      vol_acc = 0;

      // indicate INHALE
      exhaleStage = false;

      // reset time and var
      nowq = micros();
      readPEEP = false;
    }

    if(readIPP){
      exhaleStage = true;
      readIPP = false;
    }


    //1. EXHALE ROUTINE
    if(exhaleStage){ //fasa exhale
      if(lastState == 0){
        //Serial.println("EXHALE STAGE");
        lastState = 1;
      }

      //0. Cek Spurious
      if(spuriousDetect()){
        digitalWrite(pinSpur, LOW);
      }
    }
    //2. INHALE ROUTINE
    else {
      if(lastState == 1){
        //Serial.println("INHALE STAGE");
        lastState = 0;
      }

      //1. Hitung Volume
      dt = micros()-nowq;
      vol_acc += flow_val/60 * dt/1000;
      nowq = micros();

      //2. Jaga Volume
      if(vol_acc> vol_lim){
        //Serial.println("WARNING VOLUME");
        digitalWrite(pinVolWarn, LOW);
        delayMicroseconds(10);
        digitalWrite(pinVolWarn, HIGH);
      }

      // KEPERLUAN AMBIL DATA --------------------------------------
      // Serial.print(flow_raw);
      // Serial.print("\t");
      // Serial.print(flow_val);
      // Serial.print("\t");
      // Serial.println(vol_acc);

      //Serial.println("====> TIME: " + String(dt));
      //Serial.println("VOL: " + String(vol_acc));
    }
  }
}

//== FUNCTIONS ------------------------------------------------------

//- Interrupts
void readPEEPQ(){readPEEP = true;}
void readIPPQ(){readIPP = true;}

//- Calc Flow from Callibration
float calcFlow(float flow_rawq){
  float calc = (90.1479*sqrt(flow_rawq)-5011.9318);

  return calc;
}

//- Detect Spurious
// !!HOMEWORK!!
bool spuriousDetect(){
  bool spurious = false;
   if(flow_val > 1){
     spurious = true;
   }
  return spurious;
}

//- From/to Mega
void readDataFromMega(){
  String received = listeningMega();

//  //Serial.println(updated);
	if(updated == false) {
		//Serial.print("Received: ");
		//Serial.println(received);
		Serial.flush();

		int indexStart = 0;
		int indexEnd = 0;
    //Serial.println("CHECK");Serial.flush();
		for(int i = 0; i<2; i++) {
      //Serial.println("CHECK" + String(i));Serial.flush();
			indexEnd = received.indexOf(",", indexStart);
			bufferq[i] = received.substring(indexStart, indexEnd);
			indexStart = indexEnd+1;
			//    //Serial.println(String(i) + ": " + bufferq[i]);
		}

		runningState = bufferq[0].toInt();
		vol_lim = bufferq[1].toInt();

		updated = true;
   //Serial.println("UPDATED");Serial.flush();
	}
}

String listeningMega(){
	bool quit = false;
	String seriesData = "";

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
	lastData = seriesData;

	//!! Dummy Data !!
	// seriesData = "<1, 300>";

//  String seriesData2 = ;

	return seriesData.substring(1,seriesData.length()-1);
}
