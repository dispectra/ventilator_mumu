#include <SoftwareSerial.h>

SoftwareSerial SerialM(11,12);


// PIN LIST
#define pinPEEP 2
#define pinIPP 3
#define pinPressure A0
#define pinFlow A1

#define pinPresWarn 4
#define pinPresHold 5
#define pinVolWarn 6
#define pinSpur 7
#define pinFight 8

// GLOBAL VARIABLES
float pressure_val, flow_val;
float pressure_raw, flow_raw;
int PEEP_lim, vol_lim, PIP_lim;
unsigned long now;
int vol_acc = 0;

bool readPEEP = false;
bool readIPP = false;
bool exhaleStage = false;
bool updated = false;
String bufferq[2];
String lastData = "<0,0>";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialM.begin(57600);

  pinMode(pinPEEP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinPEEP), readPEEPQ, FALLING);
  pinMode(pinIPP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinIPP), readIPPQ, FALLING);

  now = micros();
}

unsigned long nowq = 0;
unsigned long dt = 0;

void loop() {
  // put your main code here, to run repeatedly:
  readDataFromMega();

  flow_raw = analogRead(pinFlow);
  flow_val = calcFlow(flow_raw);

  if(readPEEP){
    digitalWrite(pinSpur, HIGH);
    digitalWrite(pinFight, HIGH);
    exhaleStage = false;
    vol_acc = 0;
  }

  if(readIPP){
    exhaleStage = true;
  }

  if(exhaleStage){ //fasa exhale
    //0. Cek Spurious
    if(spuriousDetect()){
      digitalWrite(pinSpur, LOW);
    }
  } else { //fasa inhale
    //0. Cek Fighting
    if(fightingDetect()){
      digitalWrite(pinFight,LOW);
    }

    //1. Jaga Volume
    dt = millis()-nowq;
    vol_acc += flow_raw * dt;
    nowq = millis();
//    Serial.println(vol_acc);
    if(vol_acc> vol_lim){
      digitalWrite(pinVolWarn, LOW);
      delayMicroseconds(10);
      digitalWrite(pinVolWarn, HIGH);
    }
  }
}

//== FUNCTIONS ------------------------------------------------------

//- Interrupts
void readPEEPQ(){readPEEP = true;}
void readIPPQ(){readIPP = true;}

//- Calc Flow from Callibration
float calcFlow(float flow_rawq){
  float calc = 0.8366*flow_rawq-2424.1563;
  return calc;
}

//- Detect Spurious
bool spuriousDetect(){
  bool spurious = false;
  // if(){
  //   spurious = true;
  // }
  return spurious;
}

//- Detect Fighting
bool fightingDetect(){
  bool fight = false;
  // if(){
  //   spurious = true;
  // }
  return fight;
}


//- From/to Mega
void readDataFromMega(){
//  Serial.println("PING");
  // baca Vtidal, PEEP, PIP
  // Update nilai limit Vol_lim, PEEP_lim, PIP_lim
  String received = listeningMega();
  if(received="0,0"){updated=true;};
  Serial.println(updated);
	if(updated == false) {
		Serial.print("Received: ");
		Serial.println(received);
		Serial.flush();
   delay(1000);

		int indexStart = 0;
		int indexEnd = 0;

		for(int i = 0; i<7; i++) {
			indexEnd = received.indexOf(",", indexStart);
			bufferq[i] = received.substring(indexStart, indexEnd);
			indexStart = indexEnd+1;
			//    Serial.println(String(i) + ": " + bufferq[i]);
		}

		vol_lim = bufferq[0].toInt();
		PEEP_lim = bufferq[1].toInt();
		PIP_lim = 35;

		updated = true;
	}
}

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
	lastData = seriesData;

	//!! Dummy Data !!
//	seriesData = "<1,0,0,350,2,14,0>";

//  String seriesData2 = ;

	return seriesData.substring(1,seriesData.length()-1);
}
