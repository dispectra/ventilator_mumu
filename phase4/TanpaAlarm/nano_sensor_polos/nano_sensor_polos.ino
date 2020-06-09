#include <ArduinoSort.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>


SoftwareSerial SerialM(11,12);
Adafruit_ADS1115 ads;

// PIN LIST
#define pinPEEP 3
#define pinIPP 2

#define pinVolWarn 6
#define pinSpur 7
#define pinFight 8

// GLOBAL VARIABLES
double flow_raw, flow_val;
double flow_val2, last_val;
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
float offset = 0;
bool warned = false;
int buffsize = 50;

bool flag_delayFlow = true;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialM.begin(57600);

  ads.begin();
//  ads.setGain(GAIN_SIXTEEN);

  pinMode(pinPEEP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinPEEP), readPEEPQ, FALLING);
  pinMode(pinIPP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinIPP), readIPPQ, FALLING);

  pinMode(pinVolWarn, OUTPUT);
  pinMode(pinSpur, OUTPUT);

  digitalWrite(pinVolWarn, HIGH);
  digitalWrite(pinSpur, HIGH);
  nowq = micros();

  zeroFlowSensor();

  Serial.println("==> READY NANO SENSOR (POLOS)");
}

void loop() {
  //0. Read Serial from Mega
  readDataFromMega();

  //1. Check State
  if(runningState != 0){
    //0a. Read Flow Value
    flow_raw = ads.readADC_SingleEnded(0);
    flow_val =  calcFlow(flow_raw) + offset;

    //0b. Remove Noise Values
    if(abs(flow_val) <= 1
        || abs(roundf(flow_val*100.0)/100.0) == 0.11
        || abs(roundf(flow_val*100.0)/100.0) == 0.22
        || abs(roundf(flow_val*100.0)/100.0) == 0.32
        || abs(roundf(flow_val*100.0)/100.0) == 0.43
        || abs(roundf(flow_val*100.0)/100.0) == 0.54
        || abs(roundf(flow_val*100.0)/100.0) == 0.65
        || abs(roundf(flow_val*100.0)/100.0) == 0.75
        ){flow_val=0;}

  ///  Serial.println(flow_val);
    flow_val2 = 0.5*flow_val + 0.5*last_val;
    last_val = flow_val;

    //1. Check for Inhale/Exhale Timing
    if(readPEEP){
      //reset stuffs
      digitalWrite(pinSpur, HIGH);
      vol_acc = 0;

      // indicate INHALE
      exhaleStage = false;
      Serial.println("---------------------------------------------");

      // reset time and var
      nowq = micros();
      readPEEP = false;
    }

    if(readIPP){
      exhaleStage = true;
      readIPP = false;
    }


    //2. EXHALE ROUTINE
    if(exhaleStage){ //fasa exhale
      if (flag_delayFlow) {
        unsigned long nowa = millis();
        while(millis()-nowa < 500) {
          flow_raw = ads.readADC_SingleEnded(0);
          flow_val = calcFlow(flow_raw) + offset;
        }
        flag_delayFlow = false;}
      digitalWrite(pinVolWarn, HIGH);
//      Serial.println(flow_val);
      if(lastState == 0){
//        Serial.println("EXHALE STAGE");
        lastState = 1;
      }

      //0. Cek Spurious
      if(spuriousDetect()){
        Serial.println("-------------spurious");
//        digitalWrite(pinSpur, LOW);
//        delay(1000000);
      } else {digitalWrite(pinSpur,HIGH); };

      warned = false;
    }
    //3. INHALE ROUTINE
    else {
      flag_delayFlow = true;
      if(lastState == 1){
//        Serial.println("INHALE STAGE");/
        lastState = 0;
      }

      //1. Hitung Volume
      dt = micros()-nowq;
      vol_acc += flow_val2/60 * dt/1000;
      nowq = micros();

      //2. Jaga Volume
      if(vol_acc> vol_lim && !warned){
        Serial.println("WARNING VOLUME");
//        digitalWrite(pinVolWarn, LOW);
        delayMicroseconds(1000);
        digitalWrite(pinVolWarn, HIGH);
        warned = true;
      }
//
//      Serial.println("====> TIME: " + String(dt));
//      Serial.println("Flow: " + String(flow_val));
//      Serial.println("VOL: " + String(vol_acc));
    }

         // KEPERLUAN AMBIL DATA --------------------------------------
       Serial.print(flow_raw);
       Serial.print("\t");
       Serial.print(flow_val);
       Serial.print("\t");
       Serial.println(vol_acc);
    
  } else { //OFF Condition
    readPEEP = true;
    readIPP = false;
    zeroFlowSensor();
    digitalWrite(pinSpur, HIGH);

  //0a. Read Flow Value
    flow_raw = ads.readADC_SingleEnded(0);
    flow_val = calcFlow(flow_raw) + offset;

    //0b. Remove Noise Values
    if(abs(flow_val) <= 1
        || abs(roundf(flow_val*100.0)/100.0) == 3.24
        || abs(roundf(flow_val*100.0)/100.0) == 3.89
        || abs(roundf(flow_val*100.0)/100.0) == 3.12
        || abs(roundf(flow_val*100.0)/100.0) == 3.90
        || abs(roundf(flow_val*100.0)/100.0) == 0.81
        || abs(roundf(flow_val*100.0)/100.0) == 4.06
        || abs(roundf(flow_val*100.0)/100.0) == 4.87
        || abs(roundf(flow_val*100.0)/100.0) == 5.68
        || abs(roundf(flow_val*100.0)/100.0) == 2.34
        || abs(roundf(flow_val*100.0)/100.0) == 1.62
        || abs(roundf(flow_val*100.0)/100.0) == 1.63
        || abs(roundf(flow_val*100.0)/100.0) == 1.56
        ){flow_val=0;}
    Serial.println(flow_val);
    // delay(10);
  }
}

//== FUNCTIONS ------------------------------------------------------
void zeroFlowSensor(){
  //0. Create buffer for value and histogram
  float val[buffsize];
  float lastVal;
  int index_terpilih = 0;
  int mode_count = 0;
  int valcount = 0;

  //1. Ambil x data
  for(int i=0; i<buffsize; i++){
    val[i] = calcFlow(ads.readADC_SingleEnded(0))+offset;
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

//- Interrupts
void readPEEPQ(){readPEEP = true;}
void readIPPQ(){readIPP = true;}

//- Calc Flow from Callibration
float calcFlow(float flow_rawq){
  float calc = (25.188*sqrt(flow_rawq)-2915.7);

  return calc;
}

//- Detect Spurious
// !!HOMEWORK!!
bool spuriousDetect(){
  bool spurious = false;
   if(flow_val > 4.5){
     spurious = true;
   }
  return spurious;
}

//- From/to Mega
void readDataFromMega(){
  String received = listeningMega();

//  //Serial.println(updated);
  if(updated == false) {
    Serial.print("Received: ");
    Serial.println(received);
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
