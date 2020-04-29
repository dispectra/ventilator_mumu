#include <SoftwareSerial.h>

// PIN LIST
#define pinPEEP 2
#define pinIPP 3
#define pinPressure A0
#define pinFlow A1

#define pinPresWarn 4
#define pinPresHold 5
#define pinVolWarn 6
<<<<<<< HEAD
#define pinSpur 7
#define pinFight 8
=======
>>>>>>> 3f0841459144f100073dad03c15e66a059de2d12

// GLOBAL VARIABLES
float pressure_val, flow_val;
float pressure_raw, flow_raw;
int PEEP_lim, vol_lim, PIP_lim;
unsigned long now;
int vol_acc = 0;

bool readPEEP = false;
bool readIPP = false;
bool exhaleStage = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(pinPEEP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinPEEP), readPEEPQ, FALLING);
  pinMode(pinIPP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinIPP), readIPPQ, FALLING);
<<<<<<< HEAD

=======
  
>>>>>>> 3f0841459144f100073dad03c15e66a059de2d12
  now = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  readDataFromMega();

  pressure_raw = analogRead(pinPressure);
  flow_raw = analogRead(pinFlow);
  pressure_val = calcPressure(pressure_raw);
  flow_val = calcFlow(flow_raw);
<<<<<<< HEAD

  if(micros() - now > 100000){ //setiap 100 millis
    sendDataToMega(0);
  }

  if(readPEEP){
    sendDataToMega(1);
    digitalWrite(pinSpur, HIGH);
    digitalWrite(pinFight, HIGH);
    exhaleStage = false;
    vol_acc = 0;
  }

=======
  
  if(micros() - now > 100000){ //setiap 100 millis
    sendDataToMega(0);
  }
  
  if(readPEEP){
    sendDataToMega(1);
    exhaleStage = false;
    vol_acc = 0;
  }
  
>>>>>>> 3f0841459144f100073dad03c15e66a059de2d12
  if(readIPP){
    sendDataToMega(2);
    exhaleStage = true;
  }

  if(exhaleStage){ //fasa exhale
<<<<<<< HEAD
    //0. Cek Spurious
    if(spuriousDetect()){
      digitalWrite(pinSpur, LOW);
    }

=======
>>>>>>> 3f0841459144f100073dad03c15e66a059de2d12
    //1. Jaga pressure
    if(pressure_raw<=PEEP_lim){
      digitalWrite(pinPresHold, HIGH);
    } else {
      digitalWrite(pinPresHold, LOW);
    }
  } else { //fasa inhale
<<<<<<< HEAD
    //0. Cek Fighting
    if(fightingDetect()){
      digitalWrite(pinFight,LOW)
    }

=======
>>>>>>> 3f0841459144f100073dad03c15e66a059de2d12
    //1. Jaga Pressure
    if(pressure_raw > PIP_lim){
      digitalWrite(pinPresWarn, LOW);
      delayMicroseconds(10);
      digitalWrite(pinPresWarn, HIGH);
    }

    //2. Jaga Volume
    vol_acc += flow_raw;
    if(vol_acc> vol_lim){
      digitalWrite(pinVolWarn, LOW);
      delayMicroseconds(10);
      digitalWrite(pinVolWarn, HIGH);
    }
  }

<<<<<<< HEAD


=======
  
  
>>>>>>> 3f0841459144f100073dad03c15e66a059de2d12
}

//== FUNCTIONS ------------------------------------------------------

//- Interrupts
void readPEEPQ(){readPEEP = true;}
void readIPPQ(){readIPP = true;}

//- Calc Pressure and Flow from Callibration
float calcPressure(float pressure_rawq){
  float calc = 0.1095*pressure_rawq-4.6591;
  return calc;
}

float calcFlow(float flow_rawq){
  float calc = 5.7871*flow_rawq-248.76;
  return calc;
}

<<<<<<< HEAD
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


=======
>>>>>>> 3f0841459144f100073dad03c15e66a059de2d12
//- From/to Mega
void readDataFromMega(){
  // baca Vtidal, PEEP, PIP
  // Update nilai limit Vol_lim, PEEP_lim, PIP_lim
<<<<<<< HEAD

=======
  
>>>>>>> 3f0841459144f100073dad03c15e66a059de2d12
}

void sendDataToMega(int mode){
  // 0 : Normal
  // 1 : PEEP
  // 2 : IPP
}
