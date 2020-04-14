// Perlu data kalibrasi jml step vs volume tidal

// -- KONFIGURASI PIN
// ENA- NC
// ENA+ NC
// DIR- GND
// DIR+ dirPin
// PUL- GND
// PUL+ stepPin

//-- TWEAKABLES
// dirPin = pin DIR+
// stepPin = pin PUL+
// microstepping = settingan microstepping (1 / 2 / 4 / 8 / 16)
// volTidal = Volume Tidal (cc)
// timeInhale = waktu total inhale (dalam ms, bisa koma)
// IRat dan ERat = IERatio ( I : E )
// RR = Respiration Rate (x per minute)
// dirInhale = arah untuk inhale (HIGH / LOW)
#define dirPin 2
#define stepPin 3
#define microstepping 1
#define volTidal 6
#define IRat 1
#define ERat 2
#define RR 14
#define dirInhale HIGH

//-- CONSTANTS
const float timeInhale = RR/60 / (IRat + ERat);
const float IERatio = IRat / ERat;
const int delayInhale = timeInhale*1000 / stepTidal; // dalam microsecond
const int delayExhale = delayInhale / IERatio; 

//-- GLOBAL VARIABLEs
int stepTidal = 0;

//-- SETUP
void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  cekTidal(volTidal);
}

//-- LOOP
void loop() {
  if (stepTidal > 0) {
    Inhale();
    
    Exhale();
  } else {
    //throw error message
  }
  
}

//-- Lookup Table Volume Tidal vs Step yang diperlukan
void cekTidal(int volTidal){
  switch(volTidal){
    case 6:
      stepTidal = 50;
      break;
    case 7:
      stepTidal = 80;
      break;
    default:
      stepTidal = 0;
      break;
  }
}

//-- Sekuens Inhale
void Inhale() {  
  // 1. Set Arah
  digitalWrite(dirPin, dirInhale); 
  
  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidal; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayInhale);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayInhale);
  }
}

//-- Sekuens Exhale
void Exhale() {
  // 1. Set Arah
  digitalWrite(dirPin, !dirInhale); 
  
  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidal; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayExhale);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayExhale);
  }
}
