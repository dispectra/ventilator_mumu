// Perlu data kalibrasi jml step vs volume tidal

// -- KONFIGURASI PIN ===============================================================
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
#define dirPin 9
#define stepPin 3
#define microstepping 4
#define dirInhale LOW
#define limitSwitchIn 11
#define limitSwitchEx 12

// Slope2an
#define initDelay 600

//-- Input HMI ======================================================================
// volTidal = Volume Tidal (cc)
// IRat dan ERat = IERatio ( I : E )
// RR = Respiration Rate (x per minute)
float volTidal = 6;
int IRat = 1;
int ERat = 2;
int RR = 14;
float slopeFactor = 0;

//-- GLOBAL VARIABLEs ===============================================================
unsigned long stepTidal, delayInhale, delayExhale, timeInEx;
float timeInhale, timeExhale, IERatio, timeBreath;

//-- SETUP ==========================================================================
void setup() {
  Serial.begin(115200);
  
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  
  stepTidal = 600;
  slopeFactor = cekTidal(volTidal);
  timeBreath = (60000 / float(RR)) * 1000;
  timeInhale = (60000 / float(RR)) * (float(IRat) / float(IRat + ERat)) * 1000; // dalam microseconds
  timeExhale = (60000 / float(RR)) * (float(ERat) / float(IRat + ERat)) * 1000; // dalam microseconds
  delayInhale = 300; // dalam microseconds
  delayExhale = delayInhale; // dalam microseconds
  
//  timeInEx = stepTidal * delayInhale/; 

  pinMode(7, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(limitSwitchIn, INPUT_PULLUP);
  pinMode(limitSwitchEx, INPUT_PULLUP);
  
  Serial.println("----");
  Serial.println("Vol Tidal = " + String(volTidal));
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
}

//-- LOOP ============================================================================
void loop() {
  if(digitalRead(7) == LOW){
    if (stepTidal > 0) {
      unsigned long now = micros();
      
      Inhale();

      while((micros()-now) < timeInhale){
        delayMicroseconds(1);
      }

      unsigned long timeInhaleReal = micros()-now;
      Serial.println("==> TIME INHALE : " + String(timeInhaleReal));
      
      Exhale();
      
      while((micros()-now) < timeBreath){
        delayMicroseconds(1);
      }
      Serial.println("==> TIME EXHALE : " + String(micros()-now - timeInhaleReal));
      Serial.println("TIME TAKEN : " + String(micros() - now));
      Serial.println("----");
    } else {
      //throw error message
 
    }
  } else if(digitalRead(4) == LOW){
      digitalWrite(dirPin, LOW); 
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
  } else if(digitalRead(5) == LOW){
      digitalWrite(dirPin, HIGH); 
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
  }
//  
}

//-- Lookup Table Volume Tidal vs Step yang diperlukan ================================
float cekTidal(float vol_Tidal){
  float lookup_vol[] = {6, 7, 8};
  float lookup_step[] = {0.5, 0.25, 0.35};

  float stepTidal = 0;
  int arraySize = sizeof(lookup_vol) / sizeof(lookup_vol[0]);

  // Extrapolasi Bawah
  if(vol_Tidal < cariMin(lookup_vol, arraySize)){
    float m = float(lookup_step[1] - lookup_step[0]) / (lookup_vol[1] - lookup_vol[0]);
    float c = float(lookup_step[0]) - lookup_vol[0] * m;
    stepTidal = m * vol_Tidal + c;
  } 
  // Extrapolasi Atas
  else if(vol_Tidal > cariMax(lookup_vol, arraySize)){
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
  for(int i=0; i< arraySize; i++){
    if(list[i] <= mini) {
      mini = list[i];
    }
  }
  return mini;
}

float cariMax(float list[], int arraySize){
  float maxi = 0;
  for(int i=0; i< arraySize; i++){
    if(list[i] >= maxi) {
      maxi = list[i];
    }
  }
  return maxi;
}


//-- Sekuens Inhale ====================================================================
void Inhale() {
  // 0. Hitung Waktu
  unsigned long now = micros();
  float delayInhale2 = initDelay;
  // 1. Set Arah
  digitalWrite(dirPin, dirInhale);
  delayMicroseconds(5);

  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidal; i++) {
    if(i < slopeFactor*stepTidal){
      delayInhale2 -= (initDelay-delayInhale) / (slopeFactor*stepTidal);
    }
    if(i>(1-slopeFactor)*stepTidal){
      delayInhale2 += (initDelay-delayInhale) / (slopeFactor*stepTidal);
    }
    
//    Serial.println(delayInhale2);/

    if(digitalRead(limitSwitchIn)){
      digitalWrite(stepPin, HIGH);
    }
    delayMicroseconds(delayInhale2);
    
    if(digitalRead(limitSwitchIn)){
      digitalWrite(stepPin, LOW);
    }
    delayMicroseconds(delayInhale2);
  }

  // 3. Tampil Waktu
  Serial.print("Waktu Inhale = ");
  Serial.println(micros() - now);
}

//-- Sekuens Exhale ====================================================================
void Exhale() {
  // 0. Hitung Waktu
  unsigned long now = micros();
  float delayExhale2 = initDelay;
  
  // 1. Set Arah
  digitalWrite(dirPin, !dirInhale);
  delayMicroseconds(5); 

  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidal; i++) {
    if(i < slopeFactor*stepTidal){
      delayExhale2 -= (initDelay-delayExhale) / (slopeFactor*stepTidal);
    }
    if(i>(1-slopeFactor)*stepTidal){
      delayExhale2 += (initDelay-delayExhale) / (slopeFactor*stepTidal);
    }
    if(digitalRead(limitSwitchEx)){
      digitalWrite(stepPin, HIGH);
    }
    delayMicroseconds(delayExhale2);
    
    if(digitalRead(limitSwitchEx)){
      digitalWrite(stepPin, LOW);
    }

    delayMicroseconds(delayExhale2);
  }
  
  // 3. Tampil Waktu
  Serial.print("Waktu Exhale = ");
  Serial.println(micros() - now);
}
