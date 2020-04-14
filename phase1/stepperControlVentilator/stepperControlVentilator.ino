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
#define dirInhale HIGH

//-- Input HMI ======================================================================
// volTidal = Volume Tidal (cc)
// IRat dan ERat = IERatio ( I : E )
// RR = Respiration Rate (x per minute)
float volTidal = 8;
int IRat = 1;
int ERat = 2;
int RR = 14;

//-- GLOBAL VARIABLEs ===============================================================
unsigned long stepTidal, delayInhale, delayExhale;
float timeInhale, timeExhale, IERatio;

//-- SETUP ==========================================================================
void setup() {
  Serial.begin(115200);
  
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  
  stepTidal = cekTidal(volTidal);
  timeInhale = (60000 / float(RR)) * (float(IRat) / float(IRat + ERat)); // dalam miliseconds
  timeExhale = (60000 / float(RR)) * (float(ERat) / float(IRat + ERat)); // dalam miliseconds
  delayInhale = float(timeInhale*1000) / float(stepTidal) / 2; // dalam microseconds
  delayExhale = float(timeExhale*1000) / float(stepTidal) / 2; // dalam microseconds

  pinMode(7, INPUT_PULLUP);
  
  Serial.println("Vol Tidal = " + String(volTidal));
  Serial.println("Step Tidal = " + String(stepTidal));
  Serial.println("WAKTU IDEAL Inhale = " + String(timeInhale));
  Serial.println("WAKTU IDEAL Exhale = " + String(timeExhale));
  Serial.println("DELAY Inhale = " + String(delayInhale));
  Serial.println("DELAY Exhale = " + String(delayExhale));
}

//-- LOOP ============================================================================
void loop() {
  if(digitalRead(7) == LOW){
    if (stepTidal > 0) {
      Inhale();
  //    delay(1000);
      Exhale();
  //    delay(1000);
  } else {
    //throw error message
    
  }
  }
  
}

//-- Lookup Table Volume Tidal vs Step yang diperlukan ================================
int cekTidal(float vol_Tidal){
  float lookup_vol[] = {6, 7, 8};
  int lookup_step[] = {3500, 8000, 8000};

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
  
  // 1. Set Arah
  digitalWrite(dirPin, dirInhale); 

  // 2. Set Gerakan Stepper
  if (delayInhale > 16383){
    for(int i = 0; i < stepTidal; i++) {
      digitalWrite(stepPin, HIGH);
      
      for(int j=0; j < delayInhale/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(delayInhale % 16383);
      
      digitalWrite(stepPin, LOW);
      
      for(int j=0; j < delayInhale/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(delayInhale % 16383);
    }
  } else {
    for(int i = 0; i < stepTidal; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(delayInhale);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(delayInhale);
    }
  }

  // 3. Tampil Waktu
  Serial.print("Waktu Inhale = ");
  Serial.println(micros() - now);
}

//-- Sekuens Exhale ====================================================================
void Exhale() {
  // 0. Hitung Waktu
  unsigned long now = micros();
  
  // 1. Set Arah
  digitalWrite(dirPin, !dirInhale); 

  // 2. Set Gerakan Stepper
  if (delayExhale > 16383){
    for(int i = 0; i < stepTidal; i++) {
      digitalWrite(stepPin, HIGH);
      
      for(int j=0; j < delayExhale/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(delayExhale % 16383);
      
      digitalWrite(stepPin, LOW);
      
      for(int j=0; j < delayExhale/16383; j++){
        delayMicroseconds(16383);
      }
      delayMicroseconds(delayExhale % 16383);
    }
  } else {
    for(int i = 0; i < stepTidal; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(delayExhale);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(delayExhale);
    }
  }
  
  // 3. Tampil Waktu
  Serial.print("Waktu Exhale = ");
  Serial.println(micros() - now);
}
