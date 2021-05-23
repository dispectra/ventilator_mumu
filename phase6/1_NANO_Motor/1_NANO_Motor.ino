// Perlu data kalibrasi jml step vs volume tidal

// -- KONFIGURASI PIN ===============================================================
// ENA- NC
// ENA+ NC
// DIR- GND
// DIR+ DIR
// PUL- GND
// PUL+ PUL

//-- TWEAKABLES =====================================================================
// DIR = pin DIR+
// PUL = pin PUL+
// microPULg = settingan microPULg (1 / 2 / 4 / 8 / 16)
// dirInhale = arah untuk inhale (HIGH / LOW)
#define ENA 4
#define PUL 5
#define DIR 6

#define offsetq 8

#define Bt1 A0
#define Bt2 A1
#define Bt3 A2
#define Bt4 A4

#define dirInhale HIGH
int stateq = 2;
int stepq = 0;
int delayq = 2000;

//-- Input HMI ======================================================================
// volTidal = Volume Tidal (cc)
// IRat dan ERat = IERatio ( I : E )
// RR = Respiration Rate (x per minute)
float volTidal = 0;
int IRat = 1;
float ERat = 1;
int RR = 10;

//-- GLOBAL VARIABLEs ===============================================================
unsigned long stepTidal, delayInhale, delayExhale, timeInEx;
float timeInhale, timeExhale, IERatio, timeBreath, slopeFactor;
float initDelay = 25;
unsigned long p_vol, p_RR;
float p_IE;
int p_mode = 0;
float p_del = 0;

int num_buf = 5;
String bufferq[5];
String lastData = "<0,0,0,0,0>";
bool updated = false;


//-- SETUP ==========================================================================
void setup() {
  Serial.begin(115200);
  
  slopeFactor = 0.35;
  updateParam(volTidal, RR, ERat);
  
//  timeInEx = stepTidal * delayInhale/; 

  pinMode(ENA, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

  pinMode(Bt1, INPUT_PULLUP);
  pinMode(Bt2, INPUT_PULLUP);
  pinMode(Bt3, INPUT_PULLUP);
  pinMode(Bt4, INPUT_PULLUP);
}

//-- LOOP ============================================================================
void loop() {
  updateAllGlobal();
  
  if(p_mode!=0){
    if (stepTidal > 0) {
      unsigned long now = micros();
      
      Inhale();
      
      Serial.println("{ei}"); Serial.flush();
      delayMicroseconds(100);
      
      Exhale();
      
      while((micros()-now) < timeBreath){}
      
      Serial.println("{ec}"); Serial.flush();
    } else {
      //throw error message
    }
  } else {
//    Serial.println("Waiting.. (mode 0 )");
    Serial.flush();
    delay(1000);
    if(digitalRead(Bt1) == LOW){
      digitalWrite(DIR, dirInhale);
      Serial.println("MAJU!");
      while(digitalRead(Bt1) == LOW){
        digitalWrite(PUL, HIGH);
        delayMicroseconds(delayq);
        digitalWrite(PUL, LOW);
        delayMicroseconds(delayq);
      }
    } else if(digitalRead(Bt2) == LOW){
      digitalWrite(DIR, !dirInhale);
      Serial.println("MUNDUR!");
      while(digitalRead(Bt2) == LOW){
        digitalWrite(PUL, HIGH);
        delayMicroseconds(delayq);
        digitalWrite(PUL, LOW);
        delayMicroseconds(delayq);
      }
    }
  }
}

void cekNewParam(){
  String received = listeningMega();
  if(!updated) {
    Serial.print("Received: ");
    Serial.println(received);
    Serial.flush();


    int indexStart = 0;
    int indexEnd = 0;

    for(int i = 0; i<num_buf; i++) {
      indexEnd = received.indexOf(",", indexStart);
      bufferq[i] = received.substring(indexStart, indexEnd);
      indexStart = indexEnd+1;
    }

    p_mode = bufferq[0].toInt();
    p_vol = bufferq[1].toInt();
    p_RR = bufferq[2].toInt();
    p_IE = bufferq[3].toFloat();
    p_del = bufferq[4].toFloat();

    updateParam(p_vol, p_RR, p_IE);
    
    updated = true;
  }
}

void updateAllGlobal(){
  cekNewParam();
}

String listeningMega(){
  bool quit = false;
  String seriesData = "";

  while (!quit) {
    if (Serial.available() > 0) {
      updated = false;
      char x = Serial.read();
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
  
//  if (Serial.available() > 0) {
//    updated = false;
//    seriesData = Serial.readString();
//    quit = true;
//  } else {
//    seriesData = lastData;
//    quit = true;
//  }
//    
//  if(seriesData == lastData) {
//    updated = true;
//  }

  lastData = seriesData;

  return seriesData.substring(1,seriesData.length()-1);
}

void updateParam(float vol, int RRq, float ERatq){
  stepTidal = cekTidal(vol);
  timeBreath = (60000 / float(RRq)) * 1000;
  timeInhale = (60000 / float(RRq)) * (float(IRat) / float(IRat + ERatq)) * 1000 - 500; // dalam microseconds

  float offsetInhaleq= 0;
  
//  if(p_ntab == 3){
//    offsetInhaleq = -0.018480414*RRq-0.125672596*ERatq+0.520717433;
//  }
//  else if(p_ntab==4){
//    offsetInhaleq = -0.017318122*RRq-0.130474105*ERatq+0.449587999;
//  } else if(p_ntab==5){
//    offsetInhaleq = -0.011070931*RRq-0.14822688*ERatq+0.367918135;
//  }else if(p_ntab==6){
//    offsetInhaleq = -0.007851635*RRq-0.174476163*ERatq+0.363243903;
//  }

  timeInhale += p_del/2*1000000;
  
  timeExhale = (60000 / float(RRq)) * (float(ERatq) / float(IRat + ERatq)) * 1000; // dalam microseconds
  delayInhale = float(timeInhale) / float(stepTidal) / 2; //endDelay; // dalam microseconds
  delayExhale = 20; //delayInhale; // dalam microseconds

  Serial.println("----");
  Serial.println("RR = " + String(RRq));
  Serial.println("IE = " + String(ERatq));
  Serial.println("Vol Tidal = " + String(vol));
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
  
  Serial.println("updateParam()");
  Serial.flush();
}

//-- Lookup Table Volume Tidal vs Step yang diperlukan ================================
float cekTidal(float vol_Tidal){
  float lookup_vol[] =  { 235,  250,   279,  308,  322,  352,  382,  410,  442,  476,  484,  494,  526,  560,  579,  590,  622,  647};
  float lookup_step[] = {3500, 3600,  3850, 3900, 4000, 4200, 4400, 4600, 4800, 5000, 5100, 5200, 5400, 5600, 5700, 5800, 6000, 6200};
  
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
//  return stepTidal;
  return vol_Tidal*10;
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
//  unsigned long now = micros();
//  float delayInhale2 = delayInhale-offsetq;
  bool ntab = false;
  float axq;
  float delayInhale2 = delayInhale-offsetq;
  
//  if(!p_ntab){
//    ntab = false;
//    Serial.println("NotNtab");Serial.flush();
//  } else {
//    delayInhale2 = 0.7*delayInhale;
//    axq = 1/(0.4*stepTidal-1) * (1.5)*delayInhale; //1.38
//    Serial.println("Ntab");Serial.flush();
//    Serial.println(axq);
//  }
  
//  if(ntab){}/
  
  // 1. Set Arah
  digitalWrite(DIR, dirInhale);
  delayMicroseconds(5);
  
  // 2. Set Gerakan Stepper
  if(ntab){
    for(int i = 0; i < stepTidal; i++) {
      if(i>0.6*stepTidal){
        delayInhale2 += axq;
      }
      digitalWrite(PUL, HIGH);
      delayMicroseconds(delayInhale2);
      digitalWrite(PUL, LOW);
      delayMicroseconds(delayInhale2);
    }
  } else {
    for(int i = 0; i < stepTidal; i++) {
    digitalWrite(PUL, HIGH);
    delayMicroseconds(delayInhale2);
    digitalWrite(PUL, LOW);
    delayMicroseconds(delayInhale2);
  }
  }
  // 3. Tampil Waktu
//  Serial.print("Waktu Inhale = ");
//  Serial.println(micros() - now);
}

//-- Sekuens Exhale ====================================================================
void Exhale() {
  // 0. Hitung Waktu
  unsigned long now = micros();
  float delayExhale2 = initDelay;
  
  // 1. Set Arah
  digitalWrite(DIR, !dirInhale);
  delayMicroseconds(5); 

  // 2. Set Gerakan Stepper
  for(int i = 0; i < stepTidal; i++) {
    if(i < slopeFactor*stepTidal){
      delayExhale2 -= (initDelay-delayExhale) / (slopeFactor*stepTidal);
    }
    if(i>(1-slopeFactor)*stepTidal){
      delayExhale2 += (initDelay-delayExhale) / (slopeFactor*stepTidal);
    }
//    if(digitalRead(limitSwitchEx)){
    digitalWrite(PUL, HIGH);
//    }

    delayMicroseconds(delayExhale2);
    
//    if(digitalRead(limitSwitchEx)){
    digitalWrite(PUL, LOW);
//    }

    delayMicroseconds(delayExhale2);
  }
  
  // 3. Tampil Waktu
//  Serial.print("Waktu Exhale = ");
//  Serial.println(micros() - now);
}
