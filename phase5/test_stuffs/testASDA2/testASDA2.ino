#define ENA 4
#define PUL 5
#define DIR 6

#define Bt1 A0
#define Bt2 A1
#define Bt3 A2

String lastData = "<2000>";
int stateq = 2;
int stepq = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ENA, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

  pinMode(Bt1, INPUT_PULLUP);
  pinMode(Bt2, INPUT_PULLUP);
  pinMode(Bt3, INPUT_PULLUP);
//  pinMode(Lt, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("READY...");
}

int delayq = 2000;

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(ENA, HIGH);
  if(digitalRead(A2) == LOW){
    while(digitalRead(A2) == LOW){
      delayq = listeningMega().toInt();
    }
    Serial.println("Delay = " + String(delayq));
  }
  if(digitalRead(Bt1) == LOW){
    digitalWrite(DIR, HIGH);
    Serial.println("MAJU!");
//    if(stateq != 1){
//      stepq = 0;
//      stateq = 1;
//    }
    while(digitalRead(Bt1) == LOW){
      digitalWrite(PUL, HIGH);
      delayMicroseconds(delayq);
      digitalWrite(PUL, LOW);
      delayMicroseconds(delayq);
//      stepq +=1;
//      Serial.println("Step = " + String(stepq));
    }
  } else if(digitalRead(Bt2) == LOW){
    digitalWrite(DIR, LOW);
    Serial.println("MUNDUR!");
//    if(stateq != 0){
//      stepq = 0;
//      stateq = 0;
//    }
    while(digitalRead(Bt2) == LOW){
      digitalWrite(PUL, HIGH);
      delayMicroseconds(delayq);
      digitalWrite(PUL, LOW);
      delayMicroseconds(delayq);
//      stepq -=1;
//      Serial.println("Step = " + String(stepq));
    }
  }
}

String listeningMega(){
  bool quit = false;
  String seriesData = lastData;

  while (!quit) {
//    Serial.print(Serial.available());
    if (Serial.available() > 0) {
      char x = Serial.read();
      if (x == '>') {
        seriesData += x;
        quit = true;
      } else {
        if (x == '<') {seriesData = "";}
        seriesData += x;
      }
    } 
    else {
      seriesData = lastData;
      quit = true;
    }
  }
  lastData = seriesData;

  //!! Dummy Data !!
  // seriesData = "<1, 300>";

//  String seriesData2 = ;
Serial.println(seriesData);
//  Serial.println(seriesData.substring(1,seriesData.length()-1));
  return seriesData.substring(1,seriesData.length()-1);
}
