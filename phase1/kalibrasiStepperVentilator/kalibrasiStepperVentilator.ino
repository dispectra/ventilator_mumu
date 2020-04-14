#define dirPin 9
#define stepPin 3
#define button_in 4
#define button_ex 5
#define reset 6
#define reset2 7

int i = 0;
int k = 0;
int l = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  pinMode(button_in, INPUT_PULLUP);
  pinMode(button_ex, INPUT_PULLUP);
  pinMode(reset, INPUT_PULLUP);
  pinMode(reset2, INPUT_PULLUP);

  Serial.println("SIAP!");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(!digitalRead(button_in)) {
    digitalWrite(dirPin, HIGH); 
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);
    i +=1;
    k = 0;
    l = 0;
    Serial.println("INHALE = " + String(i));
  } else if(!digitalRead(button_ex)) {
    digitalWrite(dirPin, LOW); 
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);
    i -=1;
    k = 0;
    l = 0;
    Serial.println("INHALE = " + String(i));
  } else if (!digitalRead(reset) && k == 0) {
    Serial.println("==> RESET POS");
    for(int j=0; j< i; j++) {
      digitalWrite(dirPin, LOW); 
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }
    k = 1;
    i = 0;
    l = 0;
  } 
  else if (!digitalRead(reset2) && l == 0) {
    i = 0;
    k = 0;
    l = 1;
    Serial.println("==> RESET");
  }
}
