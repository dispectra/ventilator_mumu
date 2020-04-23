#define pinPressure A0
#define pinFlow A1

float lastValue = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
//  // Flow Naik Pressure Turun
//  while(analogRead(pinPressure)<41 && analogRead(pinFlow)>43){
//    digitalWrite(2, HIGH);
//  }

  // Filter https://www.megunolink.com/articles/coding/3-methods-filter-noisy-arduino-measurements/
  float x = 0.1095*analogRead(A0)-4.6591;
//  float y = digitalFilter(x);
  float y = 0.2*x + 0.8 * lastValue;
  Serial.print(y);
  Serial.print("\t");
  Serial.println(x);
  lastValue = y;
  delay(1);
}

float impulse[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float digitalFilter(float newImpulse) {
  // Variable declaration
  byte filterOrder = 5;
  float weight[] = {0.1, 0.2, 0.4, 0.2, 0.1};
  float respons = 0.0;

  // Updating impulses
  for (int i = 0; i<(filterOrder-1); i++) {
    impulse[i] = impulse[i+1];
  }
  impulse[filterOrder-1]=newImpulse;

  // Calculating response
  for (int i=0; i<filterOrder; i++) {
    respons += (impulse[i] * weight[i]);
//    Serial.println(impulse[i]);
  }

  return respons;
}
