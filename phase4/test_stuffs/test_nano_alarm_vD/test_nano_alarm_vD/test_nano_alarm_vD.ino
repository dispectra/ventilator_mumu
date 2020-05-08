#include <SoftwareSerial.h>
SoftwareSerial SerialMega(11,12); // RX, TX


String command = "<0,0,0,0,0,0,0,0,0>";
bool alarm[] = {0,0,0,0,0,0,0,0,0};
String packet_buffer = "";
void getCommand() {
  String received = readMega();
  if (received[0] == '<') {packet_buffer = received;}
  else {packet_buffer += received;}
  
  if (packet_buffer[packet_buffer.length() - 1] == '>') {
    command = packet_buffer;

    // Parsing command
    String txt = packet_buffer.substring(1,packet_buffer.length()-1);
    int indexStart = 0;
    int indexEnd = 0;
    for(int i = 0; i<9; i++) {
      indexEnd = txt.indexOf(",", indexStart);
      alarm[i] = txt.substring(indexStart, indexEnd).toInt();
      indexStart = indexEnd+1;
      //Serial.println(String(i) + ": " + bufferq[i]);
    }
  }
  Serial.println(command); Serial.flush();
}


String readMega() {
  unsigned int timeout = 200;
  unsigned long time_begin = millis();
  bool quit = false;
  String packet = "";

  while (!quit){
    if (SerialMega.available() > 0){
      char x = SerialMega.read();
      if (x == '>'){
        packet += x;
        quit = true;
      } else {
        if (x == '<'){packet = "";}
        packet += x;
      }
    }
    if (millis() - time_begin > timeout) {quit = true;}
  }

  return packet;
}

#define BUZZER_0_PIN A0
unsigned long buzzer0_begin = 0;
bool buzzer0_current_state, buzzer0_prev_state = false;
void buzzer0() {
  unsigned long now = millis();
  unsigned int buzzer0_period = 1000;
  unsigned int buzzer0_phase = 0;
  
  if (buzzer0_current_state == true && buzzer0_prev_state == true) {
    buzzer0_phase = (now - buzzer0_begin)% buzzer0_period;
    if (buzzer0_phase < 250) {buzzer0_beep_off();}
    else if (buzzer0_phase < 500) {buzzer0_beep_on();}
    else if (buzzer0_phase < 750) {buzzer0_beep_off();}
    else {buzzer0_beep_on();}
  }
  else if (buzzer0_current_state == true && buzzer0_prev_state == false) {
    buzzer0_begin = millis();
  }
  else if (buzzer0_current_state == false && buzzer0_prev_state == true) {
    buzzer0_beep_off();
  }
  buzzer0_prev_state = buzzer0_current_state;
}
// Active buzzer
void buzzer0_beep_on() {digitalWrite(BUZZER_0_PIN, HIGH);}
void buzzer0_beep_off() {digitalWrite(BUZZER_0_PIN,LOW);}


#define BUZZER_1_PIN A1
unsigned long buzzer1_begin = 0;
bool buzzer1_current_state, buzzer1_prev_state = false;
void buzzer1() {
  unsigned long now = millis();
  unsigned int buzzer1_period = 1000;
  unsigned int buzzer1_phase = 0;
  
  if (buzzer1_current_state == true && buzzer1_prev_state == true) {
    buzzer1_phase = (now - buzzer1_begin)% buzzer1_period;
    if (buzzer1_phase < 500) {buzzer1_beep_off();}
    else {buzzer1_beep_on();}
  }
  else if (buzzer1_current_state == true && buzzer1_prev_state == false) {
    buzzer1_begin = millis();
  }
  else if (buzzer1_current_state == false && buzzer1_prev_state == true) {
    buzzer1_beep_off();
  }
  buzzer1_prev_state = buzzer1_current_state;
}
// Active buzzer
void buzzer1_beep_on() {digitalWrite(BUZZER_1_PIN, HIGH);}
void buzzer1_beep_off() {digitalWrite(BUZZER_1_PIN,LOW);}
//
//
//#define BUZZER_2_PIN 7
//unsigned long buzzer2_begin = 0;
//bool buzzer2_current_state, buzzer2_prev_state = false;
//void buzzer2() {
//  unsigned long now = millis();
//  unsigned int buzzer2_period = 1000;
//  unsigned int buzzer2_phase = 0;
//  
//  if (buzzer2_current_state == true && buzzer2_prev_state == true) {
//    buzzer2_phase = (now - buzzer2_begin)% buzzer2_period;
//    if (buzzer2_phase < 250) {buzzer2_beep_off();}
//    else if (buzzer2_phase < 500) {buzzer2_beep_on();}
//    else if (buzzer2_phase < 750) {buzzer2_beep_off();}
//    else {buzzer2_beep_on();}
//  }
//  else if (buzzer2_current_state == true && buzzer2_prev_state == false) {
//    buzzer2_begin = millis();
//  }
//  else if (buzzer2_current_state == false && buzzer2_prev_state == true) {
//    buzzer2_beep_off();
//  }
//  buzzer2_prev_state = buzzer2_current_state;
//}
//// Active buzzer
//void buzzer2_beep_on() {digitalWrite(BUZZER_2_PIN, HIGH);}
//void buzzer2_beep_off() {digitalWrite(BUZZER_2_PIN,LOW);}


#define SILENCE_BUTTON_PIN A3
#define ALARM_0_LED_PIN 2
#define ALARM_1_LED_PIN 3
#define ALARM_2_LED_PIN 4
#define ALARM_3_LED_PIN 5
#define ALARM_4_LED_PIN 6
#define ALARM_5_LED_PIN 7
#define ALARM_6_LED_PIN 8
#define ALARM_7_LED_PIN 9
#define ALARM_8_LED_PIN 10

bool silence_state = false;
unsigned long silence_begin = 0;
#define silence_timeout 20000

void setup() {
  // Serial debug begin
  Serial.begin(115200);
  
  // Serial from mega begin
  SerialMega.begin(115200);
  pinMode(SILENCE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ALARM_0_LED_PIN, OUTPUT);
  pinMode(ALARM_1_LED_PIN, OUTPUT);
  pinMode(ALARM_2_LED_PIN, OUTPUT);
  pinMode(ALARM_3_LED_PIN, OUTPUT);
  pinMode(ALARM_4_LED_PIN, OUTPUT);
  pinMode(ALARM_5_LED_PIN, OUTPUT);
  pinMode(ALARM_6_LED_PIN, OUTPUT);
  pinMode(ALARM_7_LED_PIN, OUTPUT);
  pinMode(ALARM_8_LED_PIN, OUTPUT);
  pinMode(BUZZER_0_PIN, OUTPUT);
  pinMode(BUZZER_1_PIN, OUTPUT);
}



void loop() {
  // Get command line from SerialMega
  getCommand();
  
  // Read alarm silence button
  if (!digitalRead(SILENCE_BUTTON_PIN)) {
    silence_state = true;
    silence_begin = millis();
  }
  
  // Silence timeout check
  if (millis() - silence_begin > silence_timeout) {silence_state = false;}
  Serial.println("Silence state is: " + String(silence_state));
  
  // Buzzer routine
  if (silence_state) {
    buzzer0_beep_off();
    buzzer1_beep_off();
//    buzzer2_beep_off();
  } else {
    // Read needed buzzer state
    buzzer0_current_state = alarm[0] || alarm[1] || alarm[2] || alarm[3];   // alarm type HIGH
    buzzer1_current_state = alarm[4] || alarm[5] || alarm[6] || alarm[7];   // alarm type MEDIUM
//    buzzer2_current_state = alarm[8];

    Serial.println("buzzer0 state: " + String(buzzer0_current_state));
    Serial.println("buzzer1 state: " + String(buzzer1_current_state));

    // Run buzzer 
    buzzer0();
    buzzer1();
//    buzzer2();
  }
  
  // Pass it to LED routine
  digitalWrite(ALARM_0_LED_PIN,alarm[0]);
  digitalWrite(ALARM_1_LED_PIN,alarm[1]);
  digitalWrite(ALARM_2_LED_PIN,alarm[2]);
  digitalWrite(ALARM_3_LED_PIN,alarm[3]);
  digitalWrite(ALARM_4_LED_PIN,alarm[4]);
  digitalWrite(ALARM_5_LED_PIN,alarm[5]);
  digitalWrite(ALARM_6_LED_PIN,alarm[6]);
  digitalWrite(ALARM_7_LED_PIN,alarm[7]);
  digitalWrite(ALARM_8_LED_PIN,alarm[8]);
}
