//di program terpisah, non integrasi
//nano menggunakan yang ada di NanoAlarm_vD

bool alarms[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

#define MEGA_DO_0 44
#define MEGA_DO_1 45
#define MEGA_DO_2 46
#define MEGA_DO_3 47
#define MEGA_DO_4 48
#define MEGA_DO_5 49
#define MEGA_DO_6 50
#define MEGA_DO_7 51

void setup() {
  Serial.begin(115200);

  pinMode(MEGA_DO_0, OUTPUT);
  pinMode(MEGA_DO_1, OUTPUT);
  pinMode(MEGA_DO_2, OUTPUT);
  pinMode(MEGA_DO_3, OUTPUT);
  pinMode(MEGA_DO_4, OUTPUT);
  pinMode(MEGA_DO_5, OUTPUT);
  pinMode(MEGA_DO_6, OUTPUT);
  pinMode(MEGA_DO_7, OUTPUT);
  setAlarm("99_OFF");
}

void loop() {
  for (int i = 0; i < 8; i++) {
    String keyword = "0" + String(i) + "_ON";
    setAlarm(keyword);
    delay(2000);
    setAlarm("99_OFF");
    delay(2000);
  }
}

void setAlarm(String key) {   // Key example: 01_ON   ;   09_OFF
  //Define array of alarm triggers
  //alarmzz[0] = High pressure exceeded PIP (HIGH)
  //alarmzz[1] = Pressure too low (HIGH)
  //alarmzz[2] = Patient is fighting (HIGH)
  //alarmzz[3] = Overcurrent fault (HIGH)
  //alarmzz[4] = Sporious breath (MEDIUM)
  //alarmzz[5] = Overtidal volume (MEDIUM)
  //alarmzz[6] = Low PEEP (MEDIUM)
  //alarmzz[7] =
  //alarmzz[8] = Low/Oversupply of Oxygen (LOW)

  int key_index = key.substring(0, 2).toInt();
  //  Serial.println(key_index);  // debugging
  if (key.substring(3) == "ON") {
    alarms[key_index] = 1;
  }
  else if (key_index == 99) {
    for (int j = 0; j < 9; j++) {
      alarms[j] = 0;
    }
  }
  else {
    alarms[key_index] = 0;
  }

  String msg = "<";
  msg = msg + String(alarms[0]);
  for (int i = 1; i < 9; i++) {
    msg = msg + "," + String(alarms[i]);
  }
  msg = msg + ">";

  //  Serial3.println(msg); Serial3.flush();

  // Alternative communication using digitalWrite
  digitalWrite(MEGA_DO_0, !(alarms[0]));
  digitalWrite(MEGA_DO_1, !(alarms[1]));
  digitalWrite(MEGA_DO_2, !(alarms[2]));
  digitalWrite(MEGA_DO_3, !(alarms[3]));
  digitalWrite(MEGA_DO_4, !(alarms[4]));
  digitalWrite(MEGA_DO_5, !(alarms[5]));
  digitalWrite(MEGA_DO_6, !(alarms[6]));
  digitalWrite(MEGA_DO_7, !(alarms[7]));

  Serial.println(msg);
}