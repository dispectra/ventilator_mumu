/* Arduino Mega
 * Serial communication to/from Arduino Nano through Serial1
 */

void setup() {
  Serial.begin(115200);
  Serial2.begin(38400);
}

void loop() {
  Serial.println("Write to Nano..."); Serial.flush();
  Serial2.print("<02,04,1998>"); Serial2.flush();
  Serial.println("Write to Nano...done"); Serial.flush();
//  delay(10);
  Serial.println("eol"); Serial.flush();
}
