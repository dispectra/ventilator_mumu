/*
 * Read analog input read from AD620 amplifier
 * [MPX2010 | S+ pin] --- [S+ pin | AD620 | Vout pin] --- [A0 pin | Arduino UNO]
 * ---
 * DimasAP (github.com/dispectra), April 2nd 2020
 */

const int A_AD620_MPX2010 = A0;
int AD620_MPX2010_rawValue;

void setup() {
  Serial.begin(115200);
}

void loop() {
  AD620_MPX2010_rawValue = analogRead(A_AD620_MPX2010);
  int output_pos = analogRead(A1);
  int differences = output_pos - AD620_MPX2010_rawValue;
  Serial.print("S+ : "); Serial.print(output_pos); Serial.print("\t S- : "); Serial.print(AD620_MPX2010_rawValue); Serial.print("\t S+ - S- = "); Serial.println(differences);
  delay(100);
}
