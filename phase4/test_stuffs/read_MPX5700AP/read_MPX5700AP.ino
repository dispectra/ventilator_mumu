/*
 * Read analog input read from MPX5700AP
 * [MPX5700AP | Vout pin] --- [A0 pin | Arduino UNO]
 * ---
 * DimasAP (github.com/dispectra), April 27th 2020
 */

const int PIN_MPX5700AP = A0;
int MPX5700AP_rawValue;
float MPX5700AP_pressureValue;

float calcDatasheetPressure(int x_adc) {
  /* Function to return differential pressure value from raw ADC value by
   * following typical characteristic in the datasheet
   * ( https://www.nxp.com/docs/en/data-sheet/MPX5700.pdf )
   */
   
//  const int abr = 1023; // ADC bit range
//  const int avr = 5;    // ADC voltage range (in volt)
//  const float dvo = 0.2;  // Datasheet voltage offset (in volt)
//  const float dpg = 2222.22;  // Datasheet pressure gradient (in pascal/volt)
//  const float pa_cmh2o = 0.0102; // 1 Pa = 0.0102 cmH2O
//  float pds = (((float(x_adc) / abr * avr) - dvo) * dpg); // Pressure based on datasheet (in pascal)

//  float pds = mapFloat(x_adc, 0.0, 1023.0, 0.0, 5.0);
  float pds = (5) * float(x_adc) / (1023);
//  pds = mapFloat(pds, 0.2, 4.7, 0.0, 700000.0);
  pds = (700000) * (pds - 0.2) / (4.7 - 0.2);

  const float pa_cmh2o = 0.0102; // 1 Pa = 0.0102 cmH2O
  float pds_cmh2o = pds * pa_cmh2o; // Pressure (in cmH2O)
  return pds_cmh2o;
}

float calcCalibratedPressure(int x_adc) {
  /* Function to return calibrated differential preessure value from raw ADC value by following 
   * two variable linear equation: (y-y_1)/(y_2-y_1) = (x-x_1)/(x_2-x_1)
   *
   * Let y as Pressure, x as raw ADC value, 
   * point 1 (y_1, x_1) is  measurement #1, and point 2 (y_2, x_2) is measurement#2
   * Therefore, y = ( (y_2-y_1)*(x-x_1)/(x_2-x_1) ) + y_1
   */
  const float pressure_1 = 111; const int adcRaw_1 = 11;  // Measurement #1
  const float pressure_2 = 222; const int adcRaw_2 = 22;  // Measurement #2
  
  float pcal = ( (pressure_2 - pressure_1) * float(x_adc - adcRaw_1) / (adcRaw_2 - adcRaw_1) ) + pressure_1;
  return pcal;
}


void setup() {
  Serial.begin(115200);
  Serial.println("adc,cmH2O");
}

void loop() {
  MPX5700AP_rawValue = analogRead(PIN_MPX5700AP); //Arduino ADC 10-bit (0-5V)
  MPX5700AP_pressureValue = calcDatasheetPressure(MPX5700AP_rawValue);
  // MPX5700AP_pressureCalibratedValue = calcCalibratedPressure(MPX5700AP_rawValue);
  
  Serial.print(MPX5700AP_rawValue);
  Serial.print('\t'); Serial.print(MPX5700AP_pressureValue);
  Serial.println();
  delay(10);
}

//-- Float type mapper, for linear regression calibration --
float mapFloat(int rawX, int rawA, int rawB, float realA, float realB) {
  float realX = ( (realB - realA) * float((rawX - rawA) / (rawB - rawA)) ) + realA;
  return realX;
}
// function overloading of mapFloat()
float mapFloat(int rawX, float rawA, float rawB, float realA, float realB) {
  float realX = ( (realB - realA) * float((rawX - rawA) / (rawB - rawA)) ) + realA;
  return realX;
}
// function overloading of mapFloat()
float mapFloat(float rawX, float rawA, float rawB, float realA, float realB) {
  float realX = ( (realB - realA) * ((rawX - rawA) / (rawB - rawA)) ) + realA;
  return realX;
}
