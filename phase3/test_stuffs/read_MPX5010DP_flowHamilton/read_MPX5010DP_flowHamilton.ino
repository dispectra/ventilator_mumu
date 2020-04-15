/*
 * Read analog input read from MPX5010DP for Flow Meter Hamilton
 * [MPX5010DP FM Hamilton | Vout pin] --- [A1 pin | Arduino UNO]
 * ---
 * DimasAP (github.com/dispectra), April 12th 2020
 */

const int PIN_MPX5010DPFMH = A1;
int MPX5010DPFMH_rawValue;
float MPX5010DPFMH_flowValue;

float calcDatasheetPressure(int x_adc) {
  /* Function to return differential pressure value from raw ADC value by
   * following typical characteristic in the datasheet
   * ( http://contrails.free.fr/temp/MPX5010.pdf )
   */
  const int abr = 1023; // ADC bit range
  const int avr = 5;    // ADC voltage range (in volt)
  const float dvo = 0.2;  // Datasheet voltage offset (in volt)
  const float dpg = 2222.22;  // Datasheet pressure gradient (in pascal/volt)
  const float pa_cmh2o = 0.0102; // 1 Pa = 0.0102 cmH2O
  float pds = (((float(x_adc) / abr * avr) - dvo) * dpg); // Pressure based on datasheet (in pascal)
  float pds_cmh2o = pds * pa_cmh2o; // Pressure (in cmH2O)
  return pds_cmh2o;
}

float calcCalibratedFlow(int x_adc) {
  /* Function to return calibrated flow value from raw ADC value by following 
   * two variable linear equation: (y-y_1)/(y_2-y_1) = (x-x_1)/(x_2-x_1)
   *
   * Let y as Flow, x as raw ADC value, 
   * point 1 (y_1, x_1) is  measurement #1, and point 2 (y_2, x_2) is measurement#2
   * Therefore, y = ( (y_2-y_1)*(x-x_1)/(x_2-x_1) ) + y_1
   */
  const float flow_1 = 111; const int adcRaw_1 = 11;  // Measurement #1
  const float flow_2 = 222; const int adcRaw_2 = 22;  // Measurement #2

// Raw value squarerooted, represents the squareroot of pressure difference in flow equation
  adcRaw_1 = sqrt(adcRaw_1); adcRaw_2 = sqrt(adcRaw_2);
  float fcal = ( (flow_2 - flow_1) * float(x_adc - adcRaw_1) / (adcRaw_2 - adcRaw_1) ) + flow_1;
  return fcal;
}


void setup() {
  Serial.begin(115200);
}

void loop() {
  MPX5010DPFMH_rawValue = analogRead(PIN_MPX5010DPFMH); //Arduino ADC 10-bit (0-5V)
  MPX5010DPFMH_flowValue = calcDatasheetPressure(MPX5010DPFMH_rawValue);
  // MPX5010DPFMH_flowCalibratedValue = calcCalibratedFlow(MPX5010DPFMH_rawValue);
  
//  Serial.print(MPX5010DPFMH_rawValue);
  Serial.print('\t'); Serial.print(MPX5010DPFMH_flowValue);
  Serial.println();
  delay(10);
}
