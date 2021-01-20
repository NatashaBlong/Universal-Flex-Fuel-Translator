//Flex Fuel sensor code
//Input frequency on pin 8 - pulled up to 5V through 1.8K resistor
//V_out pin 3 - 0-5V (using 0.5-4.5V range) output through 1.8K resistor with 100uf capacitor to ground
//PLX iMFD serial packet on Tx pin 0 through 3.3V TTL voltage divider (3.3K/6.8K) ***Not in Use***

#include <Arduino.h>
#include <FreqMeasure.h>

double sum = 0;
int count = 0;
int ethanol_int, V_out_int;
float freq, ethanol, V_out, E_scalar;
float E0 = 50; //calibration data for pure gasoline
float E85 = 138; //calibration data for E85
//long P0, P1, Pdelta; // This is for Bluetooth
byte PLXpacket[7] {0x80, 0x00, 0x11, 0x00, 0x00, 0x00, 0x40};

//PLX iMFD packet, sent every 100ms:
//0x80      Start byte
//0x00 0x11 Sensor device fuel level
//0x00      Sensor instance
//0x00 0x00 Sensor value (integer, unscaled)
//0x40      Stop byte

void setup() {
  pinMode (3, OUTPUT);
  FreqMeasure.begin(); //Begin frequency measurement
  Serial.begin(19200);
  //P0 = millis(); // For Bluetooth
  E_scalar = (E85 - E0) / 85;
}

void loop() {
  if (FreqMeasure.available()) {
    //Returns the number of measurements available to read, or 0 (false) if none are unread. Average several readings together
    sum = sum + FreqMeasure.read(); 
    //An unsigned long (32 bits) containing the number of CPU clock cycles that elapsed during one cycle of the waveform. 
    //Each measurement begins immediately after the prior one without any delay, so several measurements may be averaged together for better resolution.
    count = count + 1;
    if (count > 30) {
      freq = FreqMeasure.countToFrequency(sum / count); //Convert the 32 bit unsigned long numbers to actual frequency.
      sum = 0;
      count = 0;
    }
  }

  ethanol = (freq - E0) / E_scalar; //scale frequency to E% interpolating E0 and E85 values
  if (ethanol > 100) {
    ethanol = 100;
  }
  if (ethanol < 0) {
    ethanol = 0;
  }
  ethanol_int = (int)ethanol;

  V_out = 0.5 + (0.04 * ethanol); // 0.5V-4.5V = 4V, 0.04 V increase for every 1% ethanol increase (0.04/4V == 1/100E)
  V_out = 51 * V_out; //scale to 255
  V_out_int = (int)V_out; //convert to integer for analogWrite

  analogWrite(3, V_out_int); //output V_out as PWM voltage on pin 3

  /* Bluetooth (not yet in use)
  P1 = millis(); //send PLX packet on Tx pin every 100ms
  Pdelta = P1 - P0;
  if (Pdelta >= 100){
    P0 = P1;
    PLXpacket[5] = ethanol_int; //set data byte in PLX packet to E%
    Serial.write(PLXpacket, 7);
  }
  */
}
