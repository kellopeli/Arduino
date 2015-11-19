/*
  USPASPduino Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Arduino  USPASPduino
  pin 10   PB2(SS/OC1B) pwm
  pin 11   PB3(MOSI/OC2) pwm
  pin 12   PB4(MISO)
  pin 13   PB5(SCK)
  pin A0   PC0(ADC0) red led
  pin A1   PC1(ADC1) green led
 
  This example code is in the public domain.
 */

void setup() {                
  // initialize the digital pin as an output.
  // Pin 13 has an LED connected on most Arduino boards:
  pinMode(A0, OUTPUT);     
  pinMode(A1, OUTPUT);     
}

void loop() {
  digitalWrite(A0, HIGH);   // set the LED on
  digitalWrite(A1, LOW);   // set the LED on
  delay(500);              // wait for a second
  digitalWrite(A0, LOW);    // set the LED off
  digitalWrite(A1, HIGH);    // set the LED off
  delay(500);              // wait for a second
}
