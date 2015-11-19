/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
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
  delay(1000);              // wait for a second
  digitalWrite(A0, LOW);    // set the LED off
  digitalWrite(A1, HIGH);    // set the LED off
  delay(1000);              // wait for a second
}
