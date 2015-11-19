// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 
/*
  Arduino  USPASPduino
  pin 10   PB2(SS/OC1B) pwm
  pin 11   PB3(MOSI/OC2) pwm
  pin 12   PB4(MISO)
  pin 13   PB5(SCK)
  pin A0   PC0(ADC0) red led
  pin A1   PC1(ADC1) green led
  pin A2   PC2(ADC2) (pin 25 - NC)
*/

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
 
int potpin = 2;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin 
 
void setup() 
{ 
  myservo.attach(11);  // attaches the servo on pin 9 to the servo object 
} 
 
void loop() 
{ 
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023) 
  val = map(val, 0, 1023, 0, 179);     // scale it to use it with the servo (value between 0 and 180) 
  myservo.write(val);                  // sets the servo position according to the scaled value 
  delay(15);                           // waits for the servo to get there 
} 
