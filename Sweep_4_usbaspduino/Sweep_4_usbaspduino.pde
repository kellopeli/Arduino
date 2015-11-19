// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.
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
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
int servopin = 11;
 
void setup() 
{ 
  myservo.attach(servopin);  // attaches the servo on pin 9 to the servo object 
} 
 
 
void loop() 
{ 
  for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
} 
