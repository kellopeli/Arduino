// The most useless machine
// by Topi Rantalainen 


#include <Servo.h> 

#define PWMpin 5
#define pi 3.1416

int PWMval;

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers
long high_start_time;

int count = 0;
int idleTime = 8000;

int buttonState;             // the current reading from the input pin
int lastState;
int lastButtonState = LOW;   // the previous reading from the input pin
int mytime = 1000;
int lowposition = 133;
int highposition = 48; 
int midposition = 95; 
void setup() 
{ 
  myservo.attach(9);      // attaches the servo on pin 9 to the servo object
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);  // turn on pullup resistors
  pinMode(4, OUTPUT);    // Led
  myservo.write(lowposition);
} 
 
 
void loop() 
{
  int reading;
  
  breath();
  
  reading = digitalRead(2);
  
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > idleTime) {
    // wake up 
    digitalWrite(4, HIGH);   // set the LED on
    delay(2);              // wait for a second
    digitalWrite(4, LOW);    // set the LED off
    delay(100);              // wait for a second
    digitalWrite(4, HIGH);   // set the LED on
    delay(2);              // wait for a second
    digitalWrite(4, LOW);    // set the LED off
    delay(100);              // wait for a second
    digitalWrite(4, HIGH);   // set the LED on
    delay(2);              // wait for a second
    digitalWrite(4, LOW);    // set the LED off
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    buttonState = reading;
  }

  
  if(buttonState != lastState) {
    if(buttonState == HIGH) {
      count++;
      if(count > 5) {
        for (int i=0; i<10; i++) {
          digitalWrite(4, HIGH);   // set the LED on
          delay(5);              // wait for a second
          digitalWrite(4, LOW);    // set the LED off
          delay(80);              // wait for a second
        }
        digitalWrite(4, HIGH);   // set the LED on
        count = 0;
      } else {
        digitalWrite(4, HIGH);
        delay(mytime);
      }
      mytime = 0;
      myservo.write(highposition);
    } else {
      digitalWrite(4, LOW);
      myservo.write(lowposition);
      mytime = 1000;
    }
    lastState = buttonState;
  }

  if(buttonState == HIGH) {
    if ((millis() - lastDebounceTime) > 1400) {
      myservo.write(midposition);
      delay(170);
      myservo.write(highposition);
      delay(170);
    } 
  }
  lastButtonState = reading;
} 

void breath()
{
  for(int i=0; i <= 360; i++) {
    PWMval = 5 + (1 - cos(pi/180*i))*64;
    analogWrite(PWMpin, PWMval);
    delay(10 + random(7));
  }
  delay(1000 + random(700));
}
