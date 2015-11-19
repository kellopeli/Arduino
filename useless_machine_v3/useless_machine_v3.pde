
// The most useless machine
// by Topi Rantalainen 


#include <Servo.h> 

#include "pitches.h"

#define PWMpin 5
#define Ledpin 5
#define Servopin 9
#define pi 3.1416

int PWMval;

Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

int pos = 0;    // variable to store the servo position 
// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastChangeTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

boolean wake_enabled = 0;
int count = 0;
int idleTime = 10000;

int lastReading;
int buttonState;             // the current reading from the input pin
int lastState;
int lastButtonState = LOW;   // the previous reading from the input pin
int mytime = 1000;
int lowposition = 133;
int highposition = 45; 
int midposition = 99; 

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4,4,4,4,4 };

int minValue = 1024;
int maxValue = 0;
int ledPin = 13;
int buzzPin = A0;

void setup() {
  myservo.attach(Servopin);      // attaches the servo on pin 9 to the servo object
  pinMode(2, INPUT);
  digitalWrite(2, HIGH);  // turn on pullup resistors
  pinMode(Ledpin, OUTPUT);    // Led
  pinMode(PWMpin, OUTPUT);    // Led
  myservo.write(lowposition);
  
  lastReading = digitalRead(2);
  lastChangeTime = millis();
  
  Serial.begin(9600);
  pinMode(A0, INPUT);
}

void loop() 
{
  int reading = digitalRead(2);
  piezoRead();
  
  if (reading == LOW && lastReading == LOW) {
    if ((millis() - lastChangeTime) > idleTime) {
      //breath();
      if (digitalRead(2) == LOW) {
        //delay(1000 + random(700));
        if (digitalRead(2) == HIGH) {
          louskutus();
        }
      }
      if (wake_enabled) {
        count++;
        if(count > 5) {
          flash_led();
          myservo.write(midposition);
          delay(50);
          myservo.write(lowposition);
          count = 0;
        }
      }
    }
    //delay(60);
  }
  if (reading == HIGH && lastReading == LOW) {
    lastChangeTime = millis();
    myservo.write(highposition);
    digitalWrite(Ledpin, HIGH);
    delay(60);
    //flash_300ms();
    //wake_enabled = !wake_enabled;
    digitalWrite(Ledpin, HIGH);   // set the LED on
  }  
  if (reading == HIGH && lastReading == HIGH) {
    // check for jam 
    if ((millis() - lastChangeTime) > 300) {
      // recover from jam
      myservo.write(midposition);
      delay(170);
      myservo.write(highposition);
      delay(180);
    } 
  }
  if (reading == LOW && lastReading == HIGH) {
    lastChangeTime = millis();
    digitalWrite(Ledpin, LOW);
    myservo.write(lowposition);
  }

  lastReading = reading;
} 

void piezoRead() {
  int sensorValue = analogRead(A0);
  
  if (sensorValue > 100) {
    // Knock detected
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    for (int i = 0; i < 1000; i++) {
      sensorValue = analogRead(A0);
      if (sensorValue > maxValue)
      maxValue = sensorValue;
    }
    if (maxValue > 100) {
      // 2. Knock detected
      maxValue = 0;
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
      for (int i = 0; i < 1000; i++) {
        sensorValue = analogRead(A0);
        if (sensorValue > maxValue)
        maxValue = sensorValue;
      }
      if (maxValue > 100) {
        // 3. Knock detected
        //Buzz();
        myservo.write(midposition);
        titatiitii();
        myservo.write(lowposition);    
        maxValue = 0;
      }
    }
  }  
}

void titatiitii(){
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000/noteDurations[thisNote];
    tone(A0, melody[thisNote],noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(A0);
  }
  pinMode(A0, INPUT);
}

void Buzz() {
  pinMode(A0, OUTPUT);
  for (long i = 0; i < (2048 ); i++ ) {
    // 1 / 2048Hz = 488uS, or 244uS high and 244uS low to create 50% duty cycle
    digitalWrite(buzzPin, HIGH);
    delayMicroseconds(244+i/2);
    digitalWrite(buzzPin, LOW);
    delayMicroseconds(244+i/2);
  }
  pinMode(A0, INPUT);
}

void louskutus()
{
  for (int i = 0; i < 10; i++) {
    myservo.write(midposition);
    delay(50);
    myservo.write(lowposition);
    delay(50);
  }
}

void flash_1s()
{
  for (int i=0; i<10; i++) {
    digitalWrite(Ledpin, HIGH);   // set the LED on
    delay(5);              // wait for a second
    digitalWrite(Ledpin, LOW);    // set the LED off
    delay(80);              // wait for a second
  }  
}

void flash_300ms()
{
  for (int i=0; i<3; i++) {
    digitalWrite(Ledpin, HIGH);   // set the LED on
    delay(20);              // wait for a second
    digitalWrite(Ledpin, LOW);    // set the LED off
    delay(80);              // wait for a second
  }  
}

void flash_led()
{
  digitalWrite(Ledpin, HIGH);   // set the LED on
  delay(2);              // wait for a second
  digitalWrite(Ledpin, LOW);    // set the LED off
  delay(100);              // wait for a second
  digitalWrite(Ledpin, HIGH);   // set the LED on
  delay(2);              // wait for a second
  digitalWrite(Ledpin, LOW);    // set the LED off
  delay(100);              // wait for a second
  digitalWrite(Ledpin, HIGH);   // set the LED on
  delay(2);              // wait for a second
  digitalWrite(Ledpin, LOW);    // set the LED off
}

void breath()
{
  for(int i=0; i <= 360; i++) {
    PWMval = 5 + (1 - cos(pi/180*i))*64;
    analogWrite(PWMpin, PWMval/2);
    delay(9 + random(7));
    if (digitalRead(2) == HIGH) {
      return;
    }
    if (analogRead(A0) > 100) {
      return;
    }
  }
}
