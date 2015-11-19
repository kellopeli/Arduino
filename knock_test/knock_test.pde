/*
  AnalogReadSerial
 Reads an analog input on pin 0, prints the result to the serial monitor 
 
 This example code is in the public domain.
 */
int minValue = 1024;
int maxValue = 0;
int ledPin = 13;
int buzzPin = A0;

void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);
}

void loop() {
  int sensorValue = analogRead(A0);
  if (sensorValue < minValue)
    minValue = sensorValue;
  if (sensorValue > maxValue)
    maxValue = sensorValue;
  Serial.print("min: ");
  Serial.print(minValue, DEC);
  Serial.print("max: ");
  Serial.print(maxValue, DEC);
  Serial.print("curr: ");
  Serial.println(sensorValue, DEC);
  
  if (sensorValue > 100) {
    Buzz();
    Buzz();
  }
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
