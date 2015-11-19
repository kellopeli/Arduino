// esp8266_test.ino
//
// Plot LM35 data on thingspeak.com using an Arduino and an ESP8266 WiFi 
// module.
//
// Author: Mahesh Venkitachalam
// Website: electronut.in

#include <SoftwareSerial.h>
#include <stdlib.h>

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 9
 
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
 
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// LED 
int ledPin = 13;

// replace with your channel's thingspeak API key
String apiKey = "LCU0ZIPQ54FK4B29";

// connect 10 to TX of Serial USB
// connect 11 to RX of serial USB
SoftwareSerial ser(10, 11); // RX, TX

// this runs once
void setup() {                
  // initialize the digital pin as an output.
  pinMode(ledPin, OUTPUT);    

  // enable debug serial
  Serial.begin(9600); 
  // enable software serial
  ser.begin(9600);
  
  // reset ESP8266
  ser.println("AT+RST");
}


// the loop 
void loop() {
  
  // Start up the library
  sensors.begin();

  // blink LED on board
  digitalWrite(ledPin, HIGH);   
  delay(300);               
  digitalWrite(ledPin, LOW);

  sensors.requestTemperatures(); // Send the command to get temperatures

  float temp = sensors.getTempCByIndex(0);

  // convert to string
  char buf[16];
  String strTemp = dtostrf(temp, 4, 1, buf);
  
  Serial.println(strTemp);
  
  // TCP connection
  String cmd = "AT+CIPSTART=\"TCP\",\"";
  cmd += "184.106.153.149"; // api.thingspeak.com
  cmd += "\",80";
  ser.println(cmd);
   
  if(ser.find("Error")){
    Serial.println("AT+CIPSTART error");
    return;
  }
  
  // prepare GET string
  String getStr = "GET /update?api_key=";
  getStr += apiKey;
  getStr +="&field1=";
  getStr += String(strTemp);
  getStr += "\r\n\r\n";

  // send data length
  cmd = "AT+CIPSEND=";
  cmd += String(getStr.length());
  ser.println(cmd);

  if(ser.find(">")){
    ser.print(getStr);
  }
  else{
    ser.println("AT+CIPCLOSE");
    // alert user
    Serial.println("AT+CIPCLOSE");
  }
    
  // thingspeak needs 15 sec delay between updates
  delay(16000);  
}

