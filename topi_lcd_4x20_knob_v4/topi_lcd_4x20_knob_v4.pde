#include <LCD4x20.h>

#define BASE 10
#define CMD_DISPLAY_CURSOR_ON B00000010
#define CMD_DISPLAY_CURSOR_BLINKING_ON B00000001

#define ROWS 4
#define COLS 4
// An is alias for analog input port when user for digital I/O
//#define A0 14
//#define A1 15
//#define A2 16
//#define A3 17
//#define A4 18
//#define A5 19

#define CONTROLPIN 13
// 4x4 buttons
const byte buttonWrite[4] = { 6, 7, 8, 9};     //Pins for the Vin of the buttons (rows)
const byte buttonRead[4]  = { A0, A1, A2, A3}; //Pins for reading the state of the buttons 
const char buttonChar[4][4] = {{'1','2','3','A'},
                               {'4','5','6','B'},
                               {'7','8','9','C'},
                               {'*','0','#','D'}};
const int buttonValue[4][4] = {{ 1, 2, 3, 200 },
                               { 4, 5, 6, 200 },
                               { 7, 8, 9, 200 },
                               { 200, 0, 200, 10 }};
boolean pressed[ROWS][COLS] = { 0 };




//LCD4x20 (int rsPin, int enablePin, int d4, int d5, int d6, int d7);
LCD4x20 lcd = LCD4x20(12, 10, 5, 4, 3, 2);

int ontime_ms;   // p��ll�oloaika millisekunteina

// for itoa int conversion. since int is signed 16-bit, max number 32767  we need 5 chars, plus one sign and one string terminator, so 7
// since we are reading 10-bit ADC (max 1023), we should only every need 5 total.
char buffer[5];

//char teststr[] = { "ABCDEFGHIJKLMOPQRSTUVWXYZabcdefghijklmopqrstuvwxyz01234567890ABCDEFGHIJKLMOPQRST" };
char teststr[] = { "Hello World!\nTime: " };
char stringBuffer[] = { "          " };

char twolines[] = { "this is more then twenty characters "};

int hour, hours = 0;
int minute, minutes = 0;
int second, seconds = 0;
double start_time, mips;

void setup() {
  pinMode(13, OUTPUT); //we'll use the debug LED to output a heartbeat

  pinMode(11, OUTPUT);     
  digitalWrite(11, LOW);    // set the rw bit off

  Serial.begin(57600);

  lcd.init();
  lcd.setMode(MODE_SHIFT_UP);
  lcd.print(teststr);
  
  //setup the button inputs and outputs
  for(int i = 0; i < ROWS; ++i)
  {
    pinMode(buttonWrite[i], OUTPUT);
    digitalWrite(buttonWrite[i],LOW);
    pinMode(buttonRead[i], INPUT);
    digitalWrite(buttonRead[i], HIGH);    // internal pullup resistor
  }

  ontime_ms = 0;
  lcd.setCursor(3, 0);
  lcd.print("Luettu: ");
  print_int(ontime_ms, 4);
  lcd.print("ms");
  
  pinMode(CONTROLPIN, OUTPUT);
  digitalWrite(CONTROLPIN, LOW);
}

void loop() {

  start_time = millis();

  read_button();
  
  print_time();

  mips = millis() - start_time;
  //delay(1000-mips);
}

void print_time()
{
  seconds = start_time/1000;
  second = seconds % 60;
  minutes = seconds / 60;
  minute = minutes % 60; 
  hours = minutes / 60;
  hour = hours % 60;

  lcd.setCursor(2, 6);
  print_int(hour, 2);
  lcd.print(':');
  print_int(minute, 2);
  lcd.print(':');
  print_int(second, 2);
}

void print_int(int value, int buffer_size) 
{
  char buffer[8] = {};
  int i;
  double ascii_digit;
  
  for (i = 0; i < buffer_size; i++) 
  {
    buffer[i] = '0';    
  }
  i = 0;
  do {
    i++;
    ascii_digit = (value % 10) + 48;
    buffer[buffer_size - i] = ascii_digit;
    value /= 10;
  } while (value != 0);

  lcd.print(buffer);

}

void read_button()
{
  for(byte r = 0; r < 4; ++r)
  {
    digitalWrite(buttonWrite[r], LOW);
    
    for(byte c = 0; c < COLS; ++c)
    {
      if(pressed[r][c] != digitalRead(buttonRead[c]))
      {
        pressed[r][c] = digitalRead(buttonRead[c]);
        if(!pressed[r][c]) 
        {
          if (buttonValue[r][c] <  10)
          {
            if (ontime_ms > 999)
              ontime_ms = ontime_ms - (ontime_ms / 1000) * 1000;
            ontime_ms = ontime_ms * 10 + buttonValue[r][c];
            lcd.setCursor(3, 8);
            print_int(ontime_ms, 4);
            lcd.print("ms");
          } else if (buttonValue[r][c] == 10 && ontime_ms > 0)
          {
            lcd.setCursor(4, 0);
            lcd.print("Boom: ");
            for (int i=3; i > 0; i--)
            {
              lcd.setCursor(4, 6);
              print_int(i, 1);
              delay(1000);
            }
            lcd.setCursor(4, 6);
            lcd.print("Bang");
            digitalWrite(CONTROLPIN, HIGH);
            delay(ontime_ms);
            digitalWrite(CONTROLPIN, LOW);
            delay(1000);
            lcd.setCursor(4, 0);
            lcd.print("           ");
          }
        }
        else {
        }
      }
      else {
        if(!pressed[r][c]){
        }
        else {
        }
      }
    }
    digitalWrite(buttonWrite[r], HIGH);
  }
}

