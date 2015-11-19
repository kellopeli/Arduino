/******************************************************************************
  Toimii trinamic TRa_v33 levyn kanssa.
  
  Example for converting from Step/Direction signals to SPI (for a TMC236, TMC239,
  TMC246 or TMC249) using an AVR microcontroller and the GNU C compiler for the
  AVR (WinAVR distribution).
  
  This example use an ATmega8 (it would also work without any changes on ATmega88,
  ATmega168 or ATmega48). Adaption to other AVR MCUs is also easily possible.
  
  The following connections between ATmega8 and TMC236/TMC239/TMC246/TMC249
  are assumed:
 
  ATmega8    arduino pin          TMC236/239/246/249
  MOSI (PB3)    11                SDI
  MISO (PB4)    12                SDO
  SCK  (PB5)    13                SCK
  PB2           10                CSN

  Pin PD3 (INT1) is used as the STEP input, and PD6 is used as the direction input.

   Copyright (C) 2008 TRINAMIC Motion Control GmbH & Co KG
                      SternstraÔøΩe 67
                      D - 20357 Hamburg, Germany
                      http://www.trinamic.com/
  
   This program is free software; you can redistribute it and/or modify it 
   freely.
   
   This program is distributed in the hope that it will be useful, but 
   WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
   or FITNESS FOR A PARTICULAR PURPOSE.
   
******************************************************************************/


//Include files needed for this programme
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

//Useful macros for accessing single bytes of int and long variables
#define BYTE3(var) *((unsigned char *) &var+3)
#define BYTE2(var) *((unsigned char *) &var+2)
#define BYTE1(var) *((unsigned char *) &var+1)
#define BYTE0(var) *((unsigned char *) &var)


//Sine table for the TMC236/TMC239/TMC246/TMC249 with mixed decay on falling slope. 
//Left turn table and right turn table different in order to keep phase direction bit 
//stable at 0 current.
//The tables are kept in FlashROM only.
unsigned char PROGMEM SineWaveL[64]=
{
0x20, 0x22, 0x26, 0x28, 0x2c, 0x2e, 0x30, 0x34, 0x36, 0x38, 0x38, 0x3a, 0x3c, 0x3c, 0x1e, 0x1e,
0x1e, 0x1e, 0x1e, 0x1c, 0x1c, 0x1a, 0x18, 0x18, 0x16, 0x14, 0x10, 0x0e, 0x0c, 0x08, 0x06, 0x02,
0x21, 0x23, 0x27, 0x29, 0x2d, 0x2f, 0x31, 0x35, 0x37, 0x39, 0x39, 0x3b, 0x3d, 0x3d, 0x1f, 0x1f,
0x1f, 0x1f, 0x1f, 0x1d, 0x1d, 0x1b, 0x19, 0x19, 0x17, 0x15, 0x11, 0x0f, 0x0d, 0x09, 0x07, 0x03
};

unsigned char PROGMEM SineWaveR[64]=
{
0x21, 0x02, 0x06, 0x08, 0x0c, 0x0e, 0x10, 0x14, 0x16, 0x18, 0x18, 0x1a, 0x1c, 0x1c, 0x1e, 0x1e,
0x1e, 0x1e, 0x1e, 0x3c, 0x3c, 0x3a, 0x38, 0x38, 0x36, 0x34, 0x30, 0x2e, 0x2c, 0x28, 0x26, 0x22,
0x20, 0x03, 0x07, 0x09, 0x0d, 0x0f, 0x11, 0x15, 0x17, 0x19, 0x19, 0x1b, 0x1d, 0x1d, 0x1f, 0x1f,
0x1f, 0x1f, 0x1f, 0x3d, 0x3d, 0x3b, 0x39, 0x39, 0x37, 0x35, 0x31, 0x2f, 0x2d, 0x29, 0x27, 0x23
};


volatile unsigned int StepCount;  //Counter for the table position
volatile unsigned char Direction;  //Stores the actual direction (0: CCW, !=0: CW)
unsigned int ErrorBits;            //Stores the error bits returned by the motor driver IC
unsigned int GlobalError=0;
unsigned int steps_total = 0;
unsigned long microsteps_total = 0;
unsigned char motorport[3] = {0x04, 0x02, 0x01};
unsigned int motor;
unsigned int maxload;

void setup(void)
{
  //Prepare the I/O ports
  //Here we only set the ports lines that are to be used by this application,
  //one has to set the others according to the individual situation.
  //If other I/O lines are to be used one has to change this here.
  DDRB=0x2f;
  PORTB=0xff;  //It is only important that the CSN input of the motor driver is high

  DDRD=0x00;  //All inputs
  PORTD=0xff;  //with pull-up resistors enabled.

  //Prepare the SPI.
  SPCR=0x5c;  //Bit rate = clk/4. This is optimal when using 16MHz. 
              //For clock frequencies of 8MHz or slower, use clk/2 (SPCR=0x
  SPSR=0x00;

  //Activate interrupt INT1 for the step pulse as rising edges.
  //This could also easily be changed to falling edges. Also, other
  //interrupt lines (e.g. INT0) could be used instead of INT1.
//  MCUCR=0x0c;   //0x0c: rising edge on INT1;  0x08: falling edge on INT1;
  //GICR=0x80;    //activate INT1 line

  //Global interrupt activation
//  sei();  

  // initialize the serial communication:
  Serial.begin(9600);

  Direction = 0;
  StepCount = 0;
  motor = 0;

  Serial.println("Hello World!");
  delay(2000);
}

void loop(void)
{
  
  Serial.print("Motor: ");
  Serial.print(motor, DEC);
  // Nopeudella of 60us/microstep missasi silloin tÔøΩllÔøΩin
  stepMulti(200);
  StepCount=StepCount+400;

  if (StepCount >= 400) {
    StepCount = 0;
    Serial.print("\tSteps: ");
    Serial.print(steps_total, DEC);
    Serial.print("\tMaxload: ");
    Serial.print(maxload, DEC);
    maxload = 0;
    if (GlobalError) {
      Serial.print("\tError: ");
      Serial.println(GlobalError, HEX);
      GlobalError = 0;
    }
    //Serial.println("");
    changeDirection();
    send_sleep();
    delay(1000);   
  }

  stepMulti(50);
  delay(1000);   
  changeDirection();
  stepMulti(50);
  delay(1000);   
  stepMulti(50);
  delay(200);   
  changeDirection();
  stepMulti(50);
  delay(200);   


  //motor++;
  if (motor > 2) {
    motor = 0;
  } 

}

void make_full_step(unsigned int microdelay)
{
  volatile unsigned char MicroStepCount = 0;        //To compare if the position has changed
  int i = 0;

  if (Direction) {
    MicroStepCount = 0;
  } else {
    MicroStepCount = 64;     
  }

  for (i=0; i<64; i++) {
    if(Direction)
    {
      if(MicroStepCount<63)
        MicroStepCount++;
      else
        MicroStepCount=0;
    }
    else
    {
      if(MicroStepCount>0)
        MicroStepCount--;
      else
        MicroStepCount=63;
    }
    microsteps_total++;
    make_micro_step(MicroStepCount);
    delayMicroseconds(microdelay);
  }

  steps_total++;
}

void make_micro_step(unsigned char MicroStep)
{
  unsigned int SPIData;
  unsigned int load;

  //Use appropriate table according to the direction 			
  if(Direction) {
    SPIData=(pgm_read_byte(&SineWaveR[MicroStep & 63])<<6) | pgm_read_byte(&SineWaveR[(MicroStep+16) & 63]);
  }
  else 
    SPIData=(pgm_read_byte(&SineWaveL[MicroStep & 63])<<6) | pgm_read_byte(&SineWaveL[(MicroStep+16) & 63]);

//    Serial.print("   SPI: ");
//    Serial.print(SPIData, HEX);		
  //Mixed decay bits (remove this if mixed decay not desired)
  // pitÔøΩis vaihtaa osc konkka pienemmÔøΩksi muuten tÔøΩmÔøΩ aihettaa kuultavaa ÔøΩÔøΩntÔøΩ.
  SPIData|=0x820;

  //Set CSN signal low
  PORTB&=~motorport[motor];
      
  //Send out the new data
  SPDR=BYTE1(SPIData);
  while(!(SPSR & (1<<SPIF)));
  BYTE1(ErrorBits)=SPDR;

  SPDR=BYTE0(SPIData);
  while(!(SPSR & (1<<SPIF)));
  BYTE0(ErrorBits)=SPDR;
      
  //Set the CSN signal high again
  PORTB|=motorport[motor];
      
  //ErrorBits now contains the returned bits from the TMC236/239/246/249 for further evaluation.
  //These are the error bits and with the TMC246/TMC249 also the StallGuard bits.
  if (ErrorBits & 0x7fff)
    GlobalError = GlobalError|ErrorBits;

  load = (ErrorBits & 0xe000) >> 13;
  if (load > maxload)
    maxload = load;
}

void send_sleep(void)
{
  unsigned int SPIData;

  SPIData =0x0000;

  //Set CSN signal low
  PORTB&=~motorport[motor];
      
  //Send out the new data
  SPDR=BYTE1(SPIData);
  while(!(SPSR & (1<<SPIF)));
  BYTE1(ErrorBits)=SPDR;

  SPDR=BYTE0(SPIData);
  while(!(SPSR & (1<<SPIF)));
  BYTE0(ErrorBits)=SPDR;
      
  //Set the CSN signal high again
  PORTB|=motorport[motor];

}

void changeDirection() {
  if (Direction == 1) {
    Direction = 0;
  } else {
    Direction = 1;
  }
}

void stepMulti(unsigned int steps)
{
  // 64 microsteps = 4 steps = 1 full step 
  // 200 steps = 50 full steps = 1 turn
  // initial microstepdelay 80us
  unsigned int step_delay = 100;//110;
  unsigned int ramp_len = 90;
  unsigned int steady_len = 0;
  int i;
   
   if (steps >= 2*ramp_len) {
     steady_len = steps - 2*ramp_len;
   } else {
     ramp_len = steps/2;
     steady_len = steps%2;
   }
//   Serial.print("steps:");
//   Serial.print(steps);
//   Serial.print("ramp_len:");
//   Serial.print(ramp_len);
//   Serial.print("steady:");
//   Serial.print(steady_len);
//   Serial.print("steps_tot:");
//   Serial.print(steps_total);
//   Serial.println("");
    
   // accelerate
   for (i=0;i < ramp_len;i++) {
     make_full_step(step_delay);
     step_delay--;
   }
   // steady
   for (i=0;i < steady_len;i++) {
     make_full_step(step_delay);
   }
   // decelerate
   for (i=0;i < ramp_len;i++) {
     make_full_step(step_delay);
     step_delay++;
   }
   
}


