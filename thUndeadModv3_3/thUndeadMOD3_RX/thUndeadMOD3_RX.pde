// ******************** thUndeadMOD3      ******************* 
// Mihai Andrian - thUndead http://www.fpvuk.org/forum/index.php?topic=3642.0
// 
// A lot of countless nights went into this so 
//please don't screw it up and MENTION my name if using any of my own code! Cheers
//******************* ******************* ******************* 
// First proper firmware for the openLRS system
// Designed with Long Range in mind
// Please use the configurator 
// Reliable RSSI output  
// etc etc :)
// like it ? why not contribuite to the Pringles Fund ? http://tinyurl.com/425x4hj Cheers.
// questions here: http://www.fpvuk.org/forum/index.php?topic=3642.0
// **********************************************************
// ******************   OpenLRS Tx Code   *******************
// ***  OpenLRS Designed by Melih Karakelle on 2010-2011  ***
// **  an Arudino based RC Rx/Tx system with extra futures **
// **       This Source code licensed under GPL            **
// **********************************************************

// ******************** OpenLRS DEVELOPERS ****************** 
// Mihai Andrian - thUndead http://www.fpvuk.org/forum/index.php?topic=3642.0
// Melih Karakelle (http://www.flytron.com) (forum nick name: Flytron)
// Jan-Dirk Schuitemaker (http://www.schuitemaker.org/) (forum nick name: CrashingDutchman)
// Etienne Saint-Paul (http://www.gameseed.fr) (forum nick name: Etienne) 
#include "config.h"
#include <EEPROM.h>

unsigned char tmp[2];
boolean fsdetect=0;
unsigned char Servo_Buffer[10];  //= {3000,3000,3000,3000,3000,3000,3000,3000};	//servo position values from RF
unsigned char Servo_Position[10]= {3000,3000,3000,3000,3000,3000,3000,3000};	//real servo position values
static unsigned char Servo_Number = 0;
unsigned int total_ppm_time = 0;
unsigned short Rx_RSSI,resync = 0;


static unsigned char receiver_mode = 0;


unsigned long time;
unsigned long last_pack_time ;

volatile unsigned char RF_Mode = 0;
#define Available 0
#define Transmit 1
#define Transmitted 2
#define Receive 3
#define Received 4



#if (RX_BOARD_TYPE==1)
    //## RFM22B Pinouts for Rx v1 Board
    #define SDO_pin A0
    #define SDI_pin A1        
    #define SCLK_pin 2 
    #define IRQ_pin 3
    #define nSel_pin 4
    #define IRQ_interrupt 1
        
    #define  nIRQ_1 (PIND & 0x08)==0x08 //D3
    #define  nIRQ_0 (PIND & 0x08)==0x00 //D3
    
    #define  nSEL_on PORTD |= (1<<4) //D4
    #define  nSEL_off PORTD &= 0xEF //D4
    
    #define  SCK_on PORTD |= (1<<2) //D2
    #define  SCK_off PORTD &= 0xFB //D2
    
    #define  SDI_on PORTC |= (1<<1) //C1
    #define  SDI_off PORTC &= 0xFD //C1
    
    #define  SDO_1 (PINC & 0x01) == 0x01 //C0
    #define  SDO_0 (PINC & 0x01) == 0x00 //C0
    
    //#### Other interface pinouts ###
    #define GREEN_LED_pin A2
    #define RED_LED_pin A3
    
    #define Red_LED_ON  PORTC &= ~_BV(2);PORTC |= _BV(3);
    #define Red_LED_OFF  PORTC &= ~_BV(2);PORTC &= ~_BV(3);
    
    #define Green_LED_ON  PORTC &= ~_BV(3);PORTC |= _BV(2);
    #define Green_LED_OFF  PORTC &= ~_BV(3);PORTC &= ~_BV(2);    
        
    #define Servo_Ports_LOW PORTB &= 0x00; PORTD &= 0x1F; // pulling down the servo outputs
    
    #define RSSI_MODE 1 //0=disable  1=enable 
    #define RSSI_OUT 6 //Servo9 or RSSI
    
    #define Servo1_OUT 5 //Servo1
    #define Servo2_OUT 7 //Servo2
    #define Servo3_OUT 8 //Servo3
    #define Servo4_OUT 9 //Servo4
    #define Servo5_OUT 10 //Servo5
    #define Servo6_OUT 11 //Servo6
    #define Servo7_OUT 12 //Servo7
    #define Servo8_OUT 13 //Servo8
    #define Servo9_OUT 13 //Servo9 // not have on this version
    
     
    #define Servo1_OUT_HIGH PORTD |= _BV(5) //Servo1
    #define Servo2_OUT_HIGH PORTD |= _BV(7) //Servo2
    #define Servo3_OUT_HIGH PORTB |= _BV(0) //Servo3
    #define Servo4_OUT_HIGH PORTB |= _BV(1) //Servo4
    #define Servo5_OUT_HIGH PORTB |= _BV(2) //Servo5
    #define Servo6_OUT_HIGH PORTB |= _BV(3) //Servo6
    #define Servo7_OUT_HIGH PORTB |= _BV(4) //Servo7
    #define Servo8_OUT_HIGH PORTB |= _BV(5) //Servo8
    #define Servo9_OUT_HIGH PORTB = PORTB  //Servo9 // not have on this version
    
    #define Serial_PPM_OUT_HIGH PORTB = _BV(4) //Serial PPM out on Servo 8
#endif


#if (RX_BOARD_TYPE==2)
      //### PINOUTS OF OpenLRS Rx V2 Board
      #define SDO_pin A0
      #define SDI_pin A1        
      #define SCLK_pin A2 
      #define IRQ_pin 2
      #define nSel_pin 4
      #define IRQ_interrupt 0
      
      #define  nIRQ_1 (PIND & 0x04)==0x04 //D2
      #define  nIRQ_0 (PIND & 0x04)==0x00 //D2
      
      #define  nSEL_on PORTD |= 0x10 //D4
      #define  nSEL_off PORTD &= 0xEF //D4
      
      #define  SCK_on PORTC |= 0x04 //C2
      #define  SCK_off PORTC &= 0xFB //C2
      
      #define  SDI_on PORTC |= 0x02 //C1
      #define  SDI_off PORTC &= 0xFD //C1
      
      #define  SDO_1 (PINC & 0x01) == 0x01 //C0
      #define  SDO_0 (PINC & 0x01) == 0x00 //C0
      
      //#### Other interface pinouts ###
      #define GREEN_LED_pin 13
      #define RED_LED_pin A3
    
      #define Red_LED_ON  PORTC |= _BV(3);
      #define Red_LED_OFF  PORTC &= ~_BV(3);
      
      #define Green_LED_ON  PORTB |= _BV(5);
      #define Green_LED_OFF  PORTB &= ~_BV(5);
      
      #define Servo_Ports_LOW PORTB &= 0xE0; PORTD &= 0x17; // pulling down the servo outputs
   
      #define Servo1_OUT 3 //Servo1
      #define Servo2_OUT 5 //Servo2
      #define Servo3_OUT 6 //Servo3
      #define Servo4_OUT 7 //Servo4
      #define Servo5_OUT 8 //Servo5
      #define Servo6_OUT 9 //Servo6
      #define Servo7_OUT 12 //Servo7
      #define Servo8_OUT 11 //Servo8
      //#define Servo9_OUT 12 //Servo9
       
      #define RSSI_MODE 1 //0=disable  1=enable 
      #define RSSI_OUT 10 //Servo7 or RSSI
      
      #define Servo1_OUT_HIGH PORTD |= _BV(3) //Servo1
      #define Servo2_OUT_HIGH PORTD |= _BV(5) //Servo2
      #define Servo3_OUT_HIGH PORTD |= _BV(6) //Servo3
      #define Servo4_OUT_HIGH PORTD |= _BV(7) //Servo4
      #define Servo5_OUT_HIGH PORTB |= _BV(0) //Servo5
      #define Servo6_OUT_HIGH PORTB |= _BV(1) //Servo6
      #define Servo7_OUT_HIGH PORTB |= _BV(4) //Servo7
      #define Servo8_OUT_HIGH PORTB |= _BV(3) //Servo8
     // #define Servo9_OUT_HIGH PORTB |= _BV(4) //Servo9 
      
      #define Serial_PPM_OUT_HIGH PORTB = _BV(3) //Serial PPM out on Servo 8
#endif

unsigned char CH =0;
unsigned char counter;
short FShop,firstpack =0;
short lostpack =0;

boolean willhop =0;



void setup() {   
  //LEDs


  pinMode(GREEN_LED_pin, OUTPUT);  
  pinMode(RED_LED_pin, OUTPUT);

  //RF module pins
  pinMode(SDO_pin, INPUT); //SDO
  pinMode(SDI_pin, OUTPUT); //SDI        
  pinMode(SCLK_pin, OUTPUT); //SCLK
  pinMode(IRQ_pin, INPUT); //IRQ
  pinMode(nSel_pin, OUTPUT); //nSEL


  pinMode(0, INPUT); // Serial Rx
  pinMode(1, OUTPUT);// Serial Tx


  pinMode(RSSI_OUT, OUTPUT); //RSSI pinout

  pinMode(Servo1_OUT, OUTPUT); //Servo1
  pinMode(Servo2_OUT, OUTPUT); //Servo2
  pinMode(Servo3_OUT, OUTPUT); //Servo3
  pinMode(Servo4_OUT, OUTPUT); //Servo4
  pinMode(Servo5_OUT, OUTPUT); //Servo5
  pinMode(Servo6_OUT, OUTPUT); //Servo6
  pinMode(Servo7_OUT, OUTPUT); //Servo7
  pinMode(Servo8_OUT, OUTPUT); //Servo8


  Serial.begin(SERIAL_BAUD_RATE); //Serial Transmission 

  INIT_SERVO_DRIVER();

  attachInterrupt(IRQ_interrupt,RFM22B_Int,FALLING);

}


//############ SERVO INTERRUPT ##############
// We configured the ICR1 value for 40.000 into the "init_servo_driver" function. 
// It's mean this interrupt works when the Timer1 value equal 40.000
// we are configuring it for 40.000 - servo_signal_time for each servo channel, and The interrupt generating perfect servo timings fous.
// Timer1 configured for 1/8 CPU clock. with this configuration, each clock time is equal 0.5us and we are driving the servo with 204step resolution.
ISR(TIMER1_OVF_vect)
{
  unsigned int us; // this value is not real microseconds, we are using 0.5us resolution (2048 step), this is why the all values 2 times more than realmicroseconds.

   
  Servo_Ports_LOW;

  Servo_Number++; // jump to next servo
  if (Servo_Number>8) // back to the first servo 
  {
    total_ppm_time = 0; // clear the total servo ppm time
    Servo_Number=0;
  }


  if (Servo_Number == 8)  // Check the servo number. 
  {
    //Servos accepting 50hz ppm signal, this is why we are waiting for 20ms before second signal brust. 
    us = 40000 - total_ppm_time; //wait for total 20ms loop.  waiting time = 20.000us - total servo times
  }
  else
    us = map(Servo_Buffer[Servo_Number],0,255,1501,4499);
    //us = Servo_Position[Servo_Number]; // read the servo timing from buffer

  total_ppm_time += us; // calculate total servo signal times.

  if (receiver_mode==0) // Parallel PPM
  {  
    switch (Servo_Number) {
    case 0:
      Servo1_OUT_HIGH;
      break;
    case 1:
      Servo2_OUT_HIGH;
      break;
    case 2:
      Servo3_OUT_HIGH;
      break;
    case 3:
      Servo4_OUT_HIGH;
      break;
    case 4:
      Servo5_OUT_HIGH;
      break;
    case 5:
      Servo6_OUT_HIGH;
      break;
    case 6:
      Servo7_OUT_HIGH;
      break;
    case 7:
      Servo8_OUT_HIGH;
      break;
    //case 8:
     // Servo9_OUT_HIGH;
      break;  
    }     
  }
#if (SERIAL_PPM_TYPE == 0)
  else  // Serial PPM over 8th channel
  {
    Serial_PPM_OUT_HIGH;
  }
#else
  else //Mixed Serial+Parallel PPM
  {
    Servo4_OUT_HIGH; //serial from 4th channel  
    switch (Servo_Number) { //last 4 channel works normally
    case 4:
      Servo5_OUT_HIGH;
      break;
    case 5:
      Servo6_OUT_HIGH;
      break;
    case 6:
      Servo7_OUT_HIGH;
      break;
    case 7:
      Servo8_OUT_HIGH;
      break;
    case 8:
      Servo9_OUT_HIGH;
      break;    
    }   
  }  
#endif  

  TCNT1 = 40000 - us; // configure the timer interrupt for X micro seconds     
}


//############ MAIN LOOP ##############
void loop() {



  receiver_mode = check_modes(); // Check the possible jumper positions for changing the receiver mode.

  load_failsafe_values();   // Load failsafe values on startup

  if (receiver_mode == 1) Red_LED_Blink(3); // 3x Red LED blinks for serial PPM mode.


  Red_LED_ON;

  RF22B_init_parameter(); // Configure the RFM22B's registers

  frequency_configurator(CARRIER_FREQUENCY); // Calibrate the RFM22B to this frequency, frequency hopping starts from here.

  to_rx_mode(); 

  sei();




  //################### RX SYNC AT STARTUP #################
RF_Mode = Receive;
  

 
  time = millis();

  last_pack_time = time; // reset the last pack receiving time for first usage
  firstpack =0;
  
  while(1){    /* MAIN LOOP */

    //Serial.println(seed,DEC);	 				 

    time = millis();




    if (_spi_read(0x0C)==0) {
      RF22B_init_parameter(); 
      to_rx_mode(); 
    }// detect the locked module and reboot			 



    if(RF_Mode == Received)   // RFM22B INT pin Enabled by received Data
    { 
      RF_Mode = Receive;
      last_pack_time = time; // record last package time
      lostpack=0;
      
     if (firstpack ==0)  firstpack =1;
      
        Red_LED_OFF;
        Green_LED_ON;

     

      send_read_address(0x7f); // Send the package read command

      Servo_Buffer[0] =  read_8bit_data(); 
      Servo_Buffer[1] =  read_8bit_data();
      Servo_Buffer[2] =  read_8bit_data(); 
      Servo_Buffer[3] =  read_8bit_data();
      Servo_Buffer[4] =  read_8bit_data();
      Servo_Buffer[5] =  read_8bit_data();
      
      tmp[0] =  read_8bit_data();
      tmp[1] =  read_8bit_data();
      
      if ((tmp[0]==0)&&(tmp[1]==0)&&(fsdetect==0)) fsdetect=1;
        else
         {
           Servo_Buffer[6]=tmp[0];
           Servo_Buffer[7]=tmp[1];
           
           if (fsdetect==1) 
            {
            save_failsafe_values();
           //Serial.println("Saved");
           fsdetect=0;
            }           
         }
      
      Rx_RSSI += _spi_read(0x26); // Read the RSSI value
    
      rx_reset();
      
     counter++;
     if (counter>10)
       {
      Rx_RSSI /=10;
//     Serial.print(Rx_RSSI,DEC); 
//     Serial.print(" "); 
//     analogWrite(RSSI_OUT,map(constrain(Rx_RSSI,45,120),40,120,0,255));
//     Serial.println(map(constrain(Rx_RSSI,45,120),40,120,0,255)); 
     Rx_RSSI =0;
     counter=0;
    
       }
     
      willhop =1;
        resync=0; 

      Green_LED_OFF;

      
    }
   
   
    if ((time-last_pack_time > (40-resync))&&(firstpack==1))//automatic hopping for clear channel when rf link down for 40 ms.	
    {
    Red_LED_ON;
        
        last_pack_time = time;
        lostpack++;
       if (lostpack >1)
       if (resync ==0) resync =1; else resync=0;
       willhop =1;
        
     if (lostpack >10)
            {
         //  resync =2;      
            }   
        if (lostpack >25) 
        
          {
            resync=0;
           firstpack =0;
            load_failsafe_values(); // Load Failsafe positions from EEPROM
            analogWrite(RSSI_OUT,0);   
          }
          
    } 
    
#if (FREQUENCY_HOPPING==1)
if (willhop==1)
  {
   
    Hopping();//Hop to the next frequency
    willhop =0;

  }
#endif  

  }


}


// **********************************************************
// **                   OpenLRS Functions                  **
// **        Developed by Melih Karakelle on 2010-2011     **
// **          This Source code licensed under GPL         **
// **********************************************************
// Latest Code Update : 2011-10-04
// Supported Hardware : OpenLRS Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/


void INIT_SERVO_DRIVER(void)
{
   TCCR1B   =   0x00;   //stop timer
   TCNT1H   =   0x00;   //setup
   TCNT1L   =   0x00;
   ICR1   =   40000;   // used for TOP, makes for 50 hz
   
   TCCR1A   =   0x02;   
   TCCR1B   =   0x1A; //start timer with 1/8 prescaler for 0.5us PPM resolution
   
   TIMSK1 = _BV (TOIE1);   
} 

void RFM22B_Int()
{
 if (RF_Mode == Transmit) 
    {
     RF_Mode = Transmitted; 
    } 
 if (RF_Mode == Receive) 
    {
     RF_Mode = Received; 
    }  
}

void Red_LED_Blink(unsigned short blink_count)
  {
  unsigned char i;
  for (i=0;i<blink_count;i++)
     {
     delay(100);
     Red_LED_ON;
     delay(100);
     Red_LED_OFF;
     }
  }



void load_failsafe_values(){
 
  for (int i=0;i<8;i++)
      Servo_Buffer[i] =  EEPROM.read(11+i);

}


void save_failsafe_values(void){

EEPROM.write(11,Servo_Buffer[0]); 
EEPROM.write(12,Servo_Buffer[1]); 
EEPROM.write(13,Servo_Buffer[2]); 
EEPROM.write(14,Servo_Buffer[3]); 
EEPROM.write(15,Servo_Buffer[4]); 
EEPROM.write(16,Servo_Buffer[5]); 
EEPROM.write(17,Servo_Buffer[6]); 
EEPROM.write(18,Servo_Buffer[7]); 


}

unsigned char check_modes(void){

//-- Serial PPM Selection (jumper between Ch1 and ch3)
pinMode(Servo3_OUT, INPUT); //CH3 input
digitalWrite(Servo3_OUT, HIGH); // pull up
digitalWrite(Servo1_OUT, HIGH); // CH1 is HIGH
delayMicroseconds(1);
if ( digitalRead(Servo3_OUT) == HIGH) 
	{
	digitalWrite(Servo1_OUT, LOW); // CH1 is LOW
	delayMicroseconds(1);
	if (digitalRead(Servo3_OUT) == LOW) // OK jumper plugged
			{
                        pinMode(Servo3_OUT, OUTPUT);
			return  1; //Serial PPM OUT
			}
	}
	

pinMode(Servo3_OUT, OUTPUT);

return  0; // Parallel PPM OUT
}

//############# FREQUENCY HOPPING FUNCTIONS #################
#if (FREQUENCY_HOPPING==1)
void Hopping(void)
    {

CH++;
if (CH>CHNO) CH=0;

 _spi_write(0x79, hop_list[CH]);


    }
#endif


 
// **********************************************************
// **      RFM22B/Si4432 control functions for OpenLRS     **
// **       This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2011-09-26
// Supported Hardware : OpenLRS Tx/Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************
 
#define NOP() __asm__ __volatile__("nop") 

 
#define RF22B_PWRSTATE_READY    01 
#define RF22B_PWRSTATE_TX        0x09 


#define RF22B_PWRSTATE_RX       05 
#define RF22B_Rx_packet_received_interrupt   0x02 

#define RF22B_PACKET_SENT_INTERRUPT  04 
#define RF22B_PWRSTATE_POWERDOWN  00 

 
unsigned char ItStatus1, ItStatus2; 
 
 
typedef struct   
{ 
 unsigned char reach_1s    : 1; 
} FlagType; 
FlagType               Flag; 
 
unsigned char read_8bit_data(void); 

void to_ready_mode(void); 
void send_8bit_data(unsigned char i); 
void send_read_address(unsigned char i); 
void _spi_write(unsigned char address, unsigned char data); 
void RF22B_init_parameter(void); 

void port_init(void);   
unsigned char _spi_read(unsigned char address); 
void Write0( void ); 
void Write1( void ); 
void timer2_init(void); 
void Write8bitcommand(unsigned char command); 

 
 
//***************************************************************************** 
//***************************************************************************** 

//-------------------------------------------------------------- 
void Write0( void ) 
{ 
    SCK_off;  
    NOP(); 
     
    SDI_off; 
    NOP(); 
     
    SCK_on;  
    NOP(); 
} 
//-------------------------------------------------------------- 
void Write1( void ) 
{ 
    SCK_off;
    NOP(); 
     
    SDI_on;
    NOP(); 
     
    SCK_on; 
    NOP(); 
} 
//-------------------------------------------------------------- 
void Write8bitcommand(unsigned char command)    // keep sel to low 
{ 
 unsigned char n=8; 
    nSEL_on;
    SCK_off;
    nSEL_off; 
    while(n--) 
    { 
         if(command&0x80) 
          Write1(); 
         else 
          Write0();    
              command = command << 1; 
    } 
    SCK_off;
}  


//-------------------------------------------------------------- 
unsigned char _spi_read(unsigned char address) 
{ 
 unsigned char result; 
 send_read_address(address); 
 result = read_8bit_data();  
 nSEL_on; 
 return(result); 
}  

//-------------------------------------------------------------- 
void _spi_write(unsigned char address, unsigned char data) 
{ 
 address |= 0x80; 
 Write8bitcommand(address); 
 send_8bit_data(data);  
 nSEL_on;
}  


//-------Defaults 38.400 baud---------------------------------------------- 
void RF22B_init_parameter(void) 
{ 
 ItStatus1 = _spi_read(0x03); // read status, clear interrupt   
 ItStatus2 = _spi_read(0x04); 
  _spi_write(0x06, 0x00);    // no wakeup up, lbd, 
  _spi_write(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode 
  _spi_write(0x09, 0x7f);  // c = 12.5p   
  _spi_write(0x0a, 0x05); 
  _spi_write(0x0b, 0x12);    // gpio0 TX State
  _spi_write(0x0c, 0x15);    // gpio1 RX State 
  //  -- Old PCB --
  //  _spi_write(0x0b, 0x15);    // gpio0 RX State
  //  _spi_write(0x0c, 0x12);    // gpio1 TX State 
  _spi_write(0x0d, 0xfd);    // gpio 2 micro-controller clk output 
  _spi_write(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION. 
  
  _spi_write(0x70, 0x2C);    // disable manchest 
  
       // 38.4Kbps data rate
  _spi_write(0x6e, 0x27); //case RATE_384K 
  _spi_write(0x6f, 0x52); //case RATE_384K
  
  _spi_write(0x1c, 0x1A); // case RATE_384K
  _spi_write(0x20, 0xA1);//  0x20 calculate from the datasheet= 500*(1+2*down3_bypass)/(2^ndec*RB*(1+enmanch)) 
  _spi_write(0x21, 0x20); // 0x21 , rxosr[10--8] = 0; stalltr = (default), ccoff[19:16] = 0; 
  _spi_write(0x22, 0x4E); // 0x22    ncoff =5033 = 0x13a9 
  _spi_write(0x23, 0xA5); // 0x23 
  _spi_write(0x24, 0x00); // 0x24 
  _spi_write(0x25, 0x1B); // 0x25 
  _spi_write(0x1D, 0x40); // 0x25 
  _spi_write(0x1E, 0x0A); // 0x25 
  
  _spi_write(0x2a, 0x1e); 
  
  
  _spi_write(0x30, 0x8c);    // enable packet handler, msb first, enable crc, 

  _spi_write(0x32, 0xf3);    // 0x32address enable for headere byte 0, 1,2,3, receive header check for byte 0, 1,2,3 
  _spi_write(0x33, 0x42);    // header 3, 2, 1,0 used for head length, fixed packet length, synchronize word length 3, 2, 
  _spi_write(0x34, 0x09);    // 7 default value or   // 64 nibble = 32byte preamble 
  _spi_write(0x36, 0x2d);    // synchronize word 
 _spi_write(0x37, 0xd4); 
 _spi_write(0x38, 0x00); 
 _spi_write(0x39, 0x00); 
 _spi_write(0x3a, RF_Header[0]);    // tx header 
 _spi_write(0x3b, RF_Header[1]); 
 _spi_write(0x3c, RF_Header[2]); 
 _spi_write(0x3d, RF_Header[3]); 
 _spi_write(0x3e, 8);    // total tx 17 byte 
 
  
  
    //RX HEADER
 _spi_write(0x3f, RF_Header[0]);   // check hearder 
 _spi_write(0x40, RF_Header[1]); 
 _spi_write(0x41, RF_Header[2]); 
 _spi_write(0x42, RF_Header[3]); 
 _spi_write(0x43, 0xff);    // all the bit to be checked 
 _spi_write(0x44, 0xff);    // all the bit to be checked 
 _spi_write(0x45, 0xff);    // all the bit to be checked 
 _spi_write(0x46, 0xff);    // all the bit to be checked 
  

  
  _spi_write(0x6d, 0x07); // 7 set power max power 
  _spi_write(0x79, 0x00);    // no hopping 
  #if (BAND== 0)
  _spi_write(0x7a, 0x06);    // 60khz step size (10khz x value) // no hopping 
  #else
  _spi_write(0x7a, 0x01);    // 10khz step size (10khz x value) // no hopping
  #endif
  _spi_write(0x71, 0x23); // Gfsk, fd[8] =0, no invert for Tx/Rx data, fifo mode, txclk -->gpio 
  _spi_write(0x72, 0x30); // frequency deviation setting to 19.6khz (for 38.4kbps)
 
  _spi_write(0x73, 0x00);   
  _spi_write(0x74, 0x00);    // no offset 
 

  //band 435.000
 
 _spi_write(0x75, 0x53);    
 _spi_write(0x76, 0x7D);    
 _spi_write(0x77, 0x00); 
}




//-------------------------------------------------------------- 
void send_read_address(unsigned char i) 
{ 
 i &= 0x7f; 
  
 Write8bitcommand(i); 
}  
//-------------------------------------------------------------- 
void send_8bit_data(unsigned char i) 
{ 
  unsigned char n = 8; 
  SCK_off;
    while(n--) 
    { 
         if(i&0x80) 
          Write1(); 
         else 
          Write0();    
         i = i << 1; 
    } 
   SCK_off;
}  
//-------------------------------------------------------------- 

unsigned char read_8bit_data(void) 
{ 
  unsigned char Result, i; 
  
 SCK_off;
 Result=0; 
    for(i=0;i<8;i++) 
    {                    //read fifo data byte 
       Result=Result<<1; 
       SCK_on;
       NOP(); 
       if(SDO_1) 
       { 
         Result|=1; 
       } 
       SCK_off;
       NOP(); 
     } 
    return(Result); 
}  
//-------------------------------------------------------------- 

//----------------------------------------------------------------------- 
void rx_reset(void) 
{ 
  _spi_write(0x07, RF22B_PWRSTATE_READY); 
  _spi_write(0x7e, 36);    // threshold for rx almost full, interrupt when 1 byte received 
  _spi_write(0x08, 0x03);    //clear fifo disable multi packet 
  _spi_write(0x08, 0x00);    // clear fifo, disable multi packet 
  _spi_write(0x07,RF22B_PWRSTATE_RX );  // to rx mode 
  _spi_write(0x05, RF22B_Rx_packet_received_interrupt); 
  ItStatus1 = _spi_read(0x03);  //read the Interrupt Status1 register 
  ItStatus2 = _spi_read(0x04);  
}  
//-----------------------------------------------------------------------    

void to_rx_mode(void) 
{  
 to_ready_mode(); 
 delay(50); 
 rx_reset(); 
 NOP(); 
}  

//-------------------------------------------------------------- 
 
//-------------------------------------------------------------- 
void to_ready_mode(void) 
{ 
 ItStatus1 = _spi_read(0x03);   
 ItStatus2 = _spi_read(0x04); 
 _spi_write(0x07, RF22B_PWRSTATE_READY); 
}  
//-------------------------------------------------------------- 

//--------------------------------------------------------------   
  
void frequency_configurator(long frequency){

  // frequency formulation from Si4432 chip's datasheet
  // original formulation is working with mHz values and floating numbers, I replaced them with kHz values.
  frequency = frequency / 10;
  frequency = frequency - 24000;
  #if (BAND== 0)
  frequency = frequency - 19000; // 19 for 430�439.9 MHz band from datasheet
  #else
  frequency = frequency - 21000; // 21 for 450�459.9 MHz band from datasheet
  #endif
  frequency = frequency * 64; // this is the Nominal Carrier Frequency (fc) value for register setting
  
  byte byte0 = (byte) frequency;
  byte byte1 = (byte) (frequency >> 8);
  
  _spi_write(0x76, byte1);    
  _spi_write(0x77, byte0); 

}

