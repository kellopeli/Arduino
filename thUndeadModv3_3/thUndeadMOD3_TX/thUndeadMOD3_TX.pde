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

//############ VARIABLES ########################

unsigned char CH =0;
unsigned char i;
bool pressed,willfs =0;
unsigned long fstime,fstime2 = 0;
unsigned long bzzz = 0;

volatile unsigned char  servo_buf[8];
volatile unsigned char channel_no=0;
volatile unsigned int transmitted=1;
volatile unsigned char channel_count=0;


//####### Board Pinouts #########

#if (TX_BOARD_TYPE == 0)
    #define PPM_IN A5
    #define RF_OUT_INDICATOR A4
    #define BUZZER 9
    #define BTN 10
    #define Red_LED 12
    #define Green_LED 11
    
    #define Red_LED_ON  PORTB |= _BV(4);
    #define Red_LED_OFF  PORTB &= ~_BV(4);
    
    #define Green_LED_ON   PORTB |= _BV(3);
    #define Green_LED_OFF  PORTB &= ~_BV(3);
    
    #define PPM_Pin_Interrupt_Setup  PCMSK1 = 0x20;PCICR|=(1<<PCIE1);
    #define PPM_Signal_Interrupt PCINT1_vect
    #define PPM_Signal_Edge_Check (PINC & 0x20)==0x20
    
#endif
    
#if (TX_BOARD_TYPE == 1)
    #define PPM_IN 5
    #define RF_OUT_INDICATOR 6
    #define BUZZER 7
    #define BTN 8
    
    #define Red_LED A3
    #define Green_LED A2
    
    #define Red_LED_ON  PORTC &= ~_BV(2);PORTC |= _BV(3);
    #define Red_LED_OFF  PORTC &= ~_BV(2);PORTC &= ~_BV(3);

    #define Green_LED_ON  PORTC &= ~_BV(3);PORTC |= _BV(2);
    #define Green_LED_OFF  PORTC &= ~_BV(3);PORTC &= ~_BV(2);
    
    #define PPM_Pin_Interrupt_Setup  PCMSK2 = 0x20;PCICR|=(1<<PCIE2);
    #define PPM_Signal_Interrupt PCINT2_vect
    #define PPM_Signal_Edge_Check (PIND & 0x20)==0x20
    
#endif  

#if (TX_BOARD_TYPE == 2)
    #define PPM_IN 3
    #define RF_OUT_INDICATOR A0
    #define BUZZER 10
    #define BTN 11
    #define Red_LED 13
    #define Green_LED 12
    
    #define Red_LED_ON  PORTB |= _BV(5);
    #define Red_LED_OFF  PORTB &= ~_BV(5);
    
    #define Green_LED_ON   PORTB |= _BV(4);
    #define Green_LED_OFF  PORTB &= ~_BV(4);
    
    #define PPM_Pin_Interrupt_Setup  PCMSK2 = 0x08;PCICR|=(1<<PCIE2);
    #define PPM_Signal_Interrupt PCINT2_vect
    #define PPM_Signal_Edge_Check (PIND & 0x08)==0x08
    
#endif

#if (TX_BOARD_TYPE == 3)
    #define PPM_IN 3
    #define RF_OUT_INDICATOR 5
    #define BUZZER 6
    #define BTN 7
    
    #define Red_LED A3
    #define Green_LED 13
    
    #define Red_LED_ON  PORTC |= _BV(3);
    #define Red_LED_OFF  PORTC &= ~_BV(3);

    #define Green_LED_ON  PORTB |= _BV(5);
    #define Green_LED_OFF  PORTB |= _BV(5);
    
    #define PPM_Pin_Interrupt_Setup  PCMSK2 = 0x08;PCICR|=(1<<PCIE2);
    #define PPM_Signal_Interrupt PCINT2_vect
    #define PPM_Signal_Edge_Check (PIND & 0x08)==0x08
#endif    


//####### RFM22B Pinouts #########

#if ((TX_BOARD_TYPE == 0)||(TX_BOARD_TYPE == 1))
      //## RFM22B Pinouts for Public Edition (M1 or Rx v1)
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
      
      #define SDO_pin A0
      #define SDI_pin A1        
      #define SCLK_pin 2 
      #define IRQ_pin 3
      #define nSel_pin 4
#endif      

#if (TX_BOARD_TYPE == 2)
      //## RFM22B Pinouts for Public Edition (M2)
      #define  nIRQ_1 (PIND & 0x04)==0x04 //D2
      #define  nIRQ_0 (PIND & 0x04)==0x00 //D2
      
      #define  nSEL_on PORTD |= (1<<4) //D4
      #define  nSEL_off PORTD &= 0xEF //D4
      
      #define  SCK_on PORTD |= (1<<7) //D7
      #define  SCK_off PORTD &= 0x7F //D7
      
      #define  SDI_on PORTB |= (1<<0) //B0
      #define  SDI_off PORTB &= 0xFE //B0
      
      #define  SDO_1 (PINB & 0x02) == 0x02 //B1
      #define  SDO_0 (PINB & 0x02) == 0x00 //B1
      
      #define SDO_pin 9
      #define SDI_pin 8        
      #define SCLK_pin 7 
      #define IRQ_pin 2
      #define nSel_pin 4
#endif  

#if (TX_BOARD_TYPE == 3)
      //## RFM22B Pinouts for Public Edition (Rx v2)
      #define  nIRQ_1 (PIND & 0x04)==0x04 //D2
      #define  nIRQ_0 (PIND & 0x04)==0x00 //D2
      
      #define  nSEL_on PORTD |= (1<<4) //D4
      #define  nSEL_off PORTD &= 0xEF //D4
      
      #define  SCK_on PORTC |= (1<<2) //A2
      #define  SCK_off PORTC &= 0xFB //A2
      
      #define  SDI_on PORTC |= (1<<1) //A1
      #define  SDI_off PORTC &= 0xFD //A1
      
      #define  SDO_1 (PINC & 0x01) == 0x01 //A0
      #define  SDO_0 (PINC & 0x01) == 0x00 //A0
      
      #define SDO_pin A0
      #define SDI_pin A1        
      #define SCLK_pin A2 
      #define IRQ_pin 2
      #define nSel_pin 4
#endif  

//####### MAIN LOOP #########


void setup() {  


  //RF module pins
  pinMode(SDO_pin, INPUT); //SDO
  pinMode(SDI_pin, OUTPUT); //SDI        
  pinMode(SCLK_pin, OUTPUT); //SCLK
  pinMode(IRQ_pin, INPUT); //IRQ
  pinMode(nSel_pin, OUTPUT); //nSEL

  //LED and other interfaces
  pinMode(Red_LED, OUTPUT); //RED LED
  pinMode(Green_LED, OUTPUT); //GREEN LED
  pinMode(BUZZER, OUTPUT); //Buzzer
  pinMode(BTN, INPUT); //Buton


  pinMode(PPM_IN, INPUT); //PPM from TX 
  pinMode(RF_OUT_INDICATOR, OUTPUT);

  Serial.begin(SERIAL_BAUD_RATE);



  PPM_Pin_Interrupt_Setup // turnon pinchange interrupts



  for (unsigned char i=0;i<8;i++) // set defoult servo position values.
    SetServoPos(i,3000); // set the center position

  TCCR1B   =   0x00;   //stop timer
  TCNT1H   =   0x00;   //setup
  TCNT1L   =   0x00;
  ICR1     =   60005;   // used for TOP, makes for 50 hz
  TCCR1A   =   0x02;   
  TCCR1B   =   0x1A; //start timer with 1/8 prescaler for measuring 0.5us PPM resolution
}

//##### PPM INPUT INTERRUPT #####
//Port change interrupt detects the PPM signal's rising edge and calculates the signal timing from Timer1's value.

ISR(PPM_Signal_Interrupt){

  unsigned int time_temp;
  unsigned int servo_temp;
  unsigned int servo_temp2;

  if (PPM_Signal_Edge_Check) // Only works with rising edge of the signal
  {
    time_temp = TCNT1; // read the timer1 value
    TCNT1 = 0; // reset the timer1 value for next
    if (channel_no<14) channel_no++; 


    if (time_temp > 8000) // new frame detection : >4ms LOW
    {	
      channel_count = channel_no;
      channel_no = 0;
      transmitted = 0;                               
    }
    else
    {
      if ((time_temp>1500) && (time_temp<4500)) // check the signal time and update the channel if it is between 750us-2250us
      {
        //Servo_Buffer[(2*channel_no)-1] = (byte) (time_temp >> 8); // write the high byte of the value into the servo value buffer.
        //Servo_Buffer[2*channel_no] =  (byte) (time_temp); // write the low byte of the value into the servo value buffer.                  

        SetServoPos(channel_no-1,time_temp);

      }
    }
  }

}


//############ MAIN LOOP ##############
void loop() {

  unsigned char i;
  unsigned char no =0;


  RF22B_init_parameter(); 
  frequency_configurator(CARRIER_FREQUENCY); // Calibrate the RF module for this frequency, frequency hopping starts from this frequency.

  sei();

  digitalWrite(BUZZER, HIGH);
  digitalWrite(BTN, HIGH);
  Red_LED_ON ;
  delay(100);	

  Check_Button();

  Red_LED_OFF;
  digitalWrite(BUZZER, LOW);

  digitalWrite(RF_OUT_INDICATOR,LOW);
  digitalWrite(PPM_IN,HIGH);

  transmitted = 0;
  rx_reset;
 

  while(1)
  {    /* MAIN LOOP */
  

    
    if (_spi_read(0x0C)==0) // detect the locked module and reboot
    {
      Red_LED_ON;  
      RF22B_init_parameter();
      frequency_configurator(CARRIER_FREQUENCY);
      rx_reset;
      Red_LED_OFF;
    }


    //Green LED will be on during transmission  
    Green_LED_ON ;



    // Send the data over RF

    to_tx_mode();
       
   
           

    transmitted = 1;

    //Green LED will be OFF
    Green_LED_OFF; 
    

    #if (FREQUENCY_HOPPING==1)
    Hopping();//Hop to the next frequency
    #endif
    
    buttonHOP();
    
    delay(4); // a delay but not needed now...
    
}

    
  

}


//####### FUNCTIONS #########

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
  
//############# GREEN LED BLINK #################
void Green_LED_Blink(unsigned short blink_count)
  {
  unsigned char i;
  for (i=0;i<blink_count;i++)
     {
     delay(100);
     Green_LED_ON;
     delay(100);
     Green_LED_OFF;
     }
  }  

//############# FREQUENCY HOPPING ################# thUndead FHSS
#if (FREQUENCY_HOPPING==1)


void Hopping(void)
{

CH++;
if (CH >CHNO) CH=0;

_spi_write(0x79, hop_list[CH]);


}
#endif


//############# BUTTON CHECK #################
void Check_Button(void)
{
unsigned long time,loop_time;


 if (digitalRead(BTN)==0) // Check the button
    {
    delay(1000); // wait for 1000mS when buzzer ON 
    digitalWrite(BUZZER, LOW); // Buzzer off
    
    time = millis();  //set the current time
    loop_time = time; 
    
    while ((digitalRead(BTN)==0) && (loop_time < time + 4000)) // wait for button reelase if it is already pressed.
        {
         loop_time = millis(); 
        }     
        
    //Check the button again. If it is already pressed start the binding proscedure    
    if (digitalRead(BTN)!=0)
        // if button released, reduce the power for range test.
        {
        _spi_write(0x6d, 0x00);
        }  
        
        
    }
  
  
}



void SetServoPos (unsigned char channel,int value)
{
    
  servo_buf[channel] = map(value,1501,4499,0,255);
 
}

//############ Le Stuff :D ########################
 void buttonHOP(void)
{
  
  bzzz++;
  if ((bzzz>10))
    {
    digitalWrite(BUZZER, LOW); // Buzzer off 
    bzzz=0;
    }
   if (digitalRead(BTN)==0)
     {
    
    
    
       fstime ++;
      
      
                if ((fstime > 30 ))
                      {
                       
                      fstime =0;
                     
                      willfs =1; 
                      bzzz=0;
                      digitalWrite(BUZZER, HIGH);  
                      }  
     }  


                        else { fstime =0;   }
        
         
}


// **********************************************************
// **      RFM22B/Si4432 control functions for OpenLRS     **
// **       This Source code licensed under GPL            **
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
void to_tx_mode(void); 
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
void to_sleep_mode(void); 
 
 
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

  _spi_write(0x0d, 0xfd);    // gpio 2 micro-controller clk output 
  _spi_write(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION. 
  
  _spi_write(0x70, 0x2C);    // disable manchest 
  
       // 9.6Kbps data rate
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
  _spi_write(0x34, 0x06);    // 7 default value or   // 64 nibble = 32byte preamble 
  _spi_write(0x36, 0x2d);    // synchronize word 
 _spi_write(0x37, 0xd4); 
 _spi_write(0x38, 0x00); 
 _spi_write(0x39, 0x00); 
 _spi_write(0x3a, RF_Header[0]);    // tx header 
 _spi_write(0x3b, RF_Header[1]); 
 _spi_write(0x3c, RF_Header[2]); 
 _spi_write(0x3d, RF_Header[3]); 
 _spi_write(0x3e, 8);    // total tx 3 byte 
 
  
  
    //RX HEADER
 _spi_write(0x3f, RF_Header[0]);   // check hearder 
 _spi_write(0x40, RF_Header[1]); 
 _spi_write(0x41, RF_Header[2]); 
 _spi_write(0x42, RF_Header[3]); 
 _spi_write(0x43, 0xff);    // all the bit to be checked 
 _spi_write(0x44, 0xff);    // all the bit to be checked 
 _spi_write(0x45, 0xff);    // all the bit to be checked 
 _spi_write(0x46, 0xff);    // all the bit to be checked 
  

  
  #if (BOOSTER == 0)
  _spi_write(0x6d, 0x07); // 7 set power max power 
  #else
  _spi_write(0x6d, 0x06); // 6 set power 50mw for booster 
  #endif
  
  _spi_write(0x79, 0x00);    // no hopping 
  
  #if (BAND== 0)
  _spi_write(0x7a, 0x06);    // 60khz step size (10khz x value) // no hopping 
  #else
  _spi_write(0x7a, 0x01); // 10khz step size (10khz x value) // no hopping 
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


void to_tx_mode(void) 
{ 
 
 unsigned char i;
 to_ready_mode();
 
  _spi_write(0x08, 0x03);    // disABLE AUTO TX MODE, enable multi packet clear fifo 
  _spi_write(0x08, 0x00);    // disABLE AUTO TX MODE, enable multi packet, clear fifo 
  
    // ph +fifo mode 
  _spi_write(0x34, 0x06);     // 64 nibble = 32byte preamble 
  _spi_write(0x3e, 8);    // total tx 8 byte 

  
    
  
  _spi_write(0x7f, servo_buf[0]); 
  _spi_write(0x7f, servo_buf[1]); 
  _spi_write(0x7f, servo_buf[2]); 
  _spi_write(0x7f, servo_buf[3]); 
  _spi_write(0x7f, servo_buf[4]); 
  _spi_write(0x7f, servo_buf[5]);
 
 if (willfs==1)
    { 
    willfs=0;
    servo_buf[6]=0;
    servo_buf[7]=0;
    
    }
  _spi_write(0x7f, servo_buf[6]); 
  _spi_write(0x7f, servo_buf[7]); 
    
 /*
  Serial.print(servo_buf[0],DEC);
  Serial.print(" ");
  Serial.print(rf_buf[1],DEC);
  Serial.print(" ");
  Serial.print(rf_buf[2],DEC);
  Serial.print(" ");
  Serial.print(rf_buf[3],DEC);
  Serial.print(" ");
  Serial.print(rf_buf[4],DEC);
  Serial.print(" ");
  Serial.print(rf_buf[5],DEC);
  Serial.print(" ");
  Serial.print(rf_buf[6],DEC);
  Serial.print(" ");
  Serial.print(rf_buf[7],DEC);
  Serial.println(" "); */
  
    
      
 _spi_write(0x05, RF22B_PACKET_SENT_INTERRUPT);  
 ItStatus1 = _spi_read(0x03);      //read the Interrupt Status1 register 
 ItStatus2 = _spi_read(0x04); 
  _spi_write(0x07, RF22B_PWRSTATE_TX);    // to tx mode 

  while(nIRQ_1);

  to_ready_mode();

}  
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
 
