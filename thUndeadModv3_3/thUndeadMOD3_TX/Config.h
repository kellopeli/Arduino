//###############################################################
//### Config.h Generated with thUndead's openLRS Configurator ###
//###############################################################

//####### TX BOARD TYPE #######
// 0 = Original M1 Tx Board
// 1 = OpenLRS Rx Board works as TX, reads your PPM signals from first servo port.
// 2 = Original M2 Tx Board
// 3 = OpenLRS Rx v2 Board works as TX, reads your PPM signals from first servo port.
#define TX_BOARD_TYPE 3

//####### BOOSTER #######
// 0 = No booster
// 1 = Booster
#define BOOSTER 0

//######### Band Select ##########
// 0 = 433Mhz
// 1 = 459Mhz
#define BAND 0

//######### TRANSMISSION VARIABLES ##########
#define CARRIER_FREQUENCY 437000  //  startup frequency

//####### Freq Hopping #######
// 1 = Enabled  0 = Disabled
#define FREQUENCY_HOPPING 1

//###### HOPPING CHANNELS #######
#define CHNO 7
static unsigned char hop_list[10] = {0,45,5,40,10,35,15,30,20,25};

//###### RF DEVICE ID HEADERS #######
// Change this 4 byte values for isolating your transmission, RF module accepts only data with same header
static unsigned char RF_Header[4] = {'T','R','a','X'};

//###### SERIAL PORT SPEED #######
#define SERIAL_BAUD_RATE 115200 //115.200 baud serial port speed

