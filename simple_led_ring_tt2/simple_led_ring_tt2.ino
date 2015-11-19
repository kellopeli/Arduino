// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library
#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      16
#define BRIGHTNESS_LO     10
#define BRIGHTNESS_HI     30

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 25; // delay 

void setup() {
  pixels.begin(); // This initializes the NeoPixel library.
}

void loop() {
  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  //color_loops();
  knight_rider();
}

void knight_rider()
{
  for(int i=0;i<NUMPIXELS;i++){
    light_4(BRIGHTNESS_HI, 0, 0, i);
  }
  flash_half(BRIGHTNESS_HI,0,0);
  flash_half(BRIGHTNESS_HI,BRIGHTNESS_HI,0);
  flash_half(0,BRIGHTNESS_HI,0);
  flash_half(BRIGHTNESS_HI,0,BRIGHTNESS_HI);
  flash_half(0,0,BRIGHTNESS_HI);
  flash_half(0,BRIGHTNESS_HI,BRIGHTNESS_HI);
  flash_half(BRIGHTNESS_HI,BRIGHTNESS_HI,BRIGHTNESS_HI);
}

void color_loops()
{
  loop_once(0,BRIGHTNESS_HI,0);
  loop_once(0,0,BRIGHTNESS_HI);
  loop_once(BRIGHTNESS_HI,0,0);
  loop_once(BRIGHTNESS_HI,BRIGHTNESS_HI,0);
  loop_once(0,BRIGHTNESS_HI,BRIGHTNESS_HI);
  loop_once(BRIGHTNESS_HI,0,BRIGHTNESS_HI);
  loop_once(BRIGHTNESS_HI,BRIGHTNESS_HI,BRIGHTNESS_HI);
}

void flash(int red, int green, int blue)
{
    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i%NUMPIXELS, pixels.Color(red, green, blue));
    }
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).

    for (int i = 0; i < NUMPIXELS; i++) {
      pixels.setPixelColor(i%NUMPIXELS, pixels.Color(0,0,0));
    }
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
}

void flash_half(int red, int green, int blue)
{
    for (int i = 0; i < NUMPIXELS/2; i++) {
      pixels.setPixelColor(i%NUMPIXELS, pixels.Color(red, green, blue));
    }
    for (int i = 0; i < NUMPIXELS/2; i++) {
      pixels.setPixelColor((NUMPIXELS/2+i)%NUMPIXELS, pixels.Color(0, 0, 0));
    }
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).

    for (int i = 0; i < NUMPIXELS/2; i++) {
      pixels.setPixelColor(i%NUMPIXELS, pixels.Color(0, 0, 0));
    }
    for (int i = 0; i < NUMPIXELS/2; i++) {
      pixels.setPixelColor((NUMPIXELS/2+i)%NUMPIXELS, pixels.Color(red, green, blue));
    }
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
}

void light_4(int red, int green, int blue, int i)
{
    pixels.setPixelColor(i%NUMPIXELS, pixels.Color(0,0,0));
    pixels.setPixelColor((i+1)%NUMPIXELS, pixels.Color(red/4,green/4,blue/4));
    pixels.setPixelColor((i+2)%NUMPIXELS, pixels.Color(red,green,blue)); 
    pixels.setPixelColor((i+3)%NUMPIXELS, pixels.Color(red,green,blue));
    pixels.setPixelColor((i+4)%NUMPIXELS, pixels.Color(red/4,green/4,blue/4));
    pixels.setPixelColor((i+5)%NUMPIXELS, pixels.Color(0,0,0));
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
}

void loop_once(int red, int green, int blue)
{  
  for(int i=0;i<NUMPIXELS;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i%NUMPIXELS, pixels.Color(0,0,0));
    pixels.setPixelColor((i+1)%NUMPIXELS, pixels.Color(red/6,green/6,blue/6));
    pixels.setPixelColor((i+2)%NUMPIXELS, pixels.Color(red/5,green/5,blue/5));
    pixels.setPixelColor((i+3)%NUMPIXELS, pixels.Color(red/4,green/4,blue/4));
    pixels.setPixelColor((i+4)%NUMPIXELS, pixels.Color(red/3,green/3,blue/3));
    pixels.setPixelColor((i+5)%NUMPIXELS, pixels.Color(red/2,green/2,blue/2));
    pixels.setPixelColor((i+6)%NUMPIXELS, pixels.Color(red,green,blue));
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
  }
}


