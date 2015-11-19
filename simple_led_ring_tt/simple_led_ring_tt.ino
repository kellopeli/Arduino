// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library
#include <Adafruit_NeoPixel.h>

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      16
#define BRIGHTNESS_LO     10
#define BRIGHTNESS_HI     25

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 100; // delay 

void setup() {
  pixels.begin(); // This initializes the NeoPixel library.
}

void loop() {
  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  loop_once(0,BRIGHTNESS_HI,0);

  for(int i=0;i<NUMPIXELS;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.
    pixels.setPixelColor((i+1)%NUMPIXELS, pixels.Color(0,0,BRIGHTNESS_LO)); // Moderately bright green color.
    pixels.setPixelColor((i+2)%NUMPIXELS, pixels.Color(0,0,BRIGHTNESS_HI)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
  }
  for(int i=0;i<NUMPIXELS;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.
    pixels.setPixelColor((i+1)%NUMPIXELS, pixels.Color(BRIGHTNESS_LO,0,0)); // Moderately bright green color.
    pixels.setPixelColor((i+2)%NUMPIXELS, pixels.Color(BRIGHTNESS_HI,0,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
  }
  for(int i=0;i<NUMPIXELS;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.
    pixels.setPixelColor((i+1)%NUMPIXELS, pixels.Color(0,BRIGHTNESS_LO,BRIGHTNESS_LO)); // Moderately bright green color.
    pixels.setPixelColor((i+2)%NUMPIXELS, pixels.Color(0,BRIGHTNESS_HI,BRIGHTNESS_HI)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
  }
  for(int i=0;i<NUMPIXELS;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.
    pixels.setPixelColor((i+1)%NUMPIXELS, pixels.Color(BRIGHTNESS_LO,BRIGHTNESS_LO,BRIGHTNESS_LO)); // Moderately bright green color.
    pixels.setPixelColor((i+2)%NUMPIXELS, pixels.Color(BRIGHTNESS_HI,BRIGHTNESS_HI,BRIGHTNESS_HI)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
  }
}

void loop_once(int red, int green, int blue)
{  
  for(int i=0;i<NUMPIXELS;i++){
    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(0,0,0)); // Moderately bright green color.
    pixels.setPixelColor((i+1)%NUMPIXELS, pixels.Color(0,green/2,0)); // Moderately bright green color.
    pixels.setPixelColor((i+2)%NUMPIXELS, pixels.Color(0,green,0)); // Moderately bright green color.
    pixels.show(); // This sends the updated pixel color to the hardware.
    delay(delayval); // Delay for a period of time (in milliseconds).
  }
}

