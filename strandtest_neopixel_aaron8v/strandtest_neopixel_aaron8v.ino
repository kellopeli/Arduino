#include <Adafruit_NeoPixel.h>

#define PIN 6
#define NUMPIXELS 16
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // Some example procedures showing how to display to the pixels:
  //colorWipe(strip.Color(255/5, 0, 0), 50); // Red
  //colorWipe(strip.Color(0, 255/5, 0), 50); // Green
  //colorWipe(strip.Color(0, 0, 255/5), 50); // Blue
  // Send a theater pixel chase in...

  //theaterChase(strip.Color(127/5, 127/5, 127/5), 50); // White
  //theaterChase(strip.Color(127/5,   0,   0), 50); // Red
  //theaterChase(strip.Color(  0,   0, 127/5), 50); // Blue
  theaterChaseRainbow(50);
  flash();
  //rainbow(20);
  //rainbowCycle(20);
}

void flash()
{
  for (int i = 0; i < 50; i++) {
    flash_half(255,255,255); 
  } 
}

void flash_half(int red, int green, int blue)
{
    for (int i = 0; i < NUMPIXELS/2; i++) {
      strip.setPixelColor(i%NUMPIXELS, strip.Color(red, green, blue));
    }
    for (int i = 0; i < NUMPIXELS/2; i++) {
      strip.setPixelColor((NUMPIXELS/2+i)%NUMPIXELS, strip.Color(0, 0, 0));
    }
    strip.show(); // This sends the updated pixel color to the hardware.
    delay(30); // Delay for a period of time (in milliseconds).

    for (int i = 0; i < NUMPIXELS/2; i++) {
      strip.setPixelColor(i%NUMPIXELS, strip.Color(0, 0, 0));
    }
    for (int i = 0; i < NUMPIXELS/2; i++) {
      strip.setPixelColor((NUMPIXELS/2+i)%NUMPIXELS, strip.Color(red, green, blue));
    }
    strip.show(); // This sends the updated pixel color to the hardware.
    delay(30); // Delay for a period of time (in milliseconds).
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();
     
      delay(wait);
     
      for (int i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
        }
        strip.show();
       
        delay(wait);
       
        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, 0);        //turn every third pixel off
        }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

