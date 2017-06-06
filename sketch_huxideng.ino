#include <Arduino.h>
#include <Adafruit_NeoPixel.h> 
#define NUMPIXELS      16
#define PIN           24
#define   LIGHT_DELAY    30    //(30ms)
bool direct = true;
uint8_t  lightI = 1;

uint32_t previousMillis = 0;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
void setup() {
  // put your setup code here, to run once:
  strip.begin();
  strip.show();
}

void loop() {
  // put your main code here, to run repeatedly:
  colorWipe(strip.Color(255, 255,255), 1);
}
unsigned long gokit_time_ms(void)
{
  return millis();
}
/*******************************************************
      function      : gokit_time_m

******************************************************/
unsigned long gokit_time_s(void)
{
  return millis() / 1000;
}
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}
void breathe(uint8_t wait) {
  uint32_t currentMillis = gokit_time_ms();
  if (currentMillis - previousMillis >= LIGHT_DELAY) {
    previousMillis=currentMillis;
    if (direct == true) {
      lightI++;
    } else {
      lightI--;
    }
    if ((lightI == 255) || (lightI == 0)) {
      direct = !direct;
      delay(wait);
    }
    strip.setBrightness(lightI);
    colorWipe(strip.Color(255,0,0), 1);
    strip.show();
  }
}
