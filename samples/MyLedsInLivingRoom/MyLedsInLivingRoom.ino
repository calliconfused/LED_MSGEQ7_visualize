/*
 * define all variables to adjust the controller
 */

// digital and analog pins for MSGEQ7

#define dAnalogPinL             0   // MSGEQ7 OUT
#define dAnalogPinR             1   // MSGEQ7 OUT
#define dStrobePin              2   // MSGEQ7 STROBE
#define dResetPin               4   // MSGEQ7 RESET
#define MSGEQ7_INTERVAL         ReadsPerSecond(50)
#define MSGEQ7_SMOOTH           75  // Range: 0-255

// LED strip

#define dColorRange             RBG
#define dTypeStrip              WS2812
#define dBrightness             50 

#define dNumberLedStrips        2

#define dNumberLedsStrip1       20
#define dDataPinStrip1          25

#define dNumberLedsStrip2       20
#define dDataPinStrip2          29

#define dStartNumberLedStrip1   0
#define dStartNumberLedStrip2   dStartNumberLedStrip1 + dNumberLedsStrip1

#define dNumberLedsTotal dNumberLedsStrip1 + dNumberLedsStrip2

#define SPECTRUM_EQ7  6
#define SPECTRUM_SEC  6

// display properties

#define dDisplayBarYPosition    25  // vertical position of bars
#define dDisplayBarMaxLength    25  // max length of bars
#define dDisplayBarWidth        9   // max width of bars
#define dDisplayBarWidthOffset  1   // space between bars
#define dDisplayTextYPosition   32  // lowest position of text

/* 
 * u8g2
 */

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0); 
byte bDisplayBarXPosition;
byte bDisplayBarLength;

/*
 * MSGEQ7
 */

#include "MSGEQ7.h"
CMSGEQ7<MSGEQ7_SMOOTH, dResetPin, dStrobePin, dAnalogPinL, dAnalogPinR> MSGEQ7;

/*
 * FastLED
 */

#include <FastLED.h>
CRGB leds[dNumberLedsTotal];
uint8_t iHueValue = 0;

void setup() {
  
  Serial2.begin(115200);
  Serial.begin(115200);
  u8g2.begin();
  
  FastLED.addLeds<dTypeStrip, dDataPinStrip1, dColorRange>(leds, dStartNumberLedStrip1, dNumberLedsStrip1);
  FastLED.addLeds<dTypeStrip, dDataPinStrip2, dColorRange>(leds, dStartNumberLedStrip2, dNumberLedsStrip2);
  FastLED.setBrightness(dBrightness);
  FastLED.clear();

  for (int i = 0; i < dNumberLedsTotal; i++) {
    leds[i] = CRGB(0, 0, 0);
  }

  FastLED.show();    
  
  analogReference(DEFAULT); // 5V
  MSGEQ7.begin();

}
 
void loop() {
  
  bool newReading = MSGEQ7.read(MSGEQ7_INTERVAL);

  if (newReading) {

    uint8_t bSpectrumValueL0 = MSGEQ7.get(MSGEQ7_0, 0);
    uint8_t bSpectrumValueL1 = MSGEQ7.get(MSGEQ7_1, 0);
    uint8_t bSpectrumValueL2 = MSGEQ7.get(MSGEQ7_2, 0);
    uint8_t bSpectrumValueL3 = MSGEQ7.get(MSGEQ7_3, 0);
    uint8_t bSpectrumValueL4 = MSGEQ7.get(MSGEQ7_4, 0);
    uint8_t bSpectrumValueL5 = MSGEQ7.get(MSGEQ7_5, 0);

    uint8_t bSpectrumValueR0 = MSGEQ7.get(MSGEQ7_0, 1);
    uint8_t bSpectrumValueR1 = MSGEQ7.get(MSGEQ7_1, 1);
    uint8_t bSpectrumValueR2 = MSGEQ7.get(MSGEQ7_2, 1);
    uint8_t bSpectrumValueR3 = MSGEQ7.get(MSGEQ7_3, 1);
    uint8_t bSpectrumValueR4 = MSGEQ7.get(MSGEQ7_4, 1);
    uint8_t bSpectrumValueR5 = MSGEQ7.get(MSGEQ7_5, 1);
    
    vShowOnDisplay(bSpectrumValueL0, bSpectrumValueR0, bSpectrumValueL1, bSpectrumValueR1,
                   bSpectrumValueL2, bSpectrumValueR2, bSpectrumValueL3, bSpectrumValueR3, 
                   bSpectrumValueL4, bSpectrumValueR4, bSpectrumValueL5, bSpectrumValueR5);

    Serial.print("L");
    Serial.print(MSGEQ7.getVolume(0));
    Serial.print(" R");
    Serial.println(MSGEQ7.getVolume(1));
    
  }
  
  //vShowOnDisplay(bSpectrumValueL[0], bSpectrumValueR[0], bSpectrumValueL[1], bSpectrumValueR[1],
  //               bSpectrumValueL[2], bSpectrumValueR[2], bSpectrumValueL[3], bSpectrumValueR[3], 
  //               bSpectrumValueL[4], bSpectrumValueR[4], bSpectrumValueL[5], bSpectrumValueR[5]);

  FastLED.clear();

  //vShowLedBar(map(max(bSpectrumValueL[1], bSpectrumValueR[1]), 0, 255, 0, dNumberLedsStrip1), dStartNumberLedStrip1, iHueValue);
  //vShowLedBar(map(max(bSpectrumValueL[3], bSpectrumValueR[3]), 0, 255, 0, dNumberLedsStrip2), dStartNumberLedStrip2, iHueValue);
  
  iHueValue++;

  if (iHueValue==256) {
    iHueValue = 0;
  }

  FastLED.show();

}

void vShowLedBar(byte bVolVal, int iLedStart, int iHueValue) {

  for (int i = iLedStart; i - iLedStart < bVolVal; i++) {
    leds[i] = CHSV(iHueValue, 255, 192);
  }
  
}

void vShowOnDisplay(byte bVolValL0, byte bVolValR0, byte bVolValL1, byte bVolValR1, 
                    byte bVolValL2, byte bVolValR2, byte bVolValL3, byte bVolValR3,
                    byte bVolValL4, byte bVolValR4, byte bVolValL5, byte bVolValR5) {

  u8g2.setFont(u8g2_font_4x6_mf);
  u8g2.firstPage();
  
  do {

    bDisplayBarXPosition = 1;
    bDisplayBarLength = map(bVolValL0, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 0, dDisplayTextYPosition);
    u8g2.print("63");

    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL1, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition - 2, dDisplayTextYPosition);
    u8g2.print("160");
    
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL2, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition + 0, dDisplayTextYPosition);
    u8g2.print("400");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL3, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition + 2, dDisplayTextYPosition);
    u8g2.print("1k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL4, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition + 1, dDisplayTextYPosition);
    u8g2.print("2k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL5, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition + 1, dDisplayTextYPosition);
    u8g2.print("6k");
        
    bDisplayBarXPosition = 64;
    bDisplayBarLength = map(bVolValR5, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 0, dDisplayTextYPosition);
    u8g2.print("6k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR4, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 0, dDisplayTextYPosition);
    u8g2.print("2k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR3, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition - 1, dDisplayTextYPosition);
    u8g2.print("1k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR2, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition - 2, dDisplayTextYPosition);
    u8g2.print("400");

    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR1, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 0, dDisplayTextYPosition);
    u8g2.print("160");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR0, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 2, dDisplayTextYPosition);
    u8g2.print("63");
    
  } while ( u8g2.nextPage() );
  
}

