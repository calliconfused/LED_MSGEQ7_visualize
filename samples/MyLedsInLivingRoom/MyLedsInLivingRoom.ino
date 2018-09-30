/*
 * define all variables to adjust the controller
 */

// digital and analog pins for MSGEQ7

#define dAnalogPinL             0   // MSGEQ7 OUT
#define dAnalogPinR             1   // MSGEQ7 OUT
#define dStrobePin              2   // MSGEQ7 STROBE
#define dResetPin               4   // MSGEQ7 RESET
#define MSGEQ7_INTERVAL         ReadsPerSecond(50)
#define MSGEQ7_SMOOTH           50  // Range: 0-255
#define MSGEQ7_STARTVOL         40  // to adjust when the script will start to work

// LED strip

#define dColorRange             RBG
#define dTypeStrip              WS2812
#define dBrightness             80 

#define dNumberLedsStrip1       40
#define dDataPinStrip1          25
#define dNumberLedsStrip2       40
#define dDataPinStrip2          29
#define dNumberLedsStrip3       40
#define dDataPinStrip3          37

#define dStartNumberLedStrip1   0
#define dStartNumberLedStrip2   dStartNumberLedStrip1 + dNumberLedsStrip1
#define dStartNumberLedStrip3   dStartNumberLedStrip2 + dNumberLedsStrip2

#define dNumberLedsTotal dNumberLedsStrip1 + dNumberLedsStrip2 + dNumberLedsStrip3

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

uint8_t bSpectrumValueL[6];
uint8_t bSpectrumValueR[6];

/*
 * FastLED
 */

#include <FastLED.h>
CRGB leds[dNumberLedsTotal];
uint8_t iHueValue = 0;
int iPoint = 0;

void setup() {
  
  Serial2.begin(115200);
  Serial.begin(115200);
  u8g2.begin();
  
  FastLED.addLeds<dTypeStrip, dDataPinStrip1, dColorRange>(leds, dStartNumberLedStrip1, dNumberLedsStrip1);
  FastLED.addLeds<dTypeStrip, dDataPinStrip2, dColorRange>(leds, dStartNumberLedStrip2, dNumberLedsStrip2);
  FastLED.addLeds<dTypeStrip, dDataPinStrip3, dColorRange>(leds, dStartNumberLedStrip3, dNumberLedsStrip3);
  
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

    if (MSGEQ7.getVolume(0) > MSGEQ7_STARTVOL) {

      bSpectrumValueL[0] = MSGEQ7.get(MSGEQ7_0, 0);
      bSpectrumValueL[1] = MSGEQ7.get(MSGEQ7_1, 0);
      bSpectrumValueL[2] = MSGEQ7.get(MSGEQ7_2, 0);
      bSpectrumValueL[3] = MSGEQ7.get(MSGEQ7_3, 0);
      bSpectrumValueL[4] = MSGEQ7.get(MSGEQ7_4, 0);
      bSpectrumValueL[5] = MSGEQ7.get(MSGEQ7_5, 0);
      
    }
    
    else {

      if (bSpectrumValueL[0] > 0) { bSpectrumValueL[0] = bSpectrumValueL[0] - 1; }
      if (bSpectrumValueL[1] > 0) { bSpectrumValueL[1] = bSpectrumValueL[1] - 1; }
      if (bSpectrumValueL[2] > 0) { bSpectrumValueL[2] = bSpectrumValueL[2] - 1; }
      if (bSpectrumValueL[3] > 0) { bSpectrumValueL[3] = bSpectrumValueL[3] - 1; }
      if (bSpectrumValueL[4] > 0) { bSpectrumValueL[4] = bSpectrumValueL[4] - 1; }
      if (bSpectrumValueL[5] > 0) { bSpectrumValueL[5] = bSpectrumValueL[5] - 1; }
 
    }
    
    if (MSGEQ7.getVolume(1) > MSGEQ7_STARTVOL) {
    
      bSpectrumValueR[0] = MSGEQ7.get(MSGEQ7_0, 1);
      bSpectrumValueR[1] = MSGEQ7.get(MSGEQ7_1, 1);
      bSpectrumValueR[2] = MSGEQ7.get(MSGEQ7_2, 1);
      bSpectrumValueR[3] = MSGEQ7.get(MSGEQ7_3, 1);
      bSpectrumValueR[4] = MSGEQ7.get(MSGEQ7_4, 1);
      bSpectrumValueR[5] = MSGEQ7.get(MSGEQ7_5, 1);
    
    }

    else {
    
      if (bSpectrumValueR[0] > 0) { bSpectrumValueR[0] = bSpectrumValueR[0] - 1; }
      if (bSpectrumValueR[1] > 0) { bSpectrumValueR[1] = bSpectrumValueR[1] - 1; }
      if (bSpectrumValueR[2] > 0) { bSpectrumValueR[2] = bSpectrumValueR[2] - 1; }
      if (bSpectrumValueR[3] > 0) { bSpectrumValueR[3] = bSpectrumValueR[3] - 1; }
      if (bSpectrumValueR[4] > 0) { bSpectrumValueR[4] = bSpectrumValueR[4] - 1; }
      if (bSpectrumValueR[5] > 0) { bSpectrumValueR[5] = bSpectrumValueR[5] - 1; }
    
    }
  
  }

  vShowOnDisplay(bSpectrumValueL[0], bSpectrumValueR[0], bSpectrumValueL[1], bSpectrumValueR[1],
                 bSpectrumValueL[2], bSpectrumValueR[2], bSpectrumValueL[3], bSpectrumValueR[3], 
                 bSpectrumValueL[4], bSpectrumValueR[4], bSpectrumValueL[5], bSpectrumValueR[5]);
 
  FastLED.clear();

/*  
 *  vShowLedBarSoludUV: check first which side has the maximum SpectrumValue then set the proportion between  
 *  value from the spectrum each channel from lowest to highes byte to the amount of leds in one section
 */
 
  vShowLedBarSolidUV(max(bSpectrumValueL[0], bSpectrumValueR[0]), 
                     dStartNumberLedStrip1 , dNumberLedsStrip1 / 2, iHueValue, false);
  vShowLedBarSolidUV(max(bSpectrumValueL[1], bSpectrumValueR[1]), 
                     dStartNumberLedStrip1 + dNumberLedsStrip1 / 2, dNumberLedsStrip1 / 2, iHueValue + 64, true);
  vShowLedBarSolidUV(max(bSpectrumValueL[2], bSpectrumValueR[2]), 
                     dStartNumberLedStrip2 , dNumberLedsStrip2 / 2, iHueValue + 20, true);
  vShowLedBarSolidUV(max(bSpectrumValueL[3], bSpectrumValueR[3]), 
                     dStartNumberLedStrip2 + dNumberLedsStrip2 / 2, dNumberLedsStrip2 / 2, iHueValue + 128, true);
  vShowLedBarSolidUV(max(bSpectrumValueL[4], bSpectrumValueR[4]), 
                     dStartNumberLedStrip3 , dNumberLedsStrip3 / 2, iHueValue + 32, true);
  vShowLedBarSolidUV(max(bSpectrumValueL[5], bSpectrumValueR[5]), 
                     dStartNumberLedStrip3 + dNumberLedsStrip3 / 2, dNumberLedsStrip3 / 2, iHueValue + 192, true);

//  vShowLedBarGlowUV(bSpectrumValueL[4], 0, 25, iHueValue);

  iHueValue++;
  if (iHueValue == 256) { iHueValue = 0; }

  FastLED.show();

}

void vShowLedBarSolidUV(byte bSpectrumValue, int iLedStart, int iLedCount, int iHueValue, bool bDirection) {

/*  
 * this function will show a solid bar of the UV meter 
 * just color and the amount of leds ... no more
 * bDirection means if the UV goes in positive direction e.g. 2 3 4 5 6 or negative direction e.g. 6 5 4 3 2
 */

  int iLeds = map(bSpectrumValue, 0, 255, 0, iLedCount);

  if (bDirection == true) {
    for (int i = iLedStart; i - iLedStart < iLeds; i++) {
      leds[i] = CHSV(iHueValue, 255, 255);
    }
  } else {
    for (int i = iLedStart + iLedCount; (iLedStart + iLedCount) - i < iLeds; i--) {
      leds[i] = CHSV(iHueValue, 255, 255);
    }    
  }
  
}

void vShowLedBarGlowUV(byte bSpectrumValue, int iLedStart, int iLedCount, int iHueValue, bool bDirection) {



}

void vShowOnDisplay(byte bVolValL0, byte bVolValR0, byte bVolValL1, byte bVolValR1, 
                    byte bVolValL2, byte bVolValR2, byte bVolValL3, byte bVolValR3,
                    byte bVolValL4, byte bVolValR4, byte bVolValL5, byte bVolValR5) {

/*                      
 *  this function will show the spectrum on the display
 */

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

