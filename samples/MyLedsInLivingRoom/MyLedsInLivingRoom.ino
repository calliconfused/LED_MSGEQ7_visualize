/*
 * define all variables to adjust the controller
 */

// digital and analog pins for MSGEQ7

#define dAnalogPinL             0   // MSGEQ7 OUT
#define dAnalogPinR             1   // MSGEQ7 OUT
#define dStrobePin              2   // MSGEQ7 STROBE
#define dResetPin               4   // MSGEQ7 RESET
#define MSGEQ7_INTERVAL         ReadsPerSecond(50)
#define MSGEQ7_SMOOTH           40  // Range: 0-255
#define MSGEQ7_STARTVOL         40  // to adjust when the script will start to work

// LED strip

#define dColorRange             RBG
#define dTypeStrip              WS2812
#define dBrightness             80 

#define dNumberLedsStrip1       150
#define dDataPinStrip1          25
#define dNumberLedsStrip2       150
#define dDataPinStrip2          29
#define dNumberLedsStrip3       150
#define dDataPinStrip3          33
#define dNumberLedsStrip4       150
#define dDataPinStrip4          37
#define dNumberLedsStrip5       150
#define dDataPinStrip6          41

#define dStartNumberLedStrip1   0
#define dStartNumberLedStrip2   dStartNumberLedStrip1 + dNumberLedsStrip1
#define dStartNumberLedStrip3   dStartNumberLedStrip2 + dNumberLedsStrip2
#define dStartNumberLedStrip4   dStartNumberLedStrip3 + dNumberLedsStrip3
#define dStartNumberLedStrip5   dStartNumberLedStrip4 + dNumberLedsStrip4

#define dNumberLedsTotal dNumberLedsStrip1 + dNumberLedsStrip2 + dNumberLedsStrip3 + dNumberLedsStrip4 + dNumberLedsStrip5

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

/* 
 *  additional variables
 */

byte bVolume1 = 0;
byte bVolume2 = 0;
byte bVolume3 = 0;
byte bVolume4 = 0;
byte bVolume5 = 0;

void setup() {
  
  Serial2.begin(115200);
  Serial.begin(115200);
  u8g2.begin();
  
  FastLED.addLeds<dTypeStrip, dDataPinStrip1, dColorRange>(leds, dStartNumberLedStrip1, dNumberLedsStrip1);
  FastLED.addLeds<dTypeStrip, dDataPinStrip2, dColorRange>(leds, dStartNumberLedStrip2, dNumberLedsStrip2);
  FastLED.addLeds<dTypeStrip, dDataPinStrip3, dColorRange>(leds, dStartNumberLedStrip3, dNumberLedsStrip3);
  FastLED.addLeds<dTypeStrip, dDataPinStrip3, dColorRange>(leds, dStartNumberLedStrip4, dNumberLedsStrip4);
  FastLED.addLeds<dTypeStrip, dDataPinStrip3, dColorRange>(leds, dStartNumberLedStrip5, dNumberLedsStrip5);
  
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

      bSpectrumValueL[0] = MSGEQ7.get(MSGEQ7_1, 0);
      bSpectrumValueL[1] = MSGEQ7.get(MSGEQ7_2, 0);
      bSpectrumValueL[2] = MSGEQ7.get(MSGEQ7_3, 0);
      bSpectrumValueL[3] = MSGEQ7.get(MSGEQ7_4, 0);
      bSpectrumValueL[4] = MSGEQ7.get(MSGEQ7_5, 0);
      bSpectrumValueL[5] = MSGEQ7.get(MSGEQ7_6, 0);
      
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
    
      bSpectrumValueR[0] = MSGEQ7.get(MSGEQ7_1, 1);
      bSpectrumValueR[1] = MSGEQ7.get(MSGEQ7_2, 1);
      bSpectrumValueR[2] = MSGEQ7.get(MSGEQ7_3, 1);
      bSpectrumValueR[3] = MSGEQ7.get(MSGEQ7_4, 1);
      bSpectrumValueR[4] = MSGEQ7.get(MSGEQ7_5, 1);
      bSpectrumValueR[5] = MSGEQ7.get(MSGEQ7_6, 1);
    
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
 *   check if there is a peak between 5 frames, then change the color
 */

  byte bHueChangeCount = 0;
  byte bVolumePeak = 255 * 0.2;
  
  bVolume5 = bVolume4;
  bVolume4 = bVolume3;
  bVolume3 = bVolume2;
  bVolume2 = bVolume1;
  bVolume1 = max(bSpectrumValueL[0], bSpectrumValueR[0]);

  if (bVolume1 > bVolumePeak) { 
    bHueChangeCount++;
  }
  if (bVolume2 > bVolumePeak) { 
    bHueChangeCount++;
  }
  if (bVolume3 > bVolumePeak) { 
    bHueChangeCount++; 
  }
  if (bVolume4 > bVolumePeak) { 
    bHueChangeCount++;
  }
  if (bVolume5 > bVolumePeak) { 
    bHueChangeCount++; 
  }

  if ( bHueChangeCount == 1 ) { 
    iHueValue = iHueValue + 16; 
  }

/*  
 *  vShowLedBarSoludUV: check first which side has the maximum SpectrumValue then set the proportion between  
 *  value from the spectrum each channel from lowest to highes byte to the amount of leds in one section
 */

//  vShowLedBarSolidUV(max(bSpectrumValueL[1], bSpectrumValueR[1]), 
//                     dStartNumberLedStrip1 + dNumberLedsStrip1 / 2, dNumberLedsStrip1 / 2, iHueValue + 96, false); 
 
  vShowLedBarGlowUV(max(bSpectrumValueL[0], bSpectrumValueR[0]), 
                    dStartNumberLedStrip1 , dNumberLedsStrip1 / 2, iHueValue, true);
  vShowLedBarGlowUV(max(bSpectrumValueL[1], bSpectrumValueR[1]), 
                     dStartNumberLedStrip1 + dNumberLedsStrip1 / 2, dNumberLedsStrip1 / 2, iHueValue + 96, true);
  vShowLedBarGlowUV(max(bSpectrumValueL[2], bSpectrumValueR[2]), 
                    dStartNumberLedStrip2 , dNumberLedsStrip2 / 2, iHueValue + 64, true);
  vShowLedBarGlowUV(max(bSpectrumValueL[3], bSpectrumValueR[3]), 
                     dStartNumberLedStrip2 + dNumberLedsStrip2 / 2, dNumberLedsStrip2 / 2, iHueValue + 128, true);
  vShowLedBarGlowUV(max(bSpectrumValueL[4], bSpectrumValueR[4]), 
                    dStartNumberLedStrip3 , dNumberLedsStrip3 / 2, iHueValue + 32, true);
  vShowLedBarGlowUV(max(bSpectrumValueL[5], bSpectrumValueR[5]), 
                     dStartNumberLedStrip3 + dNumberLedsStrip3 / 2, dNumberLedsStrip3 / 2, iHueValue + 168, true);
  vShowLedBarGlowUV(max(bSpectrumValueL[2], bSpectrumValueR[2]), 
                    dStartNumberLedStrip4 , dNumberLedsStrip4 / 2, iHueValue + 64, true);
  vShowLedBarGlowUV(max(bSpectrumValueL[3], bSpectrumValueR[3]), 
                     dStartNumberLedStrip4 + dNumberLedsStrip4 / 2, dNumberLedsStrip4 / 2, iHueValue + 128, true);
  vShowLedBarGlowUV(max(bSpectrumValueL[4], bSpectrumValueR[4]), 
                    dStartNumberLedStrip5 , dNumberLedsStrip5 / 2, iHueValue + 32, true);
  vShowLedBarGlowUV(max(bSpectrumValueL[5], bSpectrumValueR[5]), 
                     dStartNumberLedStrip5 + dNumberLedsStrip5 / 2, dNumberLedsStrip5 / 2, iHueValue + 168, true);

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
  
  int iLimitDark = iLedCount * 0.1;
  int iLimitDarkSteps = 255 / iLimitDark;
  int iLimitLight = iLedCount * 0.8;
  int iLimitLightSteps = 255 / (iLedCount - iLimitLight);

  int iLeds = map(bSpectrumValue, 0, 255, 0, iLedCount);
  
  byte bSat = 255;
  byte bVal = 255; 

  if ( iLeds >= iLimitLight) {
    bSat = bSat - iLimitLightSteps * ( iLeds - iLimitLight);
  }  

  if (bDirection == true) {
    for (int i = iLedStart; i - iLedStart < iLeds; i++) {
      if ( i >= iLedStart + iLeds - iLimitDark) {
        bVal = bVal - iLimitDarkSteps;
      }
      if (bSat < 255) {
        bSat = bSat + iLimitLightSteps;
      }
      leds[i] = CHSV(iHueValue, bSat, bVal);
    }
  } else {

  }

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
    u8g2.setCursor(bDisplayBarXPosition + 1, dDisplayTextYPosition);
    u8g2.print("16");

    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL1, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition + 1, dDisplayTextYPosition);
    u8g2.print("40");
    
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL2, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition + 0, dDisplayTextYPosition);
    u8g2.print("1k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL3, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition + 0, dDisplayTextYPosition);
    u8g2.print("2k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL4, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition - 1, dDisplayTextYPosition);
    u8g2.print("6k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL5, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition - 2, dDisplayTextYPosition);
    u8g2.print("16k");
        
    bDisplayBarXPosition = 64;
    bDisplayBarLength = map(bVolValR5, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition - 2, dDisplayTextYPosition);
    u8g2.print("16k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR4, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 1, dDisplayTextYPosition);
    u8g2.print("6k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR3, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 1, dDisplayTextYPosition);
    u8g2.print("2k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR2, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 0, dDisplayTextYPosition);
    u8g2.print("1k");

    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR1, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 0, dDisplayTextYPosition);
    u8g2.print("40");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR0, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 1, dDisplayTextYPosition);
    u8g2.print("16");
    
  } while ( u8g2.nextPage() );
  
}

