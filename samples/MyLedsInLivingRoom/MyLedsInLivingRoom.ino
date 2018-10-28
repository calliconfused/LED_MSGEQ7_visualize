/*
 * Calli Confuse - MyLedsInLivingRoom v0.1 - 2018
 * 
 * for more information about the complete project please visit: https://github.com/calliconfused/LED_MSGEQ7_visualize 
 * 
 * led arrangement in my living room
 * 
 *                               82 leds            82 leds
 *                         4------------------5------------------6
 *                         |                                     |
 *                         | 8 leds                              | 8 leds
 *                        /                                       \
 *                       /                                         \
 *             92 leds  /                                           \ 92 leds
 *                     /                                             \
 *                    /                                               \
 *                   3                                                 7
 *                    \                                               /
 *                     \                                             /
 *             57 leds  \                                           / 57 leds
 *                       \                                         /
 *                        \                                       /
 *                         \                                     /
 *                          2------------------1 ::: -----------8† (interface point dead, combined with 7)
 *                                 86 leds             69 leds
 * 
 */

// LED strip

#define dColorRange             RBG
#define dTypeStrip              WS2812
#define dBrightness             192

#define dDataPinBar1            25  // cable yellow I
#define dDataPinBar2            29  // cable green I
#define dDataPinBar3            33  // cable orange I
#define dDataPinBar4            37  // cable violet I
#define dDataPinBar5            41  // cable green II
#define dDataPinBar6            45  // cable yellow II
#define dDataPinBar7            49  // cable violet II
// cable orange II is damaged and is replaces together with data pin bar 7

#define dNumberLedsStrip1       86
#define dNumberLedsStrip2       57
#define dNumberLedsStrip3       92
#define dNumberLedsStrip4       8
#define dNumberLedsStrip5       82
#define dNumberLedsStrip6       82
#define dNumberLedsStrip7       8
#define dNumberLedsStrip8       92
#define dNumberLedsStrip9       57
#define dNumberLedsStrip10      69

#define dStartNumberLedStrip1   0
#define dStartNumberLedStrip2   dStartNumberLedStrip1 + dNumberLedsStrip1
#define dStartNumberLedStrip3   dStartNumberLedStrip2 + dNumberLedsStrip2
#define dStartNumberLedStrip4   dStartNumberLedStrip3 + dNumberLedsStrip3
#define dStartNumberLedStrip5   dStartNumberLedStrip4 + dNumberLedsStrip4
#define dStartNumberLedStrip6   dStartNumberLedStrip5 + dNumberLedsStrip5
#define dStartNumberLedStrip7   dStartNumberLedStrip6 + dNumberLedsStrip6
#define dStartNumberLedStrip8   dStartNumberLedStrip7 + dNumberLedsStrip7
#define dStartNumberLedStrip9   dStartNumberLedStrip8 + dNumberLedsStrip8
#define dStartNumberLedStrip10  dStartNumberLedStrip9 + dNumberLedsStrip9

#define dNumberLedsTotal dNumberLedsStrip1 + dNumberLedsStrip2 + dNumberLedsStrip3 + dNumberLedsStrip4 + dNumberLedsStrip5 + dNumberLedsStrip6 + dNumberLedsStrip7 + dNumberLedsStrip8 + dNumberLedsStrip9 + dNumberLedsStrip10

#include <FastLED.h>
CRGB leds[dNumberLedsTotal];
uint8_t iHueValue = 0;
int iPoint = 0;

// digital and analog pins for MSGEQ7

#define dAnalogPinL             0   // MSGEQ7 OUT
#define dAnalogPinR             1   // MSGEQ7 OUT
#define dStrobePin              2   // MSGEQ7 STROBE
#define dResetPin               4   // MSGEQ7 RESET
#define MSGEQ7_INTERVAL         ReadsPerSecond(50)
#define MSGEQ7_SMOOTH           40  // Range: 0-255
#define MSGEQ7_STARTVOL         40  // to adjust when the script will start to work

#include "MSGEQ7.h"
CMSGEQ7<MSGEQ7_SMOOTH, dResetPin, dStrobePin, dAnalogPinL, dAnalogPinR> MSGEQ7;

uint8_t bSpectrumValueL[6];
uint8_t bSpectrumValueR[6];

byte bVolume1 = 0;
byte bVolume2 = 0;
byte bVolume3 = 0;
byte bVolume4 = 0;
byte bVolume5 = 0;

// display properties

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2); // display will be rotate 180 degrees, otherwise cable management in housing is terrible 

#define dDisplayBarYPosition    25  // vertical position of bars
#define dDisplayBarMaxLength    25  // max length of bars
#define dDisplayBarWidth        9   // max width of bars
#define dDisplayBarWidthOffset  1   // space between bars
#define dDisplayTextYPosition   32  // lowest position of text

byte bDisplayBarXPosition;
byte bDisplayBarLength;

void setup() {
  
  Serial2.begin(115200);
  Serial.begin(115200);
  u8g2.begin();
  
  FastLED.addLeds<dTypeStrip, dDataPinBar1, dColorRange>(leds, dStartNumberLedStrip1, dNumberLedsStrip1);
  FastLED.addLeds<dTypeStrip, dDataPinBar2, dColorRange>(leds, dStartNumberLedStrip2, dNumberLedsStrip2);
  FastLED.addLeds<dTypeStrip, dDataPinBar3, dColorRange>(leds, dStartNumberLedStrip3, dNumberLedsStrip3 + dNumberLedsStrip4);
  FastLED.addLeds<dTypeStrip, dDataPinBar4, dColorRange>(leds, dStartNumberLedStrip5, dNumberLedsStrip5);
  FastLED.addLeds<dTypeStrip, dDataPinBar5, dColorRange>(leds, dStartNumberLedStrip6, dNumberLedsStrip6);
  FastLED.addLeds<dTypeStrip, dDataPinBar6, dColorRange>(leds, dStartNumberLedStrip7, dNumberLedsStrip7 + dNumberLedsStrip8);
  FastLED.addLeds<dTypeStrip, dDataPinBar7, dColorRange>(leds, dStartNumberLedStrip9, dNumberLedsStrip9 + dNumberLedsStrip10);
        
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
 
//  vShowLedBarGlowUV(max(bSpectrumValueL[0], bSpectrumValueR[0]), 
//                    dStartNumberLedStrip1 , dNumberLedsStrip1 / 2, iHueValue, true);


  vShowLedBarSolidUV(bSpectrumValueL[0], dStartNumberLedStrip1, dNumberLedsStrip1, iHueValue, true);
  vShowLedBarSolidUV(bSpectrumValueL[1], dStartNumberLedStrip2, dNumberLedsStrip2, iHueValue, false);
  vShowLedBarSolidUV(bSpectrumValueL[2], dStartNumberLedStrip3, dNumberLedsStrip3, iHueValue, true);
  vShowLedBarSolidUV(bSpectrumValueL[3], dStartNumberLedStrip5, dNumberLedsStrip5, iHueValue, false);

  vShowLedBarSolidUV(bSpectrumValueR[3], dStartNumberLedStrip6, dNumberLedsStrip6, iHueValue, true);
  vShowLedBarSolidUV(bSpectrumValueR[2], dStartNumberLedStrip8, dNumberLedsStrip8, iHueValue, false);
  vShowLedBarSolidUV(bSpectrumValueR[1], dStartNumberLedStrip9, dNumberLedsStrip9, iHueValue, true);
  vShowLedBarSolidUV(bSpectrumValueR[0], dStartNumberLedStrip10, dNumberLedsStrip10, iHueValue, false);


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
