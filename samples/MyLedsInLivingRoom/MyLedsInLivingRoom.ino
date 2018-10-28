/*
 * Calli Confused - MyLedsInLivingRoom v0.1 - 2018
 * 
 * for more information about the complete project please visit: https://github.com/calliconfused/LED_MSGEQ7_visualize 
 * 
 * led arrangement in my living room
 * 
 *                               82 leds            82 leds
 *                         D------------------E------------------F
 *                         |                                     |
 *                         | 7 leds                              | 8 leds
 *                        /                                       \
 *                       /                                         \
 *             92 leds  /                                           \ 92 leds
 *                     /                                             \
 *                    /                                               \
 *                   C                                                 G
 *                    \                                               /
 *                     \                                             /
 *             57 leds  \                                           / 57 leds
 *                       \                                         /
 *                        \                                       /
 *                         \                                     /
 *                          B------------------A ::: -----------H† (interface point dead, combined with G)
 *                                 86 leds             69 leds
 * 
 */

#include "Arduino.h"

// LED strip

#include "FastLED.h"
#define dColorOrder             GRB
#define dTypeStrip              WS2812
#define dBrightness             192

#define dDataPinBarA            25  // cable yellow I
#define dDataPinBarB            29  // cable green I
#define dDataPinBarC            33  // cable orange I
#define dDataPinBarD            37  // cable violet I
#define dDataPinBarE            41  // cable green II
#define dDataPinBarF            45  // cable yellow II
#define dDataPinBarG            49  // cable violet II
// cable orange II is damaged and is replaces together with data pin bar G

#define dNumberLedsStripA       86
#define dNumberLedsStripB       57
#define dNumberLedsStripC       92
#define dNumberLedsStripD       7
#define dNumberLedsStripE       82
#define dNumberLedsStripF       82
#define dNumberLedsStripG       8
#define dNumberLedsStripH       92
#define dNumberLedsStripI       57
#define dNumberLedsStripJ       69

#define dStartNumberLedStripA   0
#define dStartNumberLedStripB   dStartNumberLedStripA + dNumberLedsStripA
#define dStartNumberLedStripC   dStartNumberLedStripB + dNumberLedsStripB
#define dStartNumberLedStripD   dStartNumberLedStripC + dNumberLedsStripC
#define dStartNumberLedStripE   dStartNumberLedStripD + dNumberLedsStripD
#define dStartNumberLedStripF   dStartNumberLedStripE + dNumberLedsStripE
#define dStartNumberLedStripG   dStartNumberLedStripF + dNumberLedsStripF
#define dStartNumberLedStripH   dStartNumberLedStripG + dNumberLedsStripG
#define dStartNumberLedStripI   dStartNumberLedStripH + dNumberLedsStripH
#define dStartNumberLedStripJ   dStartNumberLedStripI + dNumberLedsStripI

#define dNumberLedsTotal dNumberLedsStripA + dNumberLedsStripB + dNumberLedsStripC + dNumberLedsStripD + dNumberLedsStripE + dNumberLedsStripF + dNumberLedsStripG + dNumberLedsStripH + dNumberLedsStripI + dNumberLedsStripJ

byte bHueTimer = 0;

CRGB leds[dNumberLedsTotal];
uint8_t iHueValue = 0;

// digital and analog pins for MSGEQ7

#include "MSGEQ7.h"

#define dAnalogPinL             0   // MSGEQ7 OUT
#define dAnalogPinR             1   // MSGEQ7 OUT
#define dStrobePin              2   // MSGEQ7 STROBE
#define dResetPin               4   // MSGEQ7 RESET
#define MSGEQ7_INTERVAL         ReadsPerSecond(50)
#define MSGEQ7_SMOOTH           40  // Range: 0-255
#define MSGEQ7_STARTVOL         40  // to adjust when the script will start to work

CMSGEQ7<MSGEQ7_SMOOTH, dResetPin, dStrobePin, dAnalogPinL, dAnalogPinR> MSGEQ7;

uint8_t bSpectrumValueL[6];
uint8_t bSpectrumValueR[6];

byte bVolumeL;
byte bVolumeR;

// display properties

#include "U8g2lib.h"

#ifdef U8X8_HAVE_HW_SPI
#include "SPI.h"
#endif

#ifdef U8X8_HAVE_HW_I2C
#include "Wire.h"
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
  
//  Serial2.begin(115200);
  Serial.begin(115200);
  u8g2.begin();
  
  FastLED.addLeds<dTypeStrip, dDataPinBarA, dColorOrder>(leds, dStartNumberLedStripA, dNumberLedsStripA);
  FastLED.addLeds<dTypeStrip, dDataPinBarB, dColorOrder>(leds, dStartNumberLedStripB, dNumberLedsStripB);
  FastLED.addLeds<dTypeStrip, dDataPinBarC, dColorOrder>(leds, dStartNumberLedStripC, dNumberLedsStripC + dNumberLedsStripD);
  FastLED.addLeds<dTypeStrip, dDataPinBarD, dColorOrder>(leds, dStartNumberLedStripE, dNumberLedsStripE);
  FastLED.addLeds<dTypeStrip, dDataPinBarE, dColorOrder>(leds, dStartNumberLedStripF, dNumberLedsStripF);
  FastLED.addLeds<dTypeStrip, dDataPinBarF, dColorOrder>(leds, dStartNumberLedStripG, dNumberLedsStripG + dNumberLedsStripH);
  FastLED.addLeds<dTypeStrip, dDataPinBarG, dColorOrder>(leds, dStartNumberLedStripI, dNumberLedsStripI + dNumberLedsStripJ);
        
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

    bVolumeL = MSGEQ7.getVolume(0);
    bVolumeR = MSGEQ7.getVolume(1);

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

  // Display function for all values currently off because of performance problems
//  vShowOnDisplayUV(bSpectrumValueL[0], bSpectrumValueR[0], bSpectrumValueL[1], bSpectrumValueR[1],
//                   bSpectrumValueL[2], bSpectrumValueR[2], bSpectrumValueL[3], bSpectrumValueR[3], 
//                   bSpectrumValueL[4], bSpectrumValueR[4], bSpectrumValueL[5], bSpectrumValueR[5]);
  
  vShowOnDisplayVol(bVolumeL, bVolumeR);
 
  FastLED.clear();
  
  vShowLedBarGlowUV(bSpectrumValueL[1], dStartNumberLedStripA, dNumberLedsStripA, iHueValue, true);
  vShowLedBarGlowUV(bSpectrumValueL[2], dStartNumberLedStripB, dNumberLedsStripB, iHueValue, false);
  vShowLedBarGlowUV(bSpectrumValueL[3], dStartNumberLedStripC, dNumberLedsStripC, iHueValue, true);
  vShowLedBarFullUV(bSpectrumValueL[0], dStartNumberLedStripD, dNumberLedsStripD, iHueValue);
  vShowLedBarGlowUV(bSpectrumValueL[4], dStartNumberLedStripE, dNumberLedsStripE, iHueValue, false);

  vShowLedBarGlowUV(bSpectrumValueR[4], dStartNumberLedStripF, dNumberLedsStripF, iHueValue, true);
  vShowLedBarFullUV(bSpectrumValueL[0], dStartNumberLedStripG, dNumberLedsStripG, iHueValue);
  vShowLedBarGlowUV(bSpectrumValueR[3], dStartNumberLedStripH, dNumberLedsStripH, iHueValue, false);
  vShowLedBarGlowUV(bSpectrumValueR[2], dStartNumberLedStripI, dNumberLedsStripI, iHueValue, true);
  vShowLedBarGlowUV(bSpectrumValueR[1], dStartNumberLedStripJ, dNumberLedsStripJ, iHueValue, false);

  if (bHueTimer >= 3) { 
    iHueValue++;
    bHueTimer = 0; 
  } else {
    bHueTimer++;
  }
  
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

  int iLeds = map(bSpectrumValue, 0, 255, 0, iLedCount); 
  byte bVal = 255;
    
  if (bDirection == true) {
    for (int i = iLedStart; i - iLedStart < iLeds; i++) {
      if ( i >= iLedStart + iLeds - iLimitDark) {
        bVal = bVal - iLimitDarkSteps;
      }
      leds[i] = CHSV(iHueValue, 255, bVal);
    }
  } else {
    for (int i = iLedStart + iLedCount; (iLedStart + iLedCount) - i < iLeds; i--) {
      if ( i <= (iLedStart + iLedCount) - iLeds + iLimitDark) {
        bVal = bVal - iLimitDarkSteps;
      }
      leds[i] = CHSV(iHueValue, 255, bVal);
    }
  }

}

void vShowLedBarFullUV(byte bSpectrumValue, int iLedStart, int iLedCount, int iHueValue) {

    for (int i = iLedStart; i - iLedStart < iLedCount; i++) {
      leds[i] = CHSV(iHueValue, 255, bSpectrumValue);
    }
  
}

void vShowOnDisplayVol(byte bVolumeL, byte bVolumeR) {

/*                      
 *  this function will show the spectrum on the display
 */

  u8g2.setFont(u8g2_font_4x6_mf);
  u8g2.firstPage();
  
  do {

    bDisplayBarXPosition = 1;
    bDisplayBarLength = map(bVolumeL, 0, 255, 0, 60);
    u8g2.drawBox(bDisplayBarXPosition + 60 - bDisplayBarLength, 2, bDisplayBarLength, 20);
    u8g2.drawStr(bDisplayBarXPosition + 24, dDisplayTextYPosition, "LEFT");

    bDisplayBarXPosition = 65;
    bDisplayBarLength = map(bVolumeR, 0, 255, 0, 60);
    u8g2.drawBox(bDisplayBarXPosition, 2, bDisplayBarLength, 20); 
    u8g2.drawStr(bDisplayBarXPosition + 21, dDisplayTextYPosition, "RIGHT");
    
  } while ( u8g2.nextPage() );
  
}

void vShowOnDisplayUV(byte bVolValL0, byte bVolValR0, byte bVolValL1, byte bVolValR1, 
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
    u8g2.drawStr(bDisplayBarXPosition + 1, dDisplayTextYPosition, "16");

    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL1, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.drawStr(bDisplayBarXPosition + 1, dDisplayTextYPosition, "40");
    
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL2, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.drawStr(bDisplayBarXPosition + 0, dDisplayTextYPosition, "1k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL3, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.drawStr(bDisplayBarXPosition + 0, dDisplayTextYPosition, "2k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL4, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.drawStr(bDisplayBarXPosition - 1, dDisplayTextYPosition, "6k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL5, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength); 
    u8g2.drawStr(bDisplayBarXPosition - 2, dDisplayTextYPosition, "16k");
        
    bDisplayBarXPosition = 64;
    bDisplayBarLength = map(bVolValR5, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.drawStr(bDisplayBarXPosition - 2, dDisplayTextYPosition, "16k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR4, 0, 255, 0, dDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.drawStr(bDisplayBarXPosition + 1, dDisplayTextYPosition, "6k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR3, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.drawStr(bDisplayBarXPosition + 1, dDisplayTextYPosition, "2k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR2, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.drawStr(bDisplayBarXPosition + 0, dDisplayTextYPosition, "1k");

    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR1, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.drawStr(bDisplayBarXPosition + 0, dDisplayTextYPosition, "40");
        
    bDisplayBarXPosition = bDisplayBarXPosition + dDisplayBarWidth + dDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR0, 0, 255, 0, dDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, dDisplayBarYPosition - bDisplayBarLength, dDisplayBarWidth, bDisplayBarLength);
    u8g2.drawStr(bDisplayBarXPosition + 1, dDisplayTextYPosition, "16");
    
  } while ( u8g2.nextPage() );
  
}
