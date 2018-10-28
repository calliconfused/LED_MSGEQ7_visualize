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
 *                         | 8 leds                              | 8 leds
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

// LED strip

#define dColorOrder             RGB
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
#define dNumberLedsStripD       8
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

#include <FastLED.h>
CRGB leds[dNumberLedsTotal];
uint8_t iHueValue = 0;
int iPoint = 0;

CRGBPalette16 currentPalette(PartyColors_p);

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

//  display is currently off cause of performance

//  vShowOnDisplay(bSpectrumValueL[0], bSpectrumValueR[0], bSpectrumValueL[1], bSpectrumValueR[1],
//                 bSpectrumValueL[2], bSpectrumValueR[2], bSpectrumValueL[3], bSpectrumValueR[3], 
//                 bSpectrumValueL[4], bSpectrumValueR[4], bSpectrumValueL[5], bSpectrumValueR[5]);
 
  FastLED.clear();
  
  vShowLedBarSolidUV(bSpectrumValueL[1], dStartNumberLedStripA, dNumberLedsStripA, iHueValue, true);
  vShowLedBarSolidUV(bSpectrumValueL[2], dStartNumberLedStripB, dNumberLedsStripB, iHueValue, false);
  vShowLedBarSolidUV(bSpectrumValueL[3], dStartNumberLedStripC, dNumberLedsStripC, iHueValue, true);
  vShowLedBarFullUV(bSpectrumValueL[0], dStartNumberLedStripD, dNumberLedsStripD, iHueValue);
  vShowLedBarFire(bSpectrumValueL[4], dStartNumberLedStripE, dNumberLedsStripE, false);

  vShowLedBarSolidUV(bSpectrumValueR[4], dStartNumberLedStripF, dNumberLedsStripF, iHueValue, true);
  vShowLedBarFullUV(bSpectrumValueL[0], dStartNumberLedStripG, dNumberLedsStripG, iHueValue);
  vShowLedBarSolidUV(bSpectrumValueR[3], dStartNumberLedStripH, dNumberLedsStripH, iHueValue, false);
  vShowLedBarSolidUV(bSpectrumValueR[2], dStartNumberLedStripI, dNumberLedsStripI, iHueValue, true);
  vShowLedBarSolidUV(bSpectrumValueR[1], dStartNumberLedStripJ, dNumberLedsStripJ, iHueValue, false);

  iHueValue++;
  if (iHueValue == 256) { iHueValue = 0; }

  FastLED.show();

  Serial.println(iHueValue);

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

    }
  }

}

void vShowLedBarFire(byte bSpectrumValue, int iLedStart, int iLedCount, bool bDirection) {
  
  #define xscale 20       // How far apart they are
  #define yscale 3        // How fast they move
  uint8_t index = 0;      // Current colour lookup value. 
  uint16_t sampleavg = 0;

  /* 
   *  Fire palette definition. Lower value = darker.
   *  Problem: my RGB order has an offset of 96 and it's not possible to change the order in the setup.
   *  Otherwise it won't upload to the Mega 2560. Verifying is okay, but upload not.
   *  This the old CRGBPalette16 Code:
   *  currentPalette = CRGBPalette16(CHSV(0,255,2), CHSV(0,255,4), CHSV(0,255,8), CHSV(0, 255, 8),
                                 CHSV(0, 255, 16), CRGB::Red, CRGB::Red, CRGB::Red,                                   
                                 CRGB::DarkOrange,CRGB::DarkOrange, CRGB::Orange, CRGB::Orange,
                                 CRGB::Yellow, CRGB::Orange, CRGB::Yellow, CRGB::Yellow);
   */
  
  currentPalette = CRGBPalette16(CHSV(  0+160, 255,   2), 
                                 CHSV(  0+160, 255,   4),  
                                 CHSV(  0+160, 255,   8), 
                                 CHSV(  0+160, 255,   8),
                                 
                                 CHSV(  0+160, 255,  16), 
                                 CHSV(  0+160, 255, 255), //CRGB::Red 
                                 CHSV(  0+160, 255, 255), //CRGB::Red
                                 CHSV(  0+160, 255, 255), //CRGB::Red

                                 CHSV( 33+160, 255, 255), //CRGB::DarkOrange
                                 CHSV( 33+160, 255, 255), //CRGB::DarkOrange
                                 CHSV( 39+160, 255, 255), //CRGB::Orange
                                 CHSV( 39+160, 255, 255), //CRGB::Orange
                                 
                                 CHSV( 60+140, 255, 255), //CRGB::Yellow
                                 CHSV( 39+160, 255, 255), //CRGB::Orange
                                 CHSV( 60+140, 255, 255), //CRGB::Yellow
                                 CHSV( 60+140, 255, 255));//CRGB::Yellow

  int iLeds = map(bSpectrumValue, 0, 255, 0, iLedCount);

  if (bDirection == true) {
    for (int i = iLedStart; i - iLedStart < iLeds; i++) {
      index = inoise8(i*xscale,millis()*yscale*iLeds/255);                      // X location is constant, but we move along the Y at the rate of millis(). By Andrew Tuline.
  
      index = (255 - i*256/iLeds) * index/128;                                  // Now we need to scale index so that it gets blacker as we get close to one of the ends
                                                                                // This is a simple y=mx+b equation that's been scaled. index/128 is another scaling.
      leds[i] = ColorFromPalette(currentPalette, index, iLeds, NOBLEND);        // With that value, look up the 8 bit colour palette value and assign it to the current LED. 
    }
  } else {
    for (int i = iLedStart + iLedCount; (iLedStart + iLedCount) - i < iLeds; i--) {
      index = inoise8(i*xscale,millis()*yscale*iLeds/255);                      // X location is constant, but we move along the Y at the rate of millis(). By Andrew Tuline.
  
      index = (255 - i*256/iLeds) * index/128;                                  // Now we need to scale index so that it gets blacker as we get close to one of the ends
                                                                                // This is a simple y=mx+b equation that's been scaled. index/128 is another scaling.
      leds[i] = ColorFromPalette(currentPalette, index, iLeds, NOBLEND);        // With that value, look up the 8 bit colour palette value and assign it to the current LED. 
    }    
  }

}

void vShowLedBarFullUV(byte bSpectrumValue, int iLedStart, int iLedCount, int iHueValue) {

    for (int i = iLedStart; i - iLedStart < iLedCount; i++) {
      leds[i] = CHSV(iHueValue, 255, bSpectrumValue);
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
