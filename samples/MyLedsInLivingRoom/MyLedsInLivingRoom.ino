/*
 * define all variables to adjust the controller
 */

// digital and analog pins for MSGEQ7

#define dAnalogPinL             0   // MSGEQ7 OUT
#define dAnalogPinR             1   // MSGEQ7 OUT
#define dStrobePin              2   // MSGEQ7 STROBE
#define dResetPin               4   // MSGEQ7 RESET
#define filterValue             80

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

byte bSpectrumAmount = 6;
byte bSpectrumValueL[6];
byte bSpectrumValueR[6];

/*
 * FastLED
 */

#include <FastLED.h>
CRGB leds[dNumberLedsTotal];
uint8_t iHueValue = 0;

void setup()
{
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
  
  pinMode(dAnalogPinL, INPUT);
  pinMode(dAnalogPinR, INPUT);
  pinMode(dStrobePin, OUTPUT);
  pinMode(dResetPin, OUTPUT);
  analogReference(DEFAULT); // 5V
  digitalWrite(dResetPin, LOW);
  digitalWrite(dStrobePin, HIGH);
}
 
void loop()
{
  // Set reset pin low to enable strobe
  digitalWrite(dResetPin, HIGH);
  digitalWrite(dResetPin, LOW);
 
  // Get just 6 spectrum values from the MSGEQ7 because 16 kHz isn't so much higher
  for (int i = 0; i < bSpectrumAmount; i++)
  {
    digitalWrite(dStrobePin, LOW);
    delayMicroseconds(30); // Allow output to settle
 
    bSpectrumValueL[i] = map(constrain(analogRead(dAnalogPinL), filterValue, 1023), filterValue, 1023, 0, 255);
    bSpectrumValueR[i] = map(constrain(analogRead(dAnalogPinR), filterValue, 1023), filterValue, 1023, 0, 255);
    
    // just for debug mode    
    Serial.print(bSpectrumValueL[i]);
    Serial.print(" ");
    Serial.print(bSpectrumValueR[i]);
    Serial.print(" ");    
    
    digitalWrite(dStrobePin, HIGH);
  }

  Serial.println();

  vShowOnDisplay(bSpectrumValueL[0], bSpectrumValueR[0], bSpectrumValueL[1], bSpectrumValueR[1],
                 bSpectrumValueL[2], bSpectrumValueR[2], bSpectrumValueL[3], bSpectrumValueR[3], 
                 bSpectrumValueL[4], bSpectrumValueR[4], bSpectrumValueL[5], bSpectrumValueR[5]);

  FastLED.clear();

  vShowLedBar(map(max(bSpectrumValueL[1], bSpectrumValueR[1]), 0, 255, 0, dNumberLedsStrip1), dStartNumberLedStrip1, iHueValue);
  vShowLedBar(map(max(bSpectrumValueL[3], bSpectrumValueR[3]), 0, 255, 0, dNumberLedsStrip2), dStartNumberLedStrip2, iHueValue);
  
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

