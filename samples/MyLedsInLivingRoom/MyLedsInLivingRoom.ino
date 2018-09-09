/* 
 *  activate u8g2
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

byte bDisplayBarYPosition = 25;
byte bDisplayBarMaxLength = 25;
byte bDisplayBarWidth = 9;
byte bDisplayBarWidthOffset = 1;
byte bDisplayBarXPosition;
byte bDisplayBarLength;
byte bDisplayTextYPosition = 32;

/*
 * aktiviere MSGEQ7
 */

byte analogPinL = 0; // MSGEQ7 OUT
byte analogPinR = 1; // MSGEQ7 OUT
byte strobePin = 2; // MSGEQ7 STROBE
byte resetPin = 4; // MSGEQ7 RESET
byte spectrumValueL[7];
int spectrumValueR[7];
 
byte filterValue = 80;

/*
 * aktiviere FastLED
 */

#include <FastLED.h>

#define NUM_LEDS      50
#define DATA_PIN_L      23
#define DATA_PIN_R      25
//#define CLOCK_PIN     4
#define COLORARRANGE  RBG
#define LED_STRIP     WS2812
#define BRIGHTNESS    50 
CRGB LED_X[NUM_LEDS];
#define SPECTRUM_EQ7  7
#define SPECTRUM_SEC  7
#define SECTION_LEN   (NUM_LEDS / SPECTRUM_SEC)

int LED_LS[SPECTRUM_SEC];
int LED_LE[SPECTRUM_SEC];
int LED_SECTION[SPECTRUM_SEC];
uint8_t HUE_VAL = 0;

void setup()
{
  Serial2.begin(115200);
  Serial.begin(9600);
  u8g2.begin();

  //FastLED.addLeds<LED_STRIP, DATA_PIN_L, CLOCK_PIN, COLORARRANGE>(LED_X, NUM_LEDS);
  FastLED.addLeds<LED_STRIP, DATA_PIN_L, COLORARRANGE>(LED_X, NUM_LEDS);
  FastLED.addLeds<LED_STRIP, DATA_PIN_R, COLORARRANGE>(LED_X, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  for (int i = 0; i < NUM_LEDS; i++) {
    LED_X[i] = CRGB(0, 0, 0);
  }
  FastLED.show();    
  
  // Read from MSGEQ7 OUT
  pinMode(analogPinL, INPUT);
  pinMode(analogPinR, INPUT);
  // Write to MSGEQ7 STROBE and RESET
  pinMode(strobePin, OUTPUT);
  pinMode(resetPin, OUTPUT);
 
  // Set analogPin's reference voltage
  analogReference(DEFAULT); // 5V
 
  // Set startup values for pins
  digitalWrite(resetPin, LOW);
  digitalWrite(strobePin, HIGH);
}
 
void loop()
{
  // Set reset pin low to enable strobe
  digitalWrite(resetPin, HIGH);
  digitalWrite(resetPin, LOW);
 
  // Get all 7 spectrum values from the MSGEQ7
  for (int i = 0; i < 7; i++)
  {
    digitalWrite(strobePin, LOW);
    delayMicroseconds(30); // Allow output to settle
 
    spectrumValueL[i] = analogRead(analogPinL);
    spectrumValueR[i] = analogRead(analogPinR);
     
    // Constrain any value above 1023 or below filterValue
    spectrumValueL[i] = constrain(spectrumValueL[i], filterValue, 1023);
    spectrumValueR[i] = constrain(spectrumValueR[i], filterValue, 1023);
 
    // Remap the value to a number between 0 and 255
    spectrumValueL[i] = map(spectrumValueL[i], filterValue, 1023, 0, 255);
    spectrumValueR[i] = map(spectrumValueR[i], filterValue, 1023, 0, 255);
    
    digitalWrite(strobePin, HIGH);
  }

  vShowOnDisplay(spectrumValueL[0], spectrumValueR[0], spectrumValueL[1], spectrumValueR[1],
                 spectrumValueL[2], spectrumValueR[2], spectrumValueL[3], spectrumValueR[3], 
                 spectrumValueL[4], spectrumValueR[4], spectrumValueL[5], spectrumValueR[5], 
                 spectrumValueL[6], spectrumValueR[6]);

  FastLED.clear();

  for (int i = 0; i < SPECTRUM_SEC; i++) {
   
    LED_SECTION[i] = map((spectrumValueL[i] + spectrumValueR[i])/2, 0, 255, 0, 4);

    if (i==1) {
      LED_LS[i] = 0;
    }
    else {
      LED_LS[i] = SECTION_LEN + LED_LS[i-1];
    }
    
    for (int j = LED_LS[i]; j < LED_SECTION[i] + LED_LS[i]; j++) {
      LED_X[j] = CHSV(HUE_VAL, 255, 192);
    }
  }

  HUE_VAL++;

  if (HUE_VAL==256) {
    HUE_VAL = 0;
  }

  FastLED.show();

}

void vShowOnDisplay(byte bVolValL0, byte bVolValR0, byte bVolValL1, byte bVolValR1, 
                    byte bVolValL2, byte bVolValR2, byte bVolValL3, byte bVolValR3,
                    byte bVolValL4, byte bVolValR4, byte bVolValL5, byte bVolValR5,
                    byte bVolValL6, byte bVolValR6) {

  u8g2.setFont(u8g2_font_4x6_mf);
  u8g2.firstPage();
  
  do {

    bDisplayBarXPosition = 1;
    bDisplayBarLength = map(bVolValL0, 0, 255, 0, bDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 0, bDisplayTextYPosition);
    u8g2.print("63");

    bDisplayBarXPosition = bDisplayBarXPosition + bDisplayBarWidth + bDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL1, 0, 255, 0, bDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition - 2, bDisplayTextYPosition);
    u8g2.print("160");
    
    bDisplayBarXPosition = bDisplayBarXPosition + bDisplayBarWidth + bDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL2, 0, 255, 0, bDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition + 0, bDisplayTextYPosition);
    u8g2.print("400");
        
    bDisplayBarXPosition = bDisplayBarXPosition + bDisplayBarWidth + bDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL3, 0, 255, 0, bDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition + 2, bDisplayTextYPosition);
    u8g2.print("1k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + bDisplayBarWidth + bDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL4, 0, 255, 0, bDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition + 1, bDisplayTextYPosition);
    u8g2.print("2k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + bDisplayBarWidth + bDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValL5, 0, 255, 0, bDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength); 
    u8g2.setCursor(bDisplayBarXPosition + 1, bDisplayTextYPosition);
    u8g2.print("6k");
        
    bDisplayBarXPosition = 64;
    bDisplayBarLength = map(bVolValR5, 0, 255, 0, bDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 0, bDisplayTextYPosition);
    u8g2.print("6k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + bDisplayBarWidth + bDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR4, 0, 255, 0, bDisplayBarMaxLength);
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 0, bDisplayTextYPosition);
    u8g2.print("2k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + bDisplayBarWidth + bDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR3, 0, 255, 0, bDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition - 1, bDisplayTextYPosition);
    u8g2.print("1k");
        
    bDisplayBarXPosition = bDisplayBarXPosition + bDisplayBarWidth + bDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR2, 0, 255, 0, bDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition - 2, bDisplayTextYPosition);
    u8g2.print("400");

    bDisplayBarXPosition = bDisplayBarXPosition + bDisplayBarWidth + bDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR1, 0, 255, 0, bDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 0, bDisplayTextYPosition);
    u8g2.print("160");
        
    bDisplayBarXPosition = bDisplayBarXPosition + bDisplayBarWidth + bDisplayBarWidthOffset;
    bDisplayBarLength = map(bVolValR0, 0, 255, 0, bDisplayBarMaxLength); 
    u8g2.drawBox(bDisplayBarXPosition, bDisplayBarYPosition - bDisplayBarLength, bDisplayBarWidth, bDisplayBarLength);
    u8g2.setCursor(bDisplayBarXPosition + 2, bDisplayTextYPosition);
    u8g2.print("63");
    
  } while ( u8g2.nextPage() );
  
}

