/* 
 *  aktiviere u8g2
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
/*
 * aktiviere MSGEQ7
 */

int analogPinL = 0; // MSGEQ7 OUT
int analogPinR = 1; // MSGEQ7 OUT
int strobePin = 2; // MSGEQ7 STROBE
int resetPin = 4; // MSGEQ7 RESET
int spectrumValueL[7];
int spectrumValueR[7];
 
int filterValue = 80;

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

    // Remove serial stuff after debugging
    Serial.print(spectrumValueL[i]);
    Serial.print(" ");
    Serial.print(spectrumValueR[i]);
    Serial.print(" ");
    
    digitalWrite(strobePin, HIGH);
   }
 
   Serial.println();

  u8g2.setFont(u8g2_font_profont11_mf);
  u8g2.firstPage();
  do {
    u8g2.setCursor(0, 7);
    u8g2.print("L");

    u8g2.setCursor(0, 16);
    u8g2.print("R");

    u8g2.setCursor(10, 25);
    u8g2.print("1");    
    
    u8g2.setCursor(10, 7);
    u8g2.print(spectrumValueL[1]);
    u8g2.setCursor(10, 16);
    u8g2.print(spectrumValueR[1]);

    u8g2.setCursor(30, 25);
    u8g2.print("2");
    
    u8g2.setCursor(30, 7);
    u8g2.print(spectrumValueL[2]);
    u8g2.setCursor(30, 16);
    u8g2.print(spectrumValueR[2]);    
    
    u8g2.setCursor(50, 25);
    u8g2.print("3");
    
    u8g2.setCursor(50, 7);
    u8g2.print(spectrumValueL[3]);
    u8g2.setCursor(50, 16);
    u8g2.print(spectrumValueR[3]);
    
    u8g2.setCursor(70, 25);
    u8g2.print("4");
    
    u8g2.setCursor(70, 7);
    u8g2.print(spectrumValueL[4]);
    u8g2.setCursor(70, 16);
    u8g2.print(spectrumValueR[4]);
    
    u8g2.setCursor(90, 25);
    u8g2.print("5");    
    
    u8g2.setCursor(90, 7);
    u8g2.print(spectrumValueL[5]);
    u8g2.setCursor(90, 16);
    u8g2.print(spectrumValueR[5]);
    
    u8g2.setCursor(110, 25);
    u8g2.print("6");
    
    u8g2.setCursor(110, 7);
    u8g2.print(spectrumValueL[6]);
    u8g2.setCursor(110, 16);
    u8g2.print(spectrumValueR[6]);
    
  } while ( u8g2.nextPage() );

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

