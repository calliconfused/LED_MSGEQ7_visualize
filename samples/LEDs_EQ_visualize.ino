#include <FastLED.h>

#define NUM_LEDS      32*5
#define DATA_PIN      2
#define CLOCK_PIN     3
#define COLORARRANGE  RBG
#define LED_STRIP     WS2801
#define BRIGHTNESS    96

#define ANALOG_PIN_L  A0
#define ANALOG_PIN_R  A1
#define STROBE_PIN    26
#define RESET_PIN     22

#define FILTER_VAL    80

CRGB LED_X[NUM_LEDS];

#define SPECTRUM_EQ7  7
#define SPECTRUM_SEC  6
#define SECTION_LEN   (NUM_LEDS / SPECTRUM_SEC)

int SPECTRUM_VAL_L[SPECTRUM_SEC];
int SPECTRUM_VAL_R[SPECTRUM_SEC];
int LED_LS[SPECTRUM_SEC];
int LED_LE[SPECTRUM_SEC];
int LED_SECTION[SPECTRUM_SEC];
uint8_t HUE_VAL = 0;

int TMPLED[NUM_LEDS];

void setup() { 

  FastLED.addLeds<LED_STRIP, DATA_PIN, CLOCK_PIN, COLORARRANGE>(LED_X, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  for (int i = 0; i < NUM_LEDS; i++) {
    LED_X[i] = CRGB(0, 0, 0);
  }
  FastLED.show();  
  
  pinMode(ANALOG_PIN_L, INPUT);
  pinMode(ANALOG_PIN_R, INPUT);
  pinMode(STROBE_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);

  analogReference(DEFAULT);
  digitalWrite(RESET_PIN, LOW);
  digitalWrite(STROBE_PIN, HIGH);

}

void loop() {

  digitalWrite(RESET_PIN, HIGH);
  digitalWrite(RESET_PIN, LOW);

  for (int i = 0; i < SPECTRUM_EQ7; i++) {
    digitalWrite(STROBE_PIN, LOW);
    delayMicroseconds(5000);

    SPECTRUM_VAL_L[i] = analogRead(ANALOG_PIN_L);
    SPECTRUM_VAL_L[i] = constrain(SPECTRUM_VAL_L[i], FILTER_VAL, 1023);
    SPECTRUM_VAL_L[i] = map(SPECTRUM_VAL_L[i], FILTER_VAL, 1023, 0, 255);

    SPECTRUM_VAL_R[i] = analogRead(ANALOG_PIN_R);
    SPECTRUM_VAL_R[i] = constrain(SPECTRUM_VAL_R[i], FILTER_VAL, 1023);
    SPECTRUM_VAL_R[i] = map(SPECTRUM_VAL_R[i], FILTER_VAL, 1023, 0, 255);
    
    digitalWrite(STROBE_PIN, HIGH);
  }

  FastLED.clear();

  for (int i = 0; i < SPECTRUM_SEC; i++) {
   
    LED_SECTION[i] = map((SPECTRUM_VAL_L[i] + SPECTRUM_VAL_R[i])/2, 0, 255, 0, SECTION_LEN);

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
