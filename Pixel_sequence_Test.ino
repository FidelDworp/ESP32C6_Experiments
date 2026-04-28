// MATRIX_TEST_serpentine.ino
// Test sketch om de exacte pixel mapping te bepalen
// 
// GEBRUIK:
// 1. Upload deze sketch
// 2. Open Serial Monitor (115200 baud)
// 3. Noteer voor elk pixel nummer welke COL en RIJ brandt
// 4. Formaat: "Pixel X → Col Y, Rij Z"

#define Serial Serial0

#include <Adafruit_NeoPixel.h>

#define LED_PIN       4
#define NUM_PIXELS    51    // Test tot pixel 50 voor zekerheid
#define BRIGHTNESS    100

Adafruit_NeoPixel strip(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  delay(500);
  
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.clear();
  strip.show();
  
  Serial.println("\n\n=== MATRIX SERPENTINE TEST ===");
  Serial.println("Noteer voor elk pixel: Col (0-11 van links) en Rij (0-3 van boven)");
  Serial.println("Formaat: Pixel X -> Col Y, Rij Z");
  Serial.println("=====================================\n");
  
  delay(2000);
  
  // Test elk pixel
  for (int i = 0; i < NUM_PIXELS; i++) {
    Serial.printf("Pixel %2d brandt NU (GROEN) - noteer Col & Rij...\n", i);
    
    strip.clear();
    strip.setPixelColor(i, strip.Color(0, 150, 0));  // Groen
    strip.show();
    
    delay(5000);  // 5 seconde per pixel
  }
  
  Serial.println("\n=== TEST COMPLEET ===");
  Serial.println("Start opnieuw: druk RESET knop");
  
  // Rainbow celebratie
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < NUM_PIXELS; i++) {
      strip.setPixelColor(i, strip.ColorHSV(i * 65535 / NUM_PIXELS));
    }
    strip.show();
    delay(300);
    strip.clear();
    strip.show();
    delay(200);
  }
}

void loop() {
  // Niets - test draait in setup()
}
