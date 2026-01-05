// ESP32C6_TEST.ino = Test voor controller (ESP32-C6)

#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN 8
#define NEO_NUM 1

Adafruit_NeoPixel pixels(NEO_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

bool is30Pin = false;
int testDuration = 10;

// 30-pin versie
int positionsLeft30[] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
int pinMapLeft30[] = {9, 18, 19, 20, 21, 22, 23, 15, 17, 16, 3, 2};
int leftLength30 = 12;

int positionsRight30[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
int pinMapRight30[] = {13, 12, 11, 10, 8, 1, 0, 7, 6, 5, 4};
int rightLength30 = 11;

// 32-pin versie
int positionsLeft32[] = {2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
int pinMapLeft32[] = {12, 13, 9, 18, 19, 20, 21, 22, 23, 15, 17, 16};
int leftLength32 = 12;

int positionsRight32[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
int pinMapRight32[] = {3, 2, 11, 10, 8, 1, 0, 7, 6, 5, 4};
int rightLength32 = 11;

int* currentPositionsLeft;
int* currentPinMapLeft;
int currentLeftLength;

int* currentPositionsRight;
int* currentPinMapRight;
int currentRightLength;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32-C6 Eenvoudige Pin Test ===");

  // Board keuze
  Serial.println("Typ '30' of '32' voor board versie:");
  while (true) {
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      input.trim();
      if (input == "30" || input == "32") {
        is30Pin = (input == "30");
        Serial.println(is30Pin ? "30-pin gekozen" : "32-pin gekozen");
        break;
      }
    }
  }

  // Mapping
  if (is30Pin) {
    currentPositionsLeft = positionsLeft30;
    currentPinMapLeft = pinMapLeft30;
    currentLeftLength = leftLength30;
    currentPositionsRight = positionsRight30;
    currentPinMapRight = pinMapRight30;
    currentRightLength = rightLength30;
  } else {
    currentPositionsLeft = positionsLeft32;
    currentPinMapLeft = pinMapLeft32;
    currentLeftLength = leftLength32;
    currentPositionsRight = positionsRight32;
    currentPinMapRight = pinMapRight32;
    currentRightLength = rightLength32;
  }

  pixels.begin();
  pixels.clear();

  // Testduur
  Serial.println("\nTyp aantal seconden per pin (1-20):");
  while (true) {
    if (Serial.available()) {
      String dur = Serial.readStringUntil('\n');
      dur.trim();
      int d = dur.toInt();
      if (d >= 1 && d <= 20) {
        testDuration = d;
        Serial.print("Elke pin knippert ");
        Serial.print(testDuration);
        Serial.println(" seconden");
        break;
      }
    }
  }

  Serial.println("\nTest begint automatisch...");
  delay(2000);

  // Alle pins LOW zetten
  Serial.println("Alle pins op LOW...");
  for (int i = 0; i < currentLeftLength; i++) {
    int gpio = currentPinMapLeft[i];
    pinMode(gpio, OUTPUT);
    digitalWrite(gpio, LOW);
  }
  for (int i = 0; i < currentRightLength; i++) {
    int gpio = currentPinMapRight[i];
    pinMode(gpio, OUTPUT);
    digitalWrite(gpio, LOW);
  }

  // Linker rij
  Serial.println("\n--- Linker rij ---");
  for (int i = 0; i < currentLeftLength; i++) {
    testPin(currentPositionsLeft[i], "Linker rij L", currentPinMapLeft[i]);
  }

  // Rechter rij
  Serial.println("\n--- Rechter rij ---");
  for (int i = 0; i < currentRightLength; i++) {
    testPin(currentPositionsRight[i], "Rechter rij R", currentPinMapRight[i]);
  }

  // Einde boodschap + regenboog
  Serial.println("\nEinde van de test");
  Serial.println("Regenboog op NeoPixel...");
  Serial.flush();  // Zorgt dat tekst zeker verschijnt

  // Kortere regenboog (128 stappen ≈ 2.5s)
  for (int j = 0; j < 128; j++) {
    pixels.setPixelColor(0, Wheel(j & 255));
    pixels.show();
    delay(20);
  }
  pixels.clear();
  pixels.show();

  // Alle pins vrijgeven
  for (int i = 0; i < currentLeftLength; i++) pinMode(currentPinMapLeft[i], INPUT);
  for (int i = 0; i < currentRightLength; i++) pinMode(currentPinMapRight[i], INPUT);

  Serial.println("\nBoard in veilige staat – klaar!");
  Serial.flush();
}

void testPin(int position, const char* side, int gpio) {
  Serial.print(side);
  Serial.print(position);
  Serial.print(" = IO");
  Serial.println(gpio);

  pinMode(gpio, OUTPUT);

  unsigned long start = millis();
  while (millis() - start < (unsigned long)testDuration * 1000) {
    digitalWrite(gpio, HIGH);
    delay(250);
    digitalWrite(gpio, LOW);
    delay(250);
  }

  digitalWrite(gpio, LOW);
}

void loop() {
  // Niets
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) return pixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  if (WheelPos < 170) {
    WheelPos -= 85;
    return pixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
