// ESP32C6_TEST.ino = Test voor controller (ESP32-C6)

#include <Adafruit_NeoPixel.h>

#define BUTTON_PIN 0       // BOOT knop (IO0)
#define NEOPIXEL_PIN 8
#define NEO_NUM 1

Adafruit_NeoPixel pixels(NEO_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

String report = "";
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
  Serial.println("\n=== ESP32-C6 Pin Test – Freenove Breakout Board ===");

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

  pinMode(BUTTON_PIN, INPUT_PULLUP);
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
        Serial.print("Duurtijd: ");
        Serial.print(testDuration);
        Serial.println(" seconden per pin");
        break;
      }
    }
  }

  Serial.println("\nTest begint – druk BOOT knop of typ 'k' + Enter voor OK");
  delay(2000);

  // Alle pins LOW zetten (behalve IO0)
  Serial.println("Alle pins (behalve IO0) op LOW zetten...");
  for (int i = 0; i < currentLeftLength; i++) {
    int gpio = currentPinMapLeft[i];
    if (gpio != 0) {
      pinMode(gpio, OUTPUT);
      digitalWrite(gpio, LOW);
    }
  }
  for (int i = 0; i < currentRightLength; i++) {
    int gpio = currentPinMapRight[i];
    if (gpio != 0) {
      pinMode(gpio, OUTPUT);
      digitalWrite(gpio, LOW);
    }
  }
  Serial.println("Alle pins LOW – klaar voor test");

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

  // NeoPixel test
  testNeoPixel();

  // Rapport
  Serial.println("\n=== VOLLEDIG TEST RAPPORT ===");
  Serial.print(report);

  // Eén regenboog aan het einde
  Serial.println("Eén regenboog op NeoPixel...");
  for (int j = 0; j < 256; j++) {
    pixels.setPixelColor(0, Wheel(j & 255));
    pixels.show();
    delay(20);
  }
  pixels.clear();
  pixels.show();

  // Pas nu alle pins vrijgeven (naar INPUT)
  Serial.println("Alle pins vrijgegeven (naar INPUT) – test voltooid!");
  for (int i = 0; i < currentLeftLength; i++) {
    int gpio = currentPinMapLeft[i];
    pinMode(gpio, INPUT);
  }
  for (int i = 0; i < currentRightLength; i++) {
    int gpio = currentPinMapRight[i];
    pinMode(gpio, INPUT);
  }
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // zeker pull-up

  Serial.println("Board is nu in veilige staat.");
}

void testPin(int position, const char* side, int gpio) {
  Serial.print(side);
  Serial.print(position);
  Serial.print(" = IO");
  Serial.println(gpio);

  pinMode(gpio, OUTPUT);

  unsigned long start = millis();
  bool confirmed = false;

  while (millis() - start < (unsigned long)testDuration * 1000) {
    digitalWrite(gpio, HIGH);
    delay(250);
    digitalWrite(gpio, LOW);
    delay(250);

    if (gpio != 0 && digitalRead(BUTTON_PIN) == LOW) {
      delay(20);
      if (digitalRead(BUTTON_PIN) == LOW) {
        confirmed = true;
        Serial.println("  → OK (BOOT knop)");
        while (digitalRead(BUTTON_PIN) == LOW) delay(10);
        break;
      }
    }

    if (Serial.available()) {
      String in = Serial.readStringUntil('\n');
      in.trim();
      if (in.equalsIgnoreCase("k")) {
        confirmed = true;
        Serial.println("  → OK ('k' getypt)");
        break;
      }
    }
  }

  String result = confirmed ? "OK" : "NG";
  report += String(side) + position + " (IO" + gpio + "): " + result + "\n";
  Serial.println("  → " + result);

  // Na test: terug naar LOW (niet INPUT!)
  digitalWrite(gpio, LOW);
  // pinMode blijft OUTPUT – alleen aan einde naar INPUT
}

void testNeoPixel() {
  Serial.println("\n--- NeoPixel IO8 – Regenboog (10s) ---");
  bool confirmed = false;
  unsigned long start = millis();

  while (millis() - start < 10000) {
    for (int j = 0; j < 256; j++) {
      pixels.setPixelColor(0, Wheel(j & 255));
      pixels.show();
      delay(20);

      if (digitalRead(BUTTON_PIN) == LOW) {
        delay(20);
        if (digitalRead(BUTTON_PIN) == LOW) {
          confirmed = true;
          Serial.println("  → NeoPixel OK (BOOT knop)");
          break;
        }
      }

      if (Serial.available()) {
        String in = Serial.readStringUntil('\n');
        in.trim();
        if (in.equalsIgnoreCase("k")) {
          confirmed = true;
          Serial.println("  → NeoPixel OK ('k')");
          break;
        }
      }
    }
    if (confirmed) break;
  }

  pixels.clear();
  pixels.show();

  String result = confirmed ? "OK" : "NG";
  report += "NeoPixel IO8: " + result + "\n";
  Serial.println("  → " + result);
}

void loop() {}

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
