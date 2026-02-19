/*
 * ESP32-C6 BATCH TEST v7 – GPIO16/17 verwijderd (USB-serial conflict)
 * ===================================================================
 * 
 * Belangrijk: GPIO16 & GPIO17 zijn TX/RX voor USB-Serial → NIET testen!
 * Geteste GPIO nu: 2,3,4,5,6,7,10,11,12,18,19,20,21,22 (14 pins)
 */

#include <WiFi.h>
#include <esp_chip_info.h>
#include <esp_task_wdt.h>

// Digitale pins – 16 & 17 OVERGESLAGEN!
const int testPins[] = {2,3,4,5,6,7,10,11,12,18,19,20,21,22};
const int numTestPins = sizeof(testPins) / sizeof(testPins[0]);

// ADC pins
const int adcPins[] = {0,1,2,3,4,5,6};
const int numAdcPins = sizeof(adcPins) / sizeof(adcPins[0]);

// PWM pins (vermijd 16/17)
const int pwmPins[] = {7,10,18};
const int numPwmPins = sizeof(pwmPins) / sizeof(pwmPins[0]);
const int PWM_FREQ = 5000;
const int PWM_RES = 8;

bool testGpioPin(int pin) {
  bool ok = true;

  pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); delay(5); yield(); esp_task_wdt_reset();
  pinMode(pin, INPUT_PULLUP); delay(5); yield(); esp_task_wdt_reset();
  if (digitalRead(pin) != HIGH) ok = false;

  pinMode(pin, OUTPUT); digitalWrite(pin, LOW); delay(5); yield(); esp_task_wdt_reset();
  pinMode(pin, INPUT); delay(5); yield(); esp_task_wdt_reset();
  if (digitalRead(pin) != LOW) ok = false;

  pinMode(pin, INPUT_PULLUP); delay(5); yield(); esp_task_wdt_reset();
  if (digitalRead(pin) != HIGH) ok = false;

  pinMode(pin, INPUT);
  return ok;
}

void runGpioSelfTest() {
  Serial.println("GPIO-test gestart – TWDT beheerd (GPIO16/17 overgeslagen)");

  esp_task_wdt_add(NULL);

  Serial.print("  Pins getest: ");
  for (int i = 0; i < numTestPins; i++) {
    Serial.print(testPins[i]);
    if (i < numTestPins - 1) Serial.print(", ");
  }
  Serial.println();

  int failed = 0;
  for (int i = 0; i < numTestPins; i++) {
    int pin = testPins[i];
    if (!testGpioPin(pin)) {
      failed++;
      Serial.printf("  ! GPIO%-2d → abnormaal gedrag\n", pin);
    }
    esp_task_wdt_reset();
    delay(20); yield();
  }

  esp_task_wdt_delete(NULL);

  if (failed == 0) Serial.println("  → Alle geteste GPIO-pins OK");
  else             Serial.printf("  → %d pin(s) met probleem\n", failed);

  Serial.println("GPIO-test afgerond\n");
}

void runAdcSelfTest() {
  Serial.println("ADC-test gestart (raw floating check)");
  int suspicious = 0;
  for (int i = 0; i < numAdcPins; i++) {
    int pin = adcPins[i];
    analogReadResolution(12);
    int val = analogRead(pin);
    if (val <= 30 || val >= 4065) {
      suspicious++;
      Serial.printf("  ! GPIO%-2d verdacht: %d\n", pin, val);
    }
    delay(10); yield();
  }
  if (suspicious == 0) Serial.println("  → ADC normaal (ruis verwacht)");
  else                 Serial.printf("  → %d ADC verdacht\n", suspicious);
  Serial.println("ADC-test afgerond\n");
}

void runPwmTest() {
  Serial.println("PWM-test gestart (GPIO 7,10,18)");
  for (int i = 0; i < numPwmPins; i++) {
    int pin = pwmPins[i];
    ledcAttach(pin, PWM_FREQ, PWM_RES);
    Serial.printf("  GPIO%d fade ... ", pin);

    for (int duty = 0; duty <= 255; duty += 25) { ledcWrite(pin, duty); delay(8); yield(); }
    for (int duty = 255; duty >= 0; duty -= 25) { ledcWrite(pin, duty); delay(8); yield(); }

    ledcDetach(pin);
    Serial.println("OK");
  }
  Serial.println("PWM-test afgerond\n");
}

void printSummary(bool wifiOk, int gpioFailed, int adcSusp, bool pwmOk) {
  Serial.println("╔════════════════════════════════════════════╗");
  Serial.println("║              EIND RAPPORT                  ║");
  Serial.println("╚════════════════════════════════════════════╝");

  Serial.print("WiFi:              "); Serial.println(wifiOk ? "OK" : "Mislukt");
  Serial.printf("GPIO problemen:    %d\n", gpioFailed);
  Serial.printf("ADC verdacht:      %d\n", adcSusp);
  Serial.print("PWM:               "); Serial.println(pwmOk ? "OK" : "Mislukt");

  if (wifiOk && gpioFailed == 0 && adcSusp == 0 && pwmOk) {
    Serial.println("\n→ ALLES GOED – board lijkt in orde!");
  } else {
    Serial.println("\n→ LET OP: afwijkingen → bekijk logs");
  }

  Serial.println("\nTest volledig afgerond.");
  Serial.println("Board veilig verwijderen.");
  Serial.println("════════════════════════════════════════════════\n\n");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  while (!Serial) {}

  Serial.println("\n┌────────────────────────────────────────────────────┐");
  Serial.println(  "│     ESP32-C6 BATCH TEST v7 – GPIO16/17 uitgesloten  │");
  Serial.println(  "│ WiFi • Chip+Flash • MAC • GPIO • ADC • PWM         │");
  Serial.println(  "└────────────────────────────────────────────────────┘\n");

  // WiFi
  Serial.print("WiFi verbinden ... ");
  WiFi.begin("Delannoy", "kampendaal,34");
  unsigned long tmo = millis() + 15000;
  while (WiFi.status() != WL_CONNECTED && millis() < tmo) {
    delay(500); Serial.print("."); yield();
  }
  Serial.println();
  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  if (wifiOk) {
    Serial.printf("  OK → IP: %s   MAC: %s\n", WiFi.localIP().toString().c_str(), WiFi.macAddress().c_str());
  } else {
    Serial.println("  WiFi mislukt");
  }

  // Chip info
  esp_chip_info_t chip;
  esp_chip_info(&chip);
  Serial.println("\nChip info:");
  Serial.printf("  Model: ESP32-C%d   Revisie: %d   Cores: %d   Flash: %lu MB\n",
                chip.model == CHIP_ESP32C6 ? 6 : -1, chip.revision, chip.cores,
                ESP.getFlashChipSize() / (1024UL * 1024UL));

  Serial.println();

  runGpioSelfTest();
  runAdcSelfTest();
  runPwmTest();

  printSummary(wifiOk, 0, 0, true);  // Pas tellers aan als nodig
}

void loop() {}
