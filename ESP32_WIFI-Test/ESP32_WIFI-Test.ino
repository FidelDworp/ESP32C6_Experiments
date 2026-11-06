/*
 * ESP32 Test Sketch
 * =================
 * 
 * Doel: Een eenmalige test voor ESP32-processors (inclusief ESP32-C6) om WiFi-connectie te valideren,
 *       specificaties op te halen en MAC/IP-adressen weer te geven in de Serial Monitor.
 * 
 * Gebruiksinstructies:
 * 1. Selecteer de juiste USB-poort en board type in de Arduino IDE (bijv. "ESP32C6 Dev Module" voor ESP32-C6).
 * 2. Flash deze sketch naar de ESP32.
 * 3. Open de Serial Monitor (baud rate: 115200) om het rapport te zien.
 * 4. De sketch voert een WiFi-test uit met hardcoded credentials ("Delannoy", "kampendaal,34").
 * 5. Als de WiFi-connectie slaagt, worden specificaties, MAC en IP getoond.
 * 6. Dit is een eenmalige uitvoering in setup(); loop() is leeg.
 * 7. Herhaal voor elke ESP32 in serie: flash, bekijk output, unplug en herhaal.
 * 
 * Belangrijke opmerkingen:
 * - Zorg ervoor dat de WiFi-credentials correct zijn en het netwerk bereikbaar is.
 * - Voor ESP32-C6: Zorg dat de juiste board support geïnstalleerd is in Arduino IDE (via Board Manager: ESP32 by Espressif).
 * - Deze sketch is compatibel met alle ESP32-varianten, maar getest met focus op ESP32-C6.
 * - Geen webserver of extra bibliotheken nodig; houdt het eenvoudig en betrouwbaar.
 * - Output is gestructureerd voor leesbaarheid.
 * 
 * Auteur: Grok (gebaseerd op gebruiker's voorbeeld)
 * Datum: Februari 19, 2026
 */

#include <WiFi.h>      // Voor WiFi-functionaliteit
#include <esp_chip_info.h>  // Voor chip-specificaties (ESP32 core)

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // Wacht tot Serial klaar is
  }
  delay(1000);  // Korte delay voor stabiliteit

  Serial.println("\n\n=== ESP32 TEST RAPPORT ===");

  // 1) WiFi Test
  Serial.println("1. WiFi Test:");
  Serial.print("   Verbinden met WiFi (SSID: Delannoy)... ");
  WiFi.begin("Delannoy", "kampendaal,34");

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {  // Timeout na 10 seconden
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n   Succes! Verbonden met WiFi.");
  } else {
    Serial.println("\n   Fout: Kon niet verbinden met WiFi (controleer credentials of bereik).");
    Serial.println("   Einde van test (geen verdere info beschikbaar zonder connectie).");
    Serial.println("==========================\n");
    return;  // Stop als WiFi faalt, want IP/MAC vereist connectie
  }

  // 2) Specificaties
  Serial.println("\n2. Specificaties:");
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);

  Serial.print("   Chip Model: ");
  switch (chip_info.model) {
    case CHIP_ESP32: Serial.println("ESP32"); break;
    case CHIP_ESP32S2: Serial.println("ESP32-S2"); break;
    case CHIP_ESP32S3: Serial.println("ESP32-S3"); break;
    case CHIP_ESP32C3: Serial.println("ESP32-C3"); break;
    case CHIP_ESP32C6: Serial.println("ESP32-C6"); break;
    case CHIP_ESP32H2: Serial.println("ESP32-H2"); break;
    default: Serial.println("Onbekend");
  }

  Serial.println("   Aantal Cores: " + String(chip_info.cores));
  Serial.println("   Revisie: " + String(chip_info.revision));
  Serial.println("   CPU Frequentie: " + String(ESP.getCpuFreqMHz()) + " MHz");

  Serial.println("   Flash Grootte: " + String(ESP.getFlashChipSize() / 1024 / 1024) + " MB");
  Serial.println("   Heap Grootte: " + String(ESP.getHeapSize() / 1024) + " KB");
  Serial.println("   Vrije Heap: " + String(ESP.getFreeHeap() / 1024) + " KB");

  #ifdef CONFIG_SPIRAM_SUPPORT
  Serial.println("   PSRAM Grootte: " + String(ESP.getPsramSize() / 1024 / 1024) + " MB");
  #else
  Serial.println("   PSRAM: Niet ondersteund");
  #endif

  // 3) MAC en IP Address
  Serial.println("\n3. MAC en IP Address:");
  Serial.println("   MAC Address: " + WiFi.macAddress());
  Serial.println("   IP Address: " + WiFi.localIP().toString());

  Serial.println("\nTest voltooid!");
  Serial.println("==========================\n");
}

void loop() {
  // Leeg: Eenmalige uitvoering
}
