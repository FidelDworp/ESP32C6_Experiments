/* ESP32C6 DS18B20_ID-scanner.ino = Which temperature sensors are in this bus? (First sketch from Claude AI)
Voorbeeld van een scan:

=== ESP32-C6 DS18B20 Scanner ===
Zoeken naar sensoren...

Sensor 1 gevonden: {0x28, 0x7C, 0xF0, 0x03, 0x00, 0x00, 0x80, 0x59},
Sensor 2 gevonden: {0x28, 0x72, 0xDB, 0x03, 0x00, 0x00, 0x80, 0xC2},
Sensor 3 gevonden: {0x28, 0xAA, 0xFB, 0x03, 0x00, 0x00, 0x80, 0x5F},
Sensor 4 gevonden: {0x28, 0x49, 0xDD, 0x03, 0x00, 0x00, 0x80, 0x4B},
Sensor 5 gevonden: {0x28, 0xC3, 0xD6, 0x03, 0x00, 0x00, 0x80, 0x1E},
Sensor 6 gevonden: {0x28, 0xDB, 0xB5, 0x03, 0x00, 0x00, 0x80, 0xBB},

Totaal: 6 sensor(en) gevonden
Kopieer de ROM codes hierboven naar je code!
*/

#include <Arduino.h>
#include <OneWireNg_CurrentPlatform.h>

#define ONE_WIRE_PIN 3

OneWireNg_CurrentPlatform ow(ONE_WIRE_PIN, false);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== ESP32-C6 DS18B20 Scanner ===");
  Serial.println("Zoeken naar sensoren...\n");
  
  OneWireNg::Id id;
  int count = 0;
  
  ow.searchReset();
  while (ow.search(id) == OneWireNg::EC_MORE) {
    count++;
    Serial.printf("Sensor %d gevonden: ", count);
    Serial.print("{");
    for (size_t i = 0; i < sizeof(id); i++) {
      Serial.printf("0x%02X", id[i]);
      if (i < sizeof(id) - 1) Serial.print(", ");
    }
    Serial.println("},");
    
    // Controleer of het een DS18B20 is (family code 0x28)
    if (id[0] != 0x28) {
      Serial.println("  -> WAARSCHUWING: Dit is geen DS18B20!");
    }
  }
  
  if (count == 0) {
    Serial.println("GEEN sensoren gevonden!");
    Serial.println("\nControleer:");
    Serial.println("- 4.7k pull-up resistor tussen DATA en VCC?");
    Serial.println("- Sensoren correct aangesloten (GND, VCC, DATA)?");
    Serial.println("- Juiste pin (GPIO 3)?");
  } else {
    Serial.printf("\nTotaal: %d sensor(en) gevonden\n", count);
    Serial.println("\nKopieer de ROM codes hierboven naar je code!");
  }
}

void loop() {
  // Scan 1x bij opstarten
}
