// ESP32C6 DS18B20_ID-scanner.ino = Which temperature sensors are in this bus?
// (First sketch from Claude AI)

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
    for (size_t i = 0; i < sizeof(id.data); i++) {
      Serial.printf("0x%02X", id.data[i]);
      if (i < sizeof(id.data) - 1) Serial.print(", ");
    }
    Serial.println("},");
    
    // Controleer of het een DS18B20 is (family code 0x28)
    if (id.data[0] != 0x28) {
      Serial.println("  -> WAARSCHUWING: Dit is geen DS18B20!");
    }
  }
  
  if (count == 0) {
    Serial.println("GEEN sensoren gevonden!");
    Serial.println("\nControleer:");
    Serial.println("- 4.7kΩ pull-up resistor tussen DATA en VCC?");
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
