/*  ESP32_DS18B20_TEST.ino = Proof of concept for DS18B20 temperature sensors on ESP32C6 controllers
The usual oneWire library does not work!
I asked many AI's to help me...

Voorbeeld van serial output:
---- Temperaturen ----
Sensor 1: 22.06 °C
Sensor 2: 22.00 °C
Sensor 3: 21.94 °C
Sensor 4: 21.50 °C
Sensor 5: 21.75 °C
Sensor 6: 21.81 °C
----------------------
*/

#include <Arduino.h>
#include <OneWireNg_CurrentPlatform.h>

#define ONE_WIRE_PIN 3
#define NUM_SENSORS 6

// Jouw échte sensor IDs
static const OneWireNg::Id sensorROMs[NUM_SENSORS] = {
  {0x28, 0x7C, 0xF0, 0x03, 0x00, 0x00, 0x80, 0x59},
  {0x28, 0x72, 0xDB, 0x03, 0x00, 0x00, 0x80, 0xC2},
  {0x28, 0xAA, 0xFB, 0x03, 0x00, 0x00, 0x80, 0x5F},
  {0x28, 0x49, 0xDD, 0x03, 0x00, 0x00, 0x80, 0x4B},
  {0x28, 0xC3, 0xD6, 0x03, 0x00, 0x00, 0x80, 0x1E},
  {0x28, 0xDB, 0xB5, 0x03, 0x00, 0x00, 0x80, 0xBB}
};

OneWireNg_CurrentPlatform ow(ONE_WIRE_PIN, false);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32-C6 DS18B20 6-Sensor Reader ===");
  Serial.println("Klaar om te meten!\n");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last < 2000) return;
  last = millis();

  // Start conversie voor alle sensoren tegelijk
  ow.reset();
  ow.writeByte(0xCC); // Skip ROM command
  ow.writeByte(0x44); // Convert temp command
  delay(750); // Wacht op conversie

  Serial.println("---- Temperaturen ----");
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    ow.reset();
    ow.writeByte(0x55); // Match ROM command
    for (int j = 0; j < 8; j++) {
      ow.writeByte(sensorROMs[i][j]); // Stuur sensor ID
    }
    ow.writeByte(0xBE); // Read scratchpad
    
    uint8_t data[9];
    for (int j = 0; j < 9; j++) {
      data[j] = ow.readByte();
    }
    
    // CRC check
    uint8_t crc = 0;
    for (int j = 0; j < 8; j++) {
      uint8_t inbyte = data[j];
      for (int k = 0; k < 8; k++) {
        uint8_t mix = (crc ^ inbyte) & 0x01;
        crc >>= 1;
        if (mix) crc ^= 0x8C;
        inbyte >>= 1;
      }
    }
    
    if (crc == data[8]) {
      int16_t raw = (data[1] << 8) | data[0];
      float temp = raw / 16.0;
      Serial.printf("Sensor %d: %.2f °C\n", i + 1, temp);
    } else {
      Serial.printf("Sensor %d: CRC fout!\n", i + 1);
    }
  }
  
  Serial.println("----------------------\n");
}
