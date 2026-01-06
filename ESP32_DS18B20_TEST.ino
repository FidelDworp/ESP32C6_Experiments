// ESP32_DS18B20_TEST.ino = Proof of concept for DS18B20 temperature sensors on ESP32C6 controllers
// The usual oneWire library does not work!
// I asked many AI's to help me...

#include <Arduino.h>
#include <OneWireNg_CurrentPlatform.h>

#define ONE_WIRE_PIN 3
#define NUM_SENSORS 6

static const OneWireNg::Id sensorROMs[NUM_SENSORS] = {
  {0x28, 0xFF, 0x01, 0x02, 0x03, 0x04, 0x05, 0xA1},
  {0x28, 0xFF, 0x11, 0x12, 0x13, 0x14, 0x15, 0xA2},
  {0x28, 0xFF, 0x21, 0x22, 0x23, 0x24, 0x25, 0xA3},
  {0x28, 0xFF, 0x31, 0x32, 0x33, 0x34, 0x35, 0xA4},
  {0x28, 0xFF, 0x41, 0x42, 0x43, 0x44, 0x45, 0xA5},
  {0x28, 0xFF, 0x51, 0x52, 0x53, 0x54, 0x55, 0xA6}
};

OneWireNg_CurrentPlatform ow(ONE_WIRE_PIN, false);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP32-C6 DS18B20 test (6 sensoren) ===");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last < 2000) return;
  last = millis();

  // Convert all
  ow.skip();  // skip ROM (for single or all)
  ow.write(0x44);  // convert temp
  delay(750);

  Serial.println("---- Temperaturen ----");

  for (int i = 0; i < NUM_SENSORS; i++) {
    ow.reset();
    ow.select(sensorROMs[i].data);
    ow.write(0xBE);  // read scratchpad

    uint8_t scrpd[9];
    for (int j = 0; j < 9; j++) scrpd[j] = ow.read();

    // Manual CRC check
    uint8_t crc = 0;
    for (int j = 0; j < 8; j++) {
      uint8_t inbyte = scrpd[j];
      for (int k = 0; k < 8; k++) {
        uint8_t mix = (crc ^ inbyte) & 0x01;
        crc >>= 1;
        if (mix) crc ^= 0x8C;
        inbyte >>= 1;
      }
    }

    if (crc == scrpd[8]) {
      int16_t raw = (scrpd[1] << 8) | scrpd[0];
      float temp = raw / 16.0;
      Serial.printf("Sensor %d: %.2f °C\n", i+1, temp);
    } else {
      Serial.printf("Sensor %d: CRC fout\n", i+1);
    }
  }

  Serial.println("------------------------\n");
}
