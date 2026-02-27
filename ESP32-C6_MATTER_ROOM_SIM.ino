// =============================================================================
// ESP32-C6_MATTER_ROOM_SIM.ino  –  v4  –  27feb26
// Simulatie van ROOM Matter/HomeKit integratie voor ESP32-C6
// Filip Delannoy – Zarlar thuisautomatisering
//
// DOEL: Test de volledige HomeKit interface zonder echte sensoren.
//       Gesimuleerde waarden fluctueren realistisch.
//       Alle bedieningselementen zijn functioneel via callbacks.
//
// API: arduino-esp32-master 3.3.2 (Matter library meegeleverd)
// HARDWARE: ESP32-C6, 4MB of 16MB flash
// PARTITION: Huge App (verplicht voor Matter)
// =============================================================================

#define Serial Serial0  // Fix: ESP32-C6 gebruikt Serial0

#include <WiFi.h>
#include <nvs_flash.h>   // Voor factory reset via nvs_flash_erase()
#include <Matter.h>
#include <MatterEndPoints/MatterTemperatureSensor.h>
#include <MatterEndPoints/MatterHumiditySensor.h>
#include <MatterEndPoints/MatterOccupancySensor.h>
#include <MatterEndPoints/MatterThermostat.h>
#include <MatterEndPoints/MatterColorLight.h>
#include <MatterEndPoints/MatterOnOffLight.h>

// Niet beschikbaar in 3.3.2:
//   MatterIlluminanceMeasurement  → lux web-UI only
//   MatterAirQuality              → CO₂ web-UI only


// =============================================================================
// WiFi – pas aan voor jouw netwerk (iPhone hotspot)
// =============================================================================
const char* WIFI_SSID = "iPhoneFilip";
const char* WIFI_PASS = "adnoh123";


// =============================================================================
// Matter endpoints
// =============================================================================
MatterTemperatureSensor  matter_temp;       // room_temp
MatterHumiditySensor     matter_humidity;   // humi
MatterOccupancySensor    matter_motion1;    // MOV1 PIR
MatterOccupancySensor    matter_motion2;    // MOV2 PIR
MatterThermostat         matter_thermostat; // heating_setpoint + room_temp
MatterColorLight         matter_pixels;     // Sfeerverlichting (pixels 2..n)
MatterOnOffLight         matter_bed;        // Bed modus
MatterOnOffLight         matter_thuis;      // Thuis / Weg
MatterOnOffLight         matter_pir1_light; // PIR1 manueel override
MatterOnOffLight         matter_pir2_light; // PIR2 manueel override


// =============================================================================
// Gesimuleerde variabelen (in echte sketch: echte sensorwaarden)
// =============================================================================
float   sim_room_temp         = 20.5;
float   sim_humi              = 58.0;
int     sim_co2               = 650;    // web-UI only, geen Matter endpoint
float   sim_lux               = 280.0;  // web-UI only, geen Matter endpoint
bool    sim_motion1           = false;
bool    sim_motion2           = false;
int     sim_heating_setpoint  = 20;
bool    sim_heating_on        = false;
bool    sim_bed               = false;
bool    sim_home_mode         = true;
uint8_t sim_neo_r = 255, sim_neo_g = 200, sim_neo_b = 80;
bool    sim_pixels_on         = false;
bool    sim_pir1_manual       = false;
bool    sim_pir2_manual       = false;

unsigned long last_sim_step   = 0;
unsigned long last_serial_log = 0;


// =============================================================================
// Hulpfuncties
// =============================================================================

float oscil(float center, float amplitude, float period_s, float offset_s) {
  float t = millis() / 1000.0f + offset_s;
  return center + amplitude * sinf(2.0f * PI * t / period_s);
}


// =============================================================================
// Simulatiestap – elke 5 seconden
// =============================================================================
void sim_step() {
  sim_room_temp = oscil(20.5f, 1.5f, 300.0f, 0);
  sim_humi      = constrain(oscil(60.0f, 8.0f, 480.0f, 30), 20.0f, 99.0f);
  sim_co2       = constrain((int)oscil(650, 200, 180, 60), 400, 1200);
  sim_lux       = max(0.0f, oscil(400.0f, 350.0f, 1200.0f, 0));
  sim_motion1   = (random(100) < 15);
  sim_motion2   = (random(100) < 10);

  // Verwarmingslogica (identiek aan echte sketch)
  sim_heating_on = sim_home_mode
    ? (sim_room_temp < sim_heating_setpoint - 0.5f)
    : (sim_room_temp < sim_heating_setpoint - 3.0f);
}


// =============================================================================
// Matter sensor updates (sensoren → HomeKit, elke 5s)
// =============================================================================
void update_matter_sensors() {
  matter_temp.setTemperature(sim_room_temp);
  matter_humidity.setHumidity(sim_humi);
  matter_motion1.setOccupancy(sim_motion1);
  matter_motion2.setOccupancy(sim_motion2);
  matter_thermostat.setLocalTemperature(sim_room_temp);
  // Thermostat setpoint: alleen gezet bij init en via callback vanuit HomeKit
  // CO₂ en lux: geen Matter endpoint beschikbaar → web-UI only
}


// =============================================================================
// Serial statusrapport
// =============================================================================
void print_status() {
  Serial.println(F("\n╔══════════════════════════════════════╗"));
  Serial.println(F(  "║    MATTER_ROOM_SIM  –  Status        ║"));
  Serial.printf(     "║  Uptime: %lu s\n", millis() / 1000);
  Serial.println(F(  "╠══════════════════════════════════════╣"));
  Serial.println(F(  "║ SENSOREN (read-only in HomeKit)       "));
  Serial.printf(     "║  Kamertemp  : %.1f °C\n",  sim_room_temp);
  Serial.printf(     "║  Vochtigheid: %.0f %%\n",  sim_humi);
  Serial.printf(     "║  CO₂        : %d ppm  (web-UI only)\n", sim_co2);
  Serial.printf(     "║  Lux        : %.0f     (web-UI only)\n", sim_lux);
  Serial.printf(     "║  MOV1       : %s\n",       sim_motion1 ? "beweging" : "rust");
  Serial.printf(     "║  MOV2       : %s\n",       sim_motion2 ? "beweging" : "rust");
  Serial.println(F(  "║ BEDIENING (HomeKit → controller)      "));
  Serial.printf(     "║  Thermostat : %d°C  verw=%s\n",
                     sim_heating_setpoint, sim_heating_on ? "AAN" : "UIT");
  Serial.printf(     "║  Bed        : %s\n",       sim_bed ? "AAN" : "UIT");
  Serial.printf(     "║  Thuis/Weg  : %s\n",       sim_home_mode ? "THUIS" : "WEG");
  Serial.printf(     "║  Pixels     : %s  RGB(%d,%d,%d)\n",
                     sim_pixels_on ? "AAN" : "UIT", sim_neo_r, sim_neo_g, sim_neo_b);
  Serial.printf(     "║  PIR1 lamp  : %s\n",       sim_pir1_manual ? "MANUEEL" : "AUTO");
  Serial.printf(     "║  PIR2 lamp  : %s\n",       sim_pir2_manual ? "MANUEEL" : "AUTO");
  Serial.println(F(  "╠══════════════════════════════════════╣"));
  Serial.printf(     "║  Free heap  : %d %%\n",
                     (ESP.getFreeHeap() * 100) / ESP.getHeapSize());
  Serial.printf(     "║  WiFi RSSI  : %d dBm\n",   WiFi.RSSI());
  Serial.println(F(  "╚══════════════════════════════════════╝\n"));
}


// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n=== MATTER_ROOM_SIM v4 opstarten ==="));

  // WiFi verbinding
  Serial.printf("WiFi: verbinden met '%s' ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries++ < 40) {
    delay(500); Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("\nWiFi OK – IP: %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println(F("\nWiFi MISLUKT – controleer SSID en wachtwoord!"));

  // ── Matter endpoints initialiseren ──────────────────────────────────────

  matter_temp.begin();
  matter_humidity.begin();
  matter_motion1.begin();
  matter_motion2.begin();

  // Thermostat: initieel setpoint + callback voor HomeKit → controller
  matter_thermostat.begin();
  matter_thermostat.setLocalTemperature(sim_room_temp);
  matter_thermostat.setHeatingSetpoint((float)sim_heating_setpoint);
  matter_thermostat.onChangeHeatingSetpoint([](double new_sp) -> bool {
    sim_heating_setpoint = constrain((int)round(new_sp), 15, 28);
    Serial.printf("[Matter CMD] Thermostat setpoint → %d °C\n", sim_heating_setpoint);
    // In echte sketch ook: preferences.putInt(NVS_HEATING_SETPOINT, heating_setpoint);
    return true;
  });

  // Kleurlamp (sfeerverlichting): callbacks voor aan/uit en kleur
  matter_pixels.begin();
  matter_pixels.setOnOff(sim_pixels_on);
  matter_pixels.onChangeOnOff([](bool on_off) -> bool {
    sim_pixels_on = on_off;
    Serial.printf("[Matter CMD] Pixels → %s\n", sim_pixels_on ? "AAN" : "UIT");
    // In echte sketch: for(i=2; i<pixels_num; i++) pixel_on[i] = on_off;
    return true;
  });
  matter_pixels.onChangeColorHSV([](HsvColor_t hsv) -> bool {
    // Converteer HSV (0-254 per kanaal) naar RGB voor NeoPixels
    float h = (hsv.h / 254.0f) * 360.0f;
    float s = hsv.s / 254.0f;
    float v = hsv.v / 254.0f;
    float c = v * s, x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f)), m = v - c;
    float rr, gg, bb;
    if      (h < 60)  { rr=c; gg=x; bb=0; }
    else if (h < 120) { rr=x; gg=c; bb=0; }
    else if (h < 180) { rr=0; gg=c; bb=x; }
    else if (h < 240) { rr=0; gg=x; bb=c; }
    else if (h < 300) { rr=x; gg=0; bb=c; }
    else              { rr=c; gg=0; bb=x; }
    sim_neo_r = (uint8_t)((rr+m)*255);
    sim_neo_g = (uint8_t)((gg+m)*255);
    sim_neo_b = (uint8_t)((bb+m)*255);
    Serial.printf("[Matter CMD] Pixels kleur → R=%d G=%d B=%d (HSV: %d,%d,%d)\n",
                  sim_neo_r, sim_neo_g, sim_neo_b, hsv.h, hsv.s, hsv.v);
    // In echte sketch: neo_r=sim_neo_r; neo_g=sim_neo_g; neo_b=sim_neo_b; + NVS opslaan
    return true;
  });

  // Bed modus
  matter_bed.begin();
  matter_bed.setOnOff(sim_bed);
  matter_bed.onChangeOnOff([](bool on_off) -> bool {
    sim_bed = on_off;
    Serial.printf("[Matter CMD] Bed → %s\n", sim_bed ? "AAN" : "UIT");
    // In echte sketch ook: preferences.putBool(NVS_BED_STATE, bed);
    return true;
  });

  // Thuis / Weg
  matter_thuis.begin();
  matter_thuis.setOnOff(sim_home_mode);
  matter_thuis.onChangeOnOff([](bool on_off) -> bool {
    sim_home_mode = on_off;
    Serial.printf("[Matter CMD] Thuis/Weg → %s\n", sim_home_mode ? "THUIS" : "WEG");
    // In echte sketch ook: preferences.putInt(NVS_HOME_MODE_STATE, home_mode);
    return true;
  });

  // PIR1 manueel override (pixel 0: AAN = forceert manueel, UIT = terug AUTO PIR)
  matter_pir1_light.begin();
  matter_pir1_light.setOnOff(sim_pir1_manual);
  matter_pir1_light.onChangeOnOff([](bool on_off) -> bool {
    sim_pir1_manual = on_off;
    Serial.printf("[Matter CMD] PIR1 lamp → %s\n", on_off ? "MANUEEL AAN" : "AUTO (PIR)");
    // In echte sketch: pixel_mode[0] = on_off ? 1 : 0;
    return true;
  });

  // PIR2 manueel override (pixel 1)
  matter_pir2_light.begin();
  matter_pir2_light.setOnOff(sim_pir2_manual);
  matter_pir2_light.onChangeOnOff([](bool on_off) -> bool {
    sim_pir2_manual = on_off;
    Serial.printf("[Matter CMD] PIR2 lamp → %s\n", on_off ? "MANUEEL AAN" : "AUTO (PIR)");
    // In echte sketch: pixel_mode[1] = on_off ? 1 : 0;
    return true;
  });

  // ── Matter starten ───────────────────────────────────────────────────────
  Matter.begin();

  // ── Pairing informatie ───────────────────────────────────────────────────
  Serial.println(F("\n══════════════════════════════════════"));
  if (!Matter.isDeviceCommissioned()) {
    Serial.println(F("MATTER: Nog niet gepaard."));
    Serial.println(F(""));
    Serial.println(F("► Manuele code voor Home app:"));
    Serial.println("    " + Matter.getManualPairingCode());
    Serial.println(F(""));
    Serial.println(F("► Home app → + → Accessoire toevoegen → Meer opties"));
    Serial.println(F("  Kies 'Ik heb geen code' → typ bovenstaande code in"));
    Serial.println(F("  Of: 'Voeg toe via code' → typ de 8-cijferige code"));
    Serial.println(F(""));
    Serial.println(F("Wacht op commissioning (kan 30-60s duren)..."));
    while (!Matter.isDeviceCommissioned()) {
      delay(500); Serial.print(".");
    }
    Serial.println(F("\nGEPAARD! HomeKit accessory actief."));
  } else {
    Serial.println(F("MATTER: Al gepaard. Alle endpoints actief."));
    Serial.println(F("Stuur 'reset' via Serial om pairing te wissen."));
  }
  Serial.println(F("══════════════════════════════════════\n"));

  // Initiële sensorwaarden pushen naar HomeKit
  update_matter_sensors();
  print_status();
}


// =============================================================================
// LOOP
// =============================================================================
void loop() {

  // Serial commando's
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("reset")) {
      Serial.println(F("Matter pairing wissen + reboot..."));
      nvs_flash_erase();  // Wist volledige NVS inclusief Matter commissioning data
      delay(300);
      ESP.restart();
    }
    if (cmd.equalsIgnoreCase("status")) {
      print_status();
    }
  }

  // Simulatiestap elke 5 seconden
  if (millis() - last_sim_step > 5000) {
    last_sim_step = millis();
    sim_step();
    update_matter_sensors();
  }

  // Statusrapport elke 15 seconden
  if (millis() - last_serial_log > 15000) {
    last_serial_log = millis();
    print_status();
  }

  delay(10);
}
