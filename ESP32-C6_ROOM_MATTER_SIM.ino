// =============================================================================
// ESP32-C6_ROOM_MATTER_SIM.ino  –  v6  –  28feb26  14:00
// Simulatie van ROOM Matter/HomeKit integratie voor ESP32-C6
// Filip Delannoy – Zarlar thuisautomatisering
//
// DOEL: Test de volledige HomeKit interface zonder echte sensoren.
//       Gesimuleerde waarden fluctueren realistisch.
//       Alle bedieningselementen zijn functioneel via callbacks.
//       Variabelenamen = identiek aan TESTROOM.ino voor eenvoudige integratie.
//
// FAKE SENSORS (niet beschikbaar als eigen Matter type in Apple Home):
//   CO2  (ppm)  → MatterTemperatureSensor  (waarde = co2÷100, hernoem naar "CO2 ÷100")
//                 Voorbeeld: 650 ppm → toont "6.5°C" in HomeKit
//   lux         → MatterTemperatureSensor  (waarde = sun_light÷10, hernoem naar "Lux ÷10")
//                 Voorbeeld: 280 lux → toont "28.0°C" in HomeKit
//   MatterPressureSensor wordt GENEGEERD door Apple Home → niet bruikbaar als fake
//
// API: arduino-esp32-master 3.3.2
// HARDWARE: ESP32-C6, Huge App partition
// =============================================================================

#define Serial Serial0

#include <WiFi.h>
#include <nvs_flash.h>
#include <Matter.h>
#include <MatterEndPoints/MatterTemperatureSensor.h>
#include <MatterEndPoints/MatterHumiditySensor.h>
#include <MatterEndPoints/MatterOccupancySensor.h>
#include <MatterEndPoints/MatterThermostat.h>
#include <MatterEndPoints/MatterColorLight.h>
#include <MatterEndPoints/MatterOnOffLight.h>


// =============================================================================
// WiFi
// =============================================================================
const char* WIFI_SSID = "iPhoneFilip";
const char* WIFI_PASS = "adnoh123";


// =============================================================================
// Matter endpoints
// =============================================================================
MatterTemperatureSensor  matter_temp;       // room_temp  (°C)
MatterHumiditySensor     matter_humidity;   // humi       (%)
MatterOccupancySensor    matter_motion1;    // MOV1 PIR
MatterOccupancySensor    matter_motion2;    // MOV2 PIR
MatterTemperatureSensor  matter_co2;        // co2 FAKE   (ppm÷100 als °C – hernoem naar "CO2 ÷100")
MatterTemperatureSensor  matter_lux;        // sun_light FAKE (lux/10 vermeld als °C – hernoem naar "Lux /10")
MatterThermostat         matter_thermostat; // heating_setpoint + room_temp
MatterColorLight         matter_pixels;     // neo_r/g/b  (sfeerverlichting)
MatterOnOffLight         matter_bed;        // bed        (0/1)
MatterOnOffLight         matter_thuis;      // home_mode  (0=Weg, 1=Thuis)
MatterOnOffLight         matter_pir1_light; // pixel_mode[0] (0=AUTO, 1=MANUEEL)
MatterOnOffLight         matter_pir2_light; // pixel_mode[1] (0=AUTO, 1=MANUEEL)


// =============================================================================
// Gesimuleerde variabelen – zelfde namen als TESTROOM.ino
// =============================================================================
float   room_temp        = 20.5;
float   humi             = 58.0;
int     co2              = 650;
float   sun_light        = 280.0;
bool    mov1_light       = false;   // PIR1 actief (= bezetting)
bool    mov2_light       = false;   // PIR2 actief (= bezetting)
int     heating_setpoint = 20;
int     heating_on       = 0;       // 0/1 zoals in TESTROOM
int     home_mode        = 1;       // 1=Thuis, 0=Weg
int     bed              = 0;       // 0/1
uint8_t neo_r = 255, neo_g = 200, neo_b = 80;  // Room kleurinstelling voor alle pixels
int     pixel_mode[2]    = {0, 0};  // 0=AUTO, 1=MANUEEL (MOV1, MOV2)
uint8_t thermostat_mode  = 0;       // 0=UIT, 4=VERWARMING (Matter spec)

unsigned long last_sim_step   = 0;
unsigned long last_serial_log = 0;
unsigned long mov1_off_time   = 0;  // Millis waarop MOV1 terug op rust gaat
unsigned long mov2_off_time   = 0;  // Millis waarop MOV2 terug op rust gaat
#define MOV_HOLD_MS 20000           // Beweging blijft 20s zichtbaar in HomeKit
                                    // Triggerfrequentie: 8% per 5s = gem. elke 62s
                                    // Hold (20s) << interval (62s) → duidelijke rust zichtbaar


// =============================================================================
// Hulpfunctie: sinusoïdale oscillatie
// =============================================================================
float oscil(float center, float amplitude, float period_s, float offset_s) {
  float t = millis() / 1000.0f + offset_s;
  return center + amplitude * sinf(2.0f * PI * t / period_s);
}


// =============================================================================
// Simulatiestap – elke 5 seconden
// =============================================================================
void sim_step() {
  room_temp  = oscil(20.5f, 1.5f,  300.0f, 0);
  humi       = constrain(oscil(60.0f, 8.0f, 480.0f, 30), 20.0f, 99.0f);
  co2        = constrain((int)oscil(650, 200, 180, 60), 400, 1200);
  sun_light  = max(0.0f, oscil(400.0f, 350.0f, 1200.0f, 0));
  // PIR simulatie met hold-timer: trigger zet timer, timer bepaalt bezetting
  // MOV1: 8% kans per 5s = gemiddeld elke 62s trigger, hold=20s → duidelijke rust zichtbaar
  if (random(100) < 8) {
    mov1_off_time = millis() + MOV_HOLD_MS;
    Serial.println(F("[SIM] MOV1 trigger!"));
  }
  mov1_light = (millis() < mov1_off_time);

  // MOV2: 5% kans per 5s = gemiddeld elke 100s trigger
  if (random(100) < 5) {
    mov2_off_time = millis() + MOV_HOLD_MS;
    Serial.println(F("[SIM] MOV2 trigger!"));
  }
  mov2_light = (millis() < mov2_off_time);

  // Verwarmingslogica (identiek aan TESTROOM AUTO/home_mode logica)
  heating_on = (home_mode == 1)
    ? (room_temp < heating_setpoint - 0.5f) ? 1 : 0
    : (room_temp < heating_setpoint - 3.0f) ? 1 : 0;
}


// =============================================================================
// Matter sensor updates – alle read-only sensoren → HomeKit
// =============================================================================
void update_matter_sensors() {
  matter_temp.setTemperature(room_temp);
  matter_humidity.setHumidity(humi);
  matter_motion1.setOccupancy(mov1_light);
  matter_motion2.setOccupancy(mov2_light);
  matter_co2.setTemperature(co2 / 100.0f);      // FAKE: CO2 ppm÷100 als "°C"
  matter_lux.setTemperature(sun_light / 10.0f); // FAKE: lux/10 als "°C"
  matter_thermostat.setLocalTemperature(room_temp);
}


// =============================================================================
// Serial statusrapport
// =============================================================================
void print_status() {
  const char* th_mode_str = (thermostat_mode == 4) ? "VERWARMING" :
                            (thermostat_mode == 1) ? "AUTO" :
                            (thermostat_mode == 3) ? "KOELING" : "UIT";

  Serial.println(F("\n╔══════════════════════════════════════╗"));
  Serial.println(F(  "║    MATTER_ROOM_SIM  –  Status        ║"));
  Serial.printf(     "║  Uptime: %lu s\n", millis() / 1000);
  Serial.println(F(  "╠══════════════════════════════════════╣"));
  Serial.println(F(  "║ SENSOREN                              "));
  Serial.printf(     "║  room_temp  : %.1f °C\n",  room_temp);
  Serial.printf(     "║  humi       : %.0f %%\n",  humi);
  Serial.printf(     "║  co2        : %d ppm\n",   co2);
  Serial.printf(     "║  sun_light  : %.0f lux\n", sun_light);
  Serial.printf(     "║  mov1_light : %s\n",       mov1_light ? "beweging" : "rust");
  Serial.printf(     "║  mov2_light : %s\n",       mov2_light ? "beweging" : "rust");
  Serial.println(F(  "║ BEDIENING (HomeKit → controller)      "));
  Serial.printf(     "║  heating    : %d°C  modus=%s\n", heating_setpoint, th_mode_str);
  Serial.printf(     "║  heating_on : %s\n",       heating_on ? "AAN" : "UIT");
  Serial.printf(     "║  bed        : %s\n",       bed  ? "AAN" : "UIT");
  Serial.printf(     "║  home_mode  : %s\n",       home_mode ? "THUIS" : "WEG");
  Serial.printf(     "║  neo_r/g/b  : RGB(%d,%d,%d)  (room kleur)\n", neo_r, neo_g, neo_b);
  Serial.printf(     "║  pixel_mode[0] MOV1: %s\n", pixel_mode[0] ? "MANUEEL" : "AUTO");
  Serial.printf(     "║  pixel_mode[1] MOV2: %s\n", pixel_mode[1] ? "MANUEEL" : "AUTO");
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
  delay(500);
  Serial.println(F("\n=== MATTER_ROOM_SIM v6 ==="));

  // ── WiFi ─────────────────────────────────────────────────────────────────
  Serial.printf("WiFi: verbinden met '%s' ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nWiFi OK: %s\n", WiFi.localIP().toString().c_str());

  // ── Matter endpoints initialiseren ───────────────────────────────────────
  matter_temp.begin();
  matter_humidity.begin();
  matter_motion1.begin();
  matter_motion2.begin();
  matter_co2.begin();
  matter_lux.begin();

  // Thermostat: heating-only (ControlSequenceOfOperation = 2)
  matter_thermostat.begin((MatterThermostat::ControlSequenceOfOperation_t)2);
  matter_thermostat.setLocalTemperature(room_temp);
  matter_thermostat.setHeatingSetpoint((float)heating_setpoint);
  // Min/max (15-28°C) wordt afgedwongen via constrain() in de onChangeHeatingSetpoint callback

  matter_thermostat.onChangeHeatingSetpoint([](double new_sp) -> bool {
    heating_setpoint = constrain((int)round(new_sp), 15, 28);
    Serial.printf("[HomeKit] heating_setpoint → %d °C\n", heating_setpoint);
    return true;
  });
  matter_thermostat.onChangeMode([](uint8_t mode) -> bool {
    thermostat_mode = mode;
    const char* s = (mode==0)?"UIT":(mode==1)?"AUTO":(mode==3)?"KOELING":(mode==4)?"VERWARMING":"?";
    Serial.printf("[HomeKit] thermostat_mode → %s (%d)\n", s, mode);
    return true;
  });

  // ── Sfeerverlichting kleurpicker ─────────────────────────────────────────
  // MEMO INTEGRATIE IN TESTROOM:
  //   matter_pixels stelt neo_r/g/b in voor de hele room.
  //   Dit is GEEN aan/uit schakelaar voor de pixels!
  //   De on/off toggle in HomeKit wordt genegeerd en altijd op "aan" gehouden,
  //   zodat de kleurpicker altijd beschikbaar blijft.
  //   Individuele pixels gaan aan/uit via pixel_on[i] in de pixelloop.
  //   Als neo_r/g/b wijzigt via HomeKit → alle pixel_on[i]==true pixels
  //   nemen automatisch de nieuwe kleur over via setTargetColor(i, neo_r, neo_g, neo_b).
  //   NVS opslaan na kleurwijziging: preferences.putUChar(NVS_NEO_R, neo_r); etc.
  matter_pixels.begin();
  matter_pixels.setOnOff(true);
  matter_pixels.onChangeOnOff([](bool on_off) -> bool {
    // Niet doorgeven aan pixels – on/off heeft hier geen betekenis
    // We melden altijd "aan" terug zodat HomeKit de switch niet toont als UIT
    matter_pixels.setOnOff(true);
    Serial.println(F("[HomeKit] matter_pixels on/off genegeerd (kleurpicker only)"));
    return true;
  });
  matter_pixels.onChangeColorHSV([](HsvColor_t hsv) -> bool {
    float h = (hsv.h / 254.0f) * 360.0f;
    float s = hsv.s / 254.0f;
    float v = hsv.v / 254.0f;
    float c = v*s, x = c*(1.0f-fabsf(fmodf(h/60.0f,2.0f)-1.0f)), m = v-c;
    float rr,gg,bb;
    if      (h<60)  {rr=c;gg=x;bb=0;}
    else if (h<120) {rr=x;gg=c;bb=0;}
    else if (h<180) {rr=0;gg=c;bb=x;}
    else if (h<240) {rr=0;gg=x;bb=c;}
    else if (h<300) {rr=x;gg=0;bb=c;}
    else            {rr=c;gg=0;bb=x;}
    neo_r = (uint8_t)((rr+m)*255);
    neo_g = (uint8_t)((gg+m)*255);
    neo_b = (uint8_t)((bb+m)*255);
    Serial.printf("[HomeKit] neo_r/g/b → %d,%d,%d\n", neo_r, neo_g, neo_b);
    return true;
  });

  // Bed
  matter_bed.begin();
  matter_bed.setOnOff(bed);
  matter_bed.onChangeOnOff([](bool on_off) -> bool {
    bed = on_off ? 1 : 0;
    Serial.printf("[HomeKit] bed → %s\n", bed ? "AAN" : "UIT");
    return true;
  });

  // Thuis / Weg
  matter_thuis.begin();
  matter_thuis.setOnOff(home_mode);
  matter_thuis.onChangeOnOff([](bool on_off) -> bool {
    home_mode = on_off ? 1 : 0;
    Serial.printf("[HomeKit] home_mode → %s\n", home_mode ? "THUIS" : "WEG");
    return true;
  });

  // PIR1 manueel override
  matter_pir1_light.begin();
  matter_pir1_light.setOnOff(pixel_mode[0]);
  matter_pir1_light.onChangeOnOff([](bool on_off) -> bool {
    pixel_mode[0] = on_off ? 1 : 0;
    Serial.printf("[HomeKit] pixel_mode[0] MOV1 → %s\n", pixel_mode[0] ? "MANUEEL" : "AUTO");
    return true;
  });

  // PIR2 manueel override
  matter_pir2_light.begin();
  matter_pir2_light.setOnOff(pixel_mode[1]);
  matter_pir2_light.onChangeOnOff([](bool on_off) -> bool {
    pixel_mode[1] = on_off ? 1 : 0;
    Serial.printf("[HomeKit] pixel_mode[1] MOV2 → %s\n", pixel_mode[1] ? "MANUEEL" : "AUTO");
    return true;
  });

  // ── Matter starten ───────────────────────────────────────────────────────
  Matter.begin();

  // ── Pairing check ────────────────────────────────────────────────────────
  Serial.println(F("\n══════════════════════════════════════"));
  if (!Matter.isDeviceCommissioned()) {
    Serial.println(F("MATTER: Nog niet gepaard."));
    Serial.println(F("► Manuele code:  " ));
    Serial.println("    " + Matter.getManualPairingCode());
    Serial.println(F("► Home app → + → Accessoire → Meer opties → code invoeren"));
    Serial.println(F("Wacht op commissioning..."));
    while (!Matter.isDeviceCommissioned()) { delay(500); Serial.print("."); }
    Serial.println(F("\nGEPAARD!"));
  } else {
    Serial.println(F("MATTER: Al gepaard. Typ 'reset' om te wissen."));
  }
  Serial.println(F("══════════════════════════════════════\n"));

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
      nvs_flash_erase();
      delay(300);
      ESP.restart();
    }
    if (cmd.equalsIgnoreCase("status")) {
      print_status();
    }
  }

  // Simulatiestap elke 5s
  if (millis() - last_sim_step > 5000) {
    last_sim_step = millis();
    sim_step();
    update_matter_sensors();
  }

  // Statusrapport elke 15s
  if (millis() - last_serial_log > 15000) {
    last_serial_log = millis();
    print_status();
  }
}
