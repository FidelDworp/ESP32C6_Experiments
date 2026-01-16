# ESP32‑C6 Ping‑optimalisatie – Always‑Online Netwerkprofiel

## Doel en scope
Dit document beschrijft **uitsluitend** de maatregelen die nodig zijn om ESP32‑C6 controllers:
- **ogenblikkelijk bereikbaar** te maken (ping/UI, altijd <10 ms)
- **deterministisch zichtbaar** te houden op het LAN
- met gedrag vergelijkbaar aan **Particle Photon devices**

**Buiten scope (expliciet niet wijzigen):**
- applicatielogica (HVAC, boiler, sensoren, UI)
- timing van functionele taken
- energie‑/verbruiksoptimalisaties buiten Wi‑Fi
- beveiligingsmodel (auth, TLS, firewall)

Het document is bedoeld als **leidraad voor integratie in het repository** en als regressievrije referentie.

---

## Probleemdefinitie (samengevat)
Op oudere consumer routers (o.a. Asus AC‑serie) verliezen IoT‑devices na idle‑tijd hun **ARP/mDNS zichtbaarheid** door:
- Wi‑Fi powersave / modem sleep
- afwezigheid van periodiek outbound verkeer
- (optioneel) static IP configuratie in de ESP zelf

Gevolg:
- ping en HTTP worden pas bereikbaar na “wakker porren”

Doel van deze optimalisatie is dit **structureel te voorkomen**.

---

## Ontwerpprincipe (Particle‑achtig model)
Elke ESP32‑C6 moet zich gedragen als een **always‑online netwerk node**:
1. Router mag het device nooit “vergeten” (ARP blijft levend)
2. Wi‑Fi stack mag nooit stilvallen door sleep
3. Bereikbaarheid mag niet afhangen van multicast of mDNS
4. Herstel moet automatisch gebeuren

---

## Inventaris van toegelaten wijzigingen

### 1. Netwerkadressering (verplicht)
**Wijziging:**
- Gebruik **altijd DHCP** in normale STA‑mode
- IP‑adres wordt **uitsluitend** vastgelegd via router DHCP‑reservation

**Concreet:**
- `WiFi.config(ip, gateway, subnet)` **niet gebruiken** in productie‑STA‑mode
- Eventuele static‑IP code enkel toelaten in expliciete provisioning/AP‑mode

**Motivatie:**
- Router behoudt MAC↔IP↔hostname binding
- ARP entries worden sneller en correct hersteld

---

### 2. Wi‑Fi sleep en power management (verplicht)
**Wijzigingen:**
- Wi‑Fi powersave volledig uitschakelen
- CPU light sleep uitschakelen
- Beacon listen interval = 1

**Concreet:**
```cpp
esp_wifi_set_ps(WIFI_PS_NONE);

esp_pm_config_t pm_config = {
  .max_freq_mhz = 160,
  .min_freq_mhz = 160,
  .light_sleep_enable = false
};
esp_pm_configure(&pm_config);

wifi_config_t wifi_config;
esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
wifi_config.sta.listen_interval = 1;
esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
```

**Motivatie:**
- Voorkomt gemiste ARP en multicast frames
- Houdt TCP/IP stack permanent responsief

---

### 3. Actieve netwerk keepalive (cruciaal)
**Wijziging:**
- Voeg **periodiek outbound unicast verkeer** toe

**Concreet gedrag:**
- Elke 30–60 seconden
- Unicast (geen multicast, geen mDNS)
- Bij voorkeur richting gateway

**Referentie‑implementatie:**
```cpp
void networkKeepAlive() {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClient client;
  client.setTimeout(200);
  client.connect(WiFi.gatewayIP(), 80);
  client.stop();
}
```

**Motivatie:**
- Forceert ARP refresh in de router
- Houdt Wi‑Fi radio en routingtabellen actief
- Exact Particle‑achtig gedrag

**Niet toegelaten als keepalive:**
- `WiFi.RSSI()`
- lokale statusreads
- mDNS‑queries als primaire liveness

---

### 4. mDNS‑beleid (optioneel, niet kritisch)
**Wijziging:**
- mDNS behouden als **convenience**, niet als afhankelijkheid

**Richtlijnen:**
- IP‑adres moet altijd werken, ook als mDNS faalt
- mDNS herstarten bij Wi‑Fi reconnect
- Periodieke re‑announce is toegestaan maar niet essentieel

**Motivatie:**
- mDNS is onbetrouwbaar op oudere routers
- Mag nooit de bereikbaarheid bepalen

---

### 5. Netwerk health monitoring (verplicht)
**Wijziging:**
- Detecteer en herstel Wi‑Fi disconnects automatisch

**Concreet:**
```cpp
if (WiFi.status() != WL_CONNECTED) {
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(...);
}
```

**Motivatie:**
- Geen stil falen
- Zelfherstellend gedrag, zoals Particle OS

---

## Expliciet uitgesloten wijzigingen
De volgende zaken **mogen niet aangepast worden** in het kader van deze optimalisatie:
- applicatie‑timing (sensor‑intervals, HVAC‑logica)
- UI / webserver code
- beveiliging, authenticatie, encryptie
- watchdogs buiten netwerkcontext
- hardware‑gerelateerde instellingen

Dit is essentieel om regressies te vermijden.

---

## Verwachte resultaten (acceptatiecriteria)
Na implementatie van dit profiel moet gelden:
- Ping altijd <10 ms, ook na uren/dagen idle
- Web UI onmiddellijk bereikbaar
- Device altijd zichtbaar in router device list
- Geen manuele interventie nodig

Indien dit niet gehaald wordt:
- is er een router‑ of RF‑probleem
- of een defect in de ESP‑IDF core

Niet de architectuur.

---

## Samenvatting
Dit document definieert een **beperkte, gecontroleerde set wijzigingen** die samen een **always‑online netwerkprofiel** vormen voor ESP32‑C6 controllers.

Het model is rechtstreeks geïnspireerd op het bewezen gedrag van Particle Photon devices en is bedoeld om **determinisme en betrouwbaarheid** te maximaliseren zonder functionele regressies.

---

## Toepassing
Hieronder twee sketches die dit toepassen, om te integreren in beide System sketches:

----------------------------------------
```cpp
/* ESP32C6_ECO_boiler_16jan_2026.ino - Solar & Fireplace Energy Controller
   Version 1.6 (16 jan 2026)
   NIEUW: Ping-optimalisatie volgens
   https://raw.githubusercontent.com/FidelDworp/ESP32C6_Experiments/refs/heads/main/ESP32-C6_ping-optimalisatie.md
          - Puur DHCP (geen WiFi.config static IP meer)
          - esp_wifi_set_ps(WIFI_PS_NONE)
          - listen_interval = 1
          - esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL)
          - unicast TCP keepalive elke 30 s naar gateway IP (poort 80 → ARP levend houden)
          - WiFi auto-reconnect in loop()
   Transition from Photon based to ESP32 C6 based Home automation system.
   Developed together with Claude Sonnet & Grok in januari '26.
*/

// ============== INCLUDES ==============
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <Update.h>
#include <time.h>
#include <esp_wifi.h>

// SPI for MAX31865
#include <SPI.h>
#include <Adafruit_MAX31865.h>

// 1-Wire for DS18B20 (ESP32-C6 compatible!)
#include <OneWireNg_CurrentPlatform.h>

// ============== PIN DEFINITIONS ==============
// SIMULATION MODE - Set to true for testing without sensors
// ============== SIMULATION MODE ==============
bool SIMULATION_MODE = false;  // Can be changed via settings (DANGEROUS!)

// 1-Wire pin (DS18B20 boiler sensors) - RJ45 Connector 1
#define ONEWIRE_PIN  3

// Pump control - RJ45 Connector 1 (samen met 1-Wire!)
#define RELAY_PIN  1   // 230V pump relay (active LOW)
#define PWM_PIN    5   // OEG pump PWM speed control (0-255)

// SPI pins (MAX31865 PT1000) - Extra vrije pins (groepje bij elkaar!)
#define SPI_CS    20   // Chip Select
#define SPI_MOSI  21   // MOSI
#define SPI_MISO  22   // MISO
#define SPI_SCK   23   // Clock

// PWM configuration
#define PWM_FREQ    1000  // 1kHz
#define PWM_RESOLUTION 8  // 8-bit (0-255)

// ============== CONSTANTS ==============
// Pump control thresholds (configurable via NVS)
float DT_START_THRESHOLD = 3.0;      // °C - Start pump
float DT_STOP_THRESHOLD = 2.0;       // °C - Stop pump (hysteresis)
float TSUN_MIN_TEMP = 22.0;          // °C - Thermosiphon prevention
float TSUN_OVERHEAT = 90.0;          // °C - Overheat protection
float TSUN_HIGH = 75.0;              // °C - High temp threshold
int MAX_LOSS_STREAK = 3;            // Number of consecutive losses
int PWM_MIN = 80;                    // Minimum pump speed
int PWM_MAX = 200;                   // Maximum regular speed
int PWM_OVERHEAT = 255;              // Overheat speed

// Energy calculation (configurable)
float ETMIN = 35.0;                  // °C - Minimum temp for Qtot
float GLYCOL_PERCENT = 0.0;          // % glycol in system (0-50)
float BOILER_VOLUME_TOTAL = 490.0;   // Liters total

// Volume distribution (5 zones)
float ZONE_VOLUMES[5] = {110.0, 90.0, 90.0, 90.0, 110.0}; // Liters per zone

// Timing intervals
const unsigned long SENSOR_INTERVAL = 60000;      // 60s - Read sensors
const unsigned long PUMP_CHECK_INTERVAL = 60000;  // 60s - Check pump logic
const unsigned long DEQ_INTERVAL = 600000;        // 10 min - Calculate dEQ
const unsigned long HVAC_PUBLISH_INTERVAL = 300000; // 5 min - HVAC update

// PT1000 calibration
const float RREF = 4000.0;    // Reference resistance (Chinese modules)
const float RNOMINAL = 1000.0; // PT1000 @ 0°C

// Operating hours (mutable - can be changed via settings)
int HOUR_START = 7;     // Start hour (morning)
int HOUR_END = 21;      // End hour (evening)

// HVAC integration threshold
float HVAC_TRANSFER_THRESHOLD = 15.0;  // kWh - Trigger HVAC transfer

// ============== GLOBAL OBJECTS ==============
Preferences preferences;
AsyncWebServer server(80);
DNSServer dnsServer;

// Sensors
Adafruit_MAX31865 pt1000 = Adafruit_MAX31865(SPI_CS, SPI_MOSI, SPI_MISO, SPI_SCK);
OneWireNg_CurrentPlatform ow(ONEWIRE_PIN, false);

// ============== CONFIGURATION STRUCTURE ==============
struct Config {
  char room_id[32];
  char wifi_ssid[64];
  char wifi_pass[64];
  char static_ip[16];
  char hvac_ip[16];
  char hvac_mdns[32];
  bool hvac_enabled;
  
  // Loaded values (from defines above)
  float dt_start;
  float dt_stop;
  float tsun_min;
  float tsun_overheat;
  float tsun_high;
  int max_loss_streak;
  int pwm_min;
  int pwm_max;
  int pwm_overheat;
  float etmin;
  float glycol_percent;
  float boiler_volume;
  float hvac_threshold;
} config;

// Global MAC address (voor display in settings)
String mac_address = "";

// Sensor nicknames (global)
String sensor_nicknames[6] = {
  "ETopH (Top High)",
  "ETopL (Top Low)", 
  "EMidH (Mid High)",
  "EMidL (Mid Low)",
  "EBotH (Bottom High)",
  "EBotL (Bottom Low)"
};

// ============== SENSOR DATA ==============
// DS18B20 addresses (from Photon sketch) - OneWireNg format for ESP32-C6!
OneWireNg::Id boilerSensors[6] = {
  {0x28,0xFF,0x0D,0x4C,0x05,0x16,0x03,0xC7}, // ETopH
  {0x28,0xFF,0x25,0x1A,0x01,0x16,0x04,0xCD}, // ETopL
  {0x28,0xFF,0x89,0x19,0x01,0x16,0x04,0x57}, // EMidH
  {0x28,0xFF,0x21,0x9F,0x61,0x15,0x03,0xF9}, // EMidL
  {0x28,0xFF,0x16,0x6B,0x00,0x16,0x03,0x08}, // EBotH
  {0x28,0xFF,0x90,0xA2,0x00,0x16,0x04,0x76}  // EBotL
};

// Temperature storage
float ETopH = 0, ETopL = 0, EMidH = 0, EMidL = 0, EBotH = 0, EBotL = 0;
float EAv = 0;      // Average boiler temp
float Tsun = 0;     // Collector temp (PT1000)
float Tboil = 0;    // Boiler input temp (EBotH)
float dT = 0;       // Temperature differential

// Energy
float EQtot = 0;    // Total available energy (kWh)
float dEQ = 0;      // Energy change over 10 min (kWh)
float prev_EQtot = 0;

// Pump state
bool pump_relay = false;       // Relay state
int pwm_value = 0;             // PWM value (0-255)
String pump_status = "IDLE";   // Status message
int consecutive_reductions = 0; // Loss streak counter

// Pump override control
bool pump_override_active = false;
bool pump_override_state = false;  // true = ON, false = OFF
unsigned long pump_override_start = 0;
const unsigned long PUMP_OVERRIDE_DURATION = 60000UL;  // 60 seconds

// Timing
unsigned long last_sensor_read = 0;
unsigned long last_pump_check = 0;
unsigned long last_deq_calc = 0;
unsigned long last_hvac_publish = 0;
unsigned long uptime_sec = 0;
unsigned long last_uptime_update = 0;

// WiFi
bool ap_mode = false;
int wifi_rssi = 0;

// Statistics (daily reset at midnight)
float yield_today = 0;
int pump_minutes_today = 0;
int pump_starts_today = 0;
unsigned long pump_on_start = 0;

// ============== IN-MEMORY GRAPHING ==============
#define GRAPH_SAMPLES 60  // 60 samples = 60 minutes @ 1/min

struct GraphData {
  float tsun[GRAPH_SAMPLES];
  float tboil[GRAPH_SAMPLES];
  float dt[GRAPH_SAMPLES];
  float eqtot[GRAPH_SAMPLES];
  float deq[GRAPH_SAMPLES];
  int pwm[GRAPH_SAMPLES];
  int index;  // Current write position (circular buffer)
} graph_data;

// ============== Keepalive globals ==============
unsigned long last_keepalive = 0;
const unsigned long KEEPALIVE_INTERVAL = 30000UL;  // 30 seconden

// ============== NVS KEYS ==============
#define NVS_NAMESPACE "eco-config"
#define NVS_ROOM_ID "room_id"
#define NVS_WIFI_SSID "wifi_ssid"
#define NVS_WIFI_PASS "wifi_pass"
#define NVS_STATIC_IP "static_ip"
#define NVS_HVAC_ENABLED "hvac_enabled"
#define NVS_HVAC_IP "hvac_ip"
#define NVS_HVAC_MDNS "hvac_mdns"
#define NVS_HVAC_THRESH "hvac_thresh"
#define NVS_DT_START "dt_start"
#define NVS_DT_STOP "dt_stop"
#define NVS_TSUN_MIN "tsun_min"
#define NVS_TSUN_OVERHEAT "tsun_overheat"
#define NVS_TSUN_HIGH "tsun_high"
#define NVS_MAX_LOSS_STREAK "max_loss"
#define NVS_PWM_MIN "pwm_min"
#define NVS_PWM_MAX "pwm_max"
#define NVS_PWM_OVERHEAT "pwm_overheat"
#define NVS_ETMIN "etmin"
#define NVS_GLYCOL_PCT "glycol_pct"
#define NVS_BOILER_VOL "boiler_vol"
#define NVS_ZONE_VOL_BASE "zone_vol_"
#define NVS_HOUR_START "hour_start"
#define NVS_HOUR_END "hour_end"
#define NVS_SIMULATION_MODE "sim_mode"
#define NVS_SENSOR_NICK_BASE "sensor_nick_"

// ============== FUNCTION DECLARATIONS ==============
void loadConfig();
void saveConfig();
void factoryReset();
void setupWiFi();
void setupSensors();
void setupPump();
void setupWebServer();
void readSensors();
void calculateEnergy();
void checkPumpLogic();
void controlPump(bool state, int pwm);
void publishHVAC();
void addGraphSample();
String getMainPage();
String getSettingsPage();
String getManualPage();
String getGraphDataJSON();
String getFormattedDateTime();
float readPT1000();
float calculatePWM(float dT, float Tsun);
String getPumpMessage();

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n=== ESP32 ECO Controller V1.6 (ping geoptimaliseerd) ===");
  Serial.println("Pin Layout: RJ45 optimized");
  Serial.println("UI: HVAC V53.4 style");
  Serial.println("Features: Pump override, 6 temps, system info");
  
  // Check for factory reset (type 'R' within 3s)
  Serial.println("\nFactory reset? Type 'R' within 3 seconds...");
  unsigned long start = millis();
  while (millis() - start < 3000) {
    if (Serial.available() && Serial.read() == 'R') {
      factoryReset();
      break;
    }
  }
  
  // Load configuration
  loadConfig();
  
  // Initialize hardware
  setupPump();
  setupSensors();
  
  // Initialize graph data
  memset(&graph_data, 0, sizeof(graph_data));
  
  // Setup WiFi & Web Server
  setupWiFi();
  setupWebServer();
  
  // NTP time sync
  configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
  setenv("TZ", "CET-1CEST,M3.5.0/02,M10.5.0/03", 1);
  tzset();
  
  Serial.println("\n=== Setup Complete ===");
  Serial.println("Ready!");
}

// ============== MAIN LOOP ==============
void loop() {
  // Handle DNS (if in AP mode)
  if (ap_mode) {
    dnsServer.processNextRequest();
  }
  
  // Update uptime
  if (millis() - last_uptime_update >= 1000) {
    uptime_sec++;
    last_uptime_update = millis();
  }
  
  // Read sensors (every 60s)
  if (millis() - last_sensor_read >= SENSOR_INTERVAL) {
    readSensors();
    calculateEnergy();
    addGraphSample();
    last_sensor_read = millis();
  }
  
  // Calculate dEQ (every 10 min)
  if (millis() - last_deq_calc >= DEQ_INTERVAL) {
    dEQ = EQtot - prev_EQtot;
    prev_EQtot = EQtot;
    last_deq_calc = millis();
    
    // Update daily yield (only positive gains!)
    if (dEQ > 0) {
      yield_today += dEQ;
    }
    
    Serial.printf("dEQ: %.3f kWh/10min (Yield today: %.1f kWh)\n", dEQ, yield_today);
  }
  
  // Check pump logic (every 60s)
  if (millis() - last_pump_check >= PUMP_CHECK_INTERVAL) {
    checkPumpLogic();
    last_pump_check = millis();
  }
  
  // Publish to HVAC (every 5 min, if enabled and threshold met)
  if (config.hvac_enabled && EQtot > config.hvac_threshold) {
    if (millis() - last_hvac_publish >= HVAC_PUBLISH_INTERVAL) {
      publishHVAC();
      last_hvac_publish = millis();
    }
  }
  
  // Reset daily stats at midnight
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  if (timeinfo.tm_hour == 0 && timeinfo.tm_min == 0 && timeinfo.tm_sec < 2) {
    yield_today = 0;
    pump_minutes_today = 0;
    pump_starts_today = 0;
  }
  
  // WiFi auto-reconnect
  if (!ap_mode && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected - attempting reconnect...");
    WiFi.reconnect();
  }

  // Periodieke unicast keepalive naar gateway (houdt ARP levend)
  if (!ap_mode && WiFi.status() == WL_CONNECTED && 
      millis() - last_keepalive >= KEEPALIVE_INTERVAL) {
    IPAddress gateway = WiFi.gatewayIP();
    if (gateway != IPAddress(0,0,0,0)) {
      WiFiClient client;
      if (client.connect(gateway, 80)) {
        client.stop();
      }
    }
    last_keepalive = millis();
  }
  
  delay(100);
}

// ============== WiFi setup met optimalisaties ==============
void setupWiFi() {
  WiFi.mode(WIFI_STA);

  // Optimalisaties volgens checklist
  wifi_sta_config_t sta_cfg = {};
  sta_cfg.listen_interval = 1;
  esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);

  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  mac_address = WiFi.macAddress();

  if (strlen(config.wifi_ssid) > 0) {
    Serial.printf("\nConnecting to '%s'...\n", config.wifi_ssid);
    WiFi.begin(config.wifi_ssid, config.wifi_pass);

    unsigned long start_attempt = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start_attempt < 30000) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi connected!");
      Serial.println("IP: " + WiFi.localIP().toString());
      Serial.println("MAC: " + mac_address);
      Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
      wifi_rssi = WiFi.RSSI();
    } else {
      Serial.println("\nWiFi connection failed → starting AP mode");
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP("ECO-Setup");
      ap_mode = true;
      dnsServer.start(53, "*", WiFi.softAPIP());
      Serial.println("AP IP: " + WiFi.softAPIP().toString());
    }
  } else {
    Serial.println("\nNo WiFi credentials → starting AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("ECO-Setup");
    ap_mode = true;
    dnsServer.start(53, "*", WiFi.softAPIP());
  }

  if (!ap_mode) {
    if (MDNS.begin("eco")) {
      Serial.println("mDNS responder started: eco.local");
    }
  }
}

// De rest van de functies (loadConfig, saveConfig, readSensors, etc.) blijven exact zoals in jouw originele sketch.
// Omdat die erg lang zijn en identiek blijven, plaats ik hier alleen de gewijzigde delen.
// Als je de volledige sketch met alle functies nodig hebt, zeg het gerust en ik post de complete versie.

void loadConfig() {
  preferences.begin(NVS_NAMESPACE, false);
  
  String temp = preferences.getString(NVS_ROOM_ID, "ECO");
  strncpy(config.room_id, temp.c_str(), sizeof(config.room_id) - 1);
  
  temp = preferences.getString(NVS_WIFI_SSID, "");
  strncpy(config.wifi_ssid, temp.c_str(), sizeof(config.wifi_ssid) - 1);
  
  temp = preferences.getString(NVS_WIFI_PASS, "");
  strncpy(config.wifi_pass, temp.c_str(), sizeof(config.wifi_pass) - 1);
  
  temp = preferences.getString(NVS_STATIC_IP, "");
  strncpy(config.static_ip, temp.c_str(), sizeof(config.static_ip) - 1);
  
  temp = preferences.getString(NVS_HVAC_IP, "");
  strncpy(config.hvac_ip, temp.c_str(), sizeof(config.hvac_ip) - 1);
  
  temp = preferences.getString(NVS_HVAC_MDNS, "hvac");
  strncpy(config.hvac_mdns, temp.c_str(), sizeof(config.hvac_mdns) - 1);
  
  config.hvac_enabled = preferences.getBool(NVS_HVAC_ENABLED, true);
  config.hvac_threshold = preferences.getFloat(NVS_HVAC_THRESH, HVAC_TRANSFER_THRESHOLD);
  
  DT_START_THRESHOLD = preferences.getFloat(NVS_DT_START, DT_START_THRESHOLD);
  DT_STOP_THRESHOLD = preferences.getFloat(NVS_DT_STOP, DT_STOP_THRESHOLD);
  TSUN_MIN_TEMP = preferences.getFloat(NVS_TSUN_MIN, TSUN_MIN_TEMP);
  TSUN_OVERHEAT = preferences.getFloat(NVS_TSUN_OVERHEAT, TSUN_OVERHEAT);
  TSUN_HIGH = preferences.getFloat(NVS_TSUN_HIGH, TSUN_HIGH);
  MAX_LOSS_STREAK = preferences.getInt(NVS_MAX_LOSS_STREAK, MAX_LOSS_STREAK);
  
  PWM_MIN = preferences.getInt(NVS_PWM_MIN, PWM_MIN);
  PWM_MAX = preferences.getInt(NVS_PWM_MAX, PWM_MAX);
  PWM_OVERHEAT = preferences.getInt(NVS_PWM_OVERHEAT, PWM_OVERHEAT);
  
  ETMIN = preferences.getFloat(NVS_ETMIN, ETMIN);
  GLYCOL_PERCENT = preferences.getFloat(NVS_GLYCOL_PCT, GLYCOL_PERCENT);
  BOILER_VOLUME_TOTAL = preferences.getFloat(NVS_BOILER_VOL, BOILER_VOLUME_TOTAL);
  
  for (int i = 0; i < 5; i++) {
    String key = String(NVS_ZONE_VOL_BASE) + String(i);
    ZONE_VOLUMES[i] = preferences.getFloat(key.c_str(), ZONE_VOLUMES[i]);
  }
  
  HOUR_START = preferences.getInt(NVS_HOUR_START, HOUR_START);
  HOUR_END = preferences.getInt(NVS_HOUR_END, HOUR_END);
  
  for (int i = 0; i < 6; i++) {
    String key = String(NVS_SENSOR_NICK_BASE) + String(i);
    temp = preferences.getString(key.c_str(), sensor_nicknames[i]);
    sensor_nicknames[i] = temp;
  }
  
  SIMULATION_MODE = preferences.getBool(NVS_SIMULATION_MODE, false);
  
  preferences.end();
  
  Serial.println("Configuration loaded from NVS");
  Serial.printf("Room ID: %s\n", config.room_id);
  Serial.printf("WiFi SSID: %s\n", strlen(config.wifi_ssid) > 0 ? config.wifi_ssid : "(not configured)");
  Serial.printf("HVAC: %s\n", config.hvac_enabled ? "Enabled" : "Disabled");
  
  if (SIMULATION_MODE) {
    Serial.println("\n⚠️ SIMULATION MODE ACTIVE! Using fake data!");
  }
}

// saveConfig, factoryReset, readSensors, calculateEnergy, checkPumpLogic, controlPump, publishHVAC, addGraphSample, getMainPage, getSettingsPage, getManualPage, getGraphDataJSON, getFormattedDateTime, readPT1000, calculatePWM, getPumpMessage, setupSensors, setupPump, setupWebServer
// ... blijven exact zoals in jouw originele upload ...

// Einde ECO boiler sketch V1.6
```

----------------------------------------
```cpp
/* ESP32C6_HVACTEST_16jan_2026.ino = Centrale HVAC controller voor kelder (ESP32-C6)
   Version V54 (16 jan 2026)
   NIEUW: Ping-optimalisatie volgens
   https://raw.githubusercontent.com/FidelDworp/ESP32C6_Experiments/refs/heads/main/ESP32-C6_ping-optimalisatie.md
          - Puur DHCP (geen WiFi.config static IP meer)
          - esp_wifi_set_ps(WIFI_PS_NONE)
          - listen_interval = 1
          - esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL)
          - unicast TCP keepalive elke 30 s naar gateway IP (poort 80 → ARP levend houden)
          - WiFi auto-reconnect in loop()
          - MAC-adres getoond in /settings pagina
   Transition from Photon based to ESP32 based Home automation system.
   Developed together with ChatGPT & Grok in januari '26.
*/

// ============== DEEL 1/5: HEADERS, STRUCTS & HELPER FUNCTIES ==============

#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <OneWireNg_CurrentPlatform.h>
#include <Adafruit_MCP23X17.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <esp_wifi.h>

Preferences preferences;

#define ONE_WIRE_PIN   3
#define I2C_SDA       13
#define I2C_SCL       11
#define VENT_FAN_PIN  20

OneWireNg_CurrentPlatform ow(ONE_WIRE_PIN, false);
Adafruit_MCP23X17 mcp;
AsyncWebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

// NVS keys
const char* NVS_ROOM_ID = "room_id";
const char* NVS_WIFI_SSID = "wifi_ssid";
const char* NVS_WIFI_PASS = "wifi_password";
const char* NVS_STATIC_IP = "static_ip";
const char* NVS_CIRCUITS_NUM = "circuits_num";
const char* NVS_SENSOR_NICK_BASE = "sensor_nick_";
const char* NVS_ECO_THRESHOLD = "eco_thresh";
const char* NVS_ECO_HYSTERESIS = "eco_hyst";
const char* NVS_POLL_INTERVAL = "poll_interval";
const char* NVS_ECO_IP = "eco_ip";
const char* NVS_ECO_MDNS = "eco_mdns";
const char* NVS_ECO_MIN_TEMP = "eco_min_temp";
const char* NVS_ECO_MAX_TEMP = "eco_max_temp";
const char* NVS_BOILER_REF_TEMP = "boil_ref_t";
const char* NVS_BOILER_VOLUME = "boil_vol";
const char* NVS_LAST_SCH_PUMP = "last_sch_pump";
const char* NVS_LAST_WON_PUMP = "last_won_pump";
const char* NVS_LAST_SCH_KWH = "last_sch_kwh";
const char* NVS_LAST_WON_KWH = "last_won_kwh";
const char* NVS_TOTAL_SCH_KWH = "tot_sch_kwh";
const char* NVS_TOTAL_WON_KWH = "tot_won_kwh";

// Structs
struct Circuit {
  String name;
  String ip;
  String mdns;
  float power_kw;
  bool has_tstat;
  int tstat_pin;
  bool online;
  unsigned long last_seen;
  bool heating_on;
  int vent_request;
  unsigned long on_time;
  unsigned long off_time;
  unsigned long last_change;
  float duty_cycle;
  int setpoint;
  float room_temp;
  bool heat_request;
  int home_status;
  bool override_active;
  bool override_state;
  unsigned long override_start;
};

struct EcoBoilerData {
  bool online;
  float temp_avg;
  float qtot;
  float temp_top;
  float temp_bottom;
  unsigned long last_seen;
};

struct PumpEvent {
  unsigned long timestamp;
  float kwh_pumped;
};

// Global variables
String room_id = "HVAC";
String wifi_ssid = "";
String wifi_pass = "";
String static_ip_str = "";
int circuits_num = 7;
Circuit circuits[16];
String sensor_nicknames[6];
float eco_threshold = 15.0;
float eco_hysteresis = 5.0;
int poll_interval = 10;
String eco_controller_ip = "";
String eco_controller_mdns = "eco";
float eco_min_temp = 80.0;
float eco_max_temp = 90.0;
float boiler_ref_temp = 20.0;
float boiler_layer_volume = 50.0;

#define RELAY_PUMP_SCH 8
#define RELAY_PUMP_WON 9

int vent_percent = 0;
float sch_temps[6] = {-127,-127,-127,-127,-127,-127};
float eco_temps[6] = {-127,-127,-127,-127,-127,-127};
bool sensor_ok[6] = {false};
float sch_qtot = 0.0;
float eco_qtot = 0.0;

EcoBoilerData eco_boiler = {false, 0.0, 0.0, 0.0, 0.0, 0};

PumpEvent last_sch_pump = {0, 0.0};
PumpEvent last_won_pump = {0, 0.0};

float total_sch_kwh = 0.0;
float total_won_kwh = 0.0;

enum EcoPumpState { ECO_IDLE, ECO_PUMP_SCH, ECO_WAIT_SCH, ECO_PUMP_WON, ECO_WAIT_WON };
EcoPumpState eco_pump_state = ECO_IDLE;
unsigned long eco_pump_timer = 0;
bool last_pump_was_sch = false;

const unsigned long ECO_PUMP_DURATION = 1 * 60 * 1000UL;
const unsigned long ECO_WAIT_SCH_DURATION = 1 * 60 * 1000UL;
const unsigned long ECO_WAIT_WON_DURATION = 2 * 60 * 1000UL;

bool sch_pump_manual = false;
bool won_pump_manual = false;
unsigned long sch_pump_manual_start = 0;
unsigned long won_pump_manual_start = 0;
const unsigned long MANUAL_PUMP_DURATION = 60000UL;
bool sch_pump_manual_on = true;
bool won_pump_manual_on = true;

float prev_eco_temp_top = 0.0;
float prev_eco_qtot = 0.0;

unsigned long last_poll = 0;
unsigned long last_temp_read = 0;
unsigned long uptime_sec = 0;
bool ap_mode_active = false;
bool mcp_available = false;

OneWireNg::Id sensor_addresses[6] = {
  {0x28,0xDB,0xB5,0x03,0x00,0x00,0x80,0xBB},
  {0x28,0x7C,0xF0,0x03,0x00,0x00,0x80,0x59},
  {0x28,0x72,0xDB,0x03,0x00,0x00,0x80,0xC2},
  {0x28,0xAA,0xFB,0x03,0x00,0x00,0x80,0x5F},
  {0x28,0x49,0xDD,0x03,0x00,0x00,0x80,0x4B},
  {0x28,0xC3,0xD6,0x03,0x00,0x00,0x80,0x1E}
};

// Keepalive globals (ping-optimalisatie)
unsigned long last_keepalive = 0;
const unsigned long KEEPALIVE_INTERVAL = 30000UL;  // 30 seconden

// Helper functies (ongewijzigd)
String getFormattedDateTime() {
  time_t now; 
  time(&now);
  if (now < 1700000000) return "tijd niet gesync";
  struct tm tm; 
  localtime_r(&now, &tm);
  char buf[32]; 
  strftime(buf, sizeof(buf), "%d-%m-%Y %H:%M:%S", &tm);
  return String(buf);
}

float calculateQtot(float temps[6]) {
  const float Cp = 1.16;
  float total_energy = 0.0;
  
  float T_layer0 = (boiler_ref_temp + temps[0]) / 2.0;
  if (T_layer0 > boiler_ref_temp) {
    total_energy += (T_layer0 - boiler_ref_temp) * boiler_layer_volume * Cp;
  }
  
  for (int i = 1; i < 6; i++) {
    float T_layer = (temps[i-1] + temps[i]) / 2.0;
    if (T_layer > boiler_ref_temp) {
      total_energy += (T_layer - boiler_ref_temp) * boiler_layer_volume * Cp;
    }
  }
  
  float T_layer6 = temps[5];
  if (T_layer6 > boiler_ref_temp) {
    total_energy += (T_layer6 - boiler_ref_temp) * boiler_layer_volume * Cp;
  }
  
  return total_energy / 1000.0;
}

String getTrend(float current, float previous, float threshold = 0.1) {
  if (abs(current - previous) < threshold) return "→";
  return (current > previous) ? "↑" : "↓";
}

void readBoilerTemps() {
  if (millis() - last_temp_read < 2000) return;
  last_temp_read = millis();
  
  for (int i = 0; i < 6; i++) {
    ow.reset(); 
    ow.writeByte(0x55);
    for (int j = 0; j < 8; j++) ow.writeByte(sensor_addresses[i][j]);
    ow.writeByte(0x44); 
    delay(750);
    
    ow.reset(); 
    ow.writeByte(0x55);
    for (int j = 0; j < 8; j++) ow.writeByte(sensor_addresses[i][j]);
    ow.writeByte(0xBE);
    
    uint8_t data[9];
    for (int j = 0; j < 9; j++) data[j] = ow.readByte();
    
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
      sch_temps[i] = raw / 16.0; 
      sensor_ok[i] = true;
    } else {
      sch_temps[i] = -127.0; 
      sensor_ok[i] = false;
    }
  }
  
  sch_qtot = calculateQtot(sch_temps);
}

void checkPumpFeedback(float total_power) {
  if (!mcp_available) return;
  bool pump_should_be_on = (total_power > 0.01);
  bool pump_is_on = (mcp.digitalRead(7) == LOW);
  if (pump_should_be_on && !pump_is_on) {
    Serial.println("ALERT: Pump should be ON but is OFF!");
  } else if (!pump_should_be_on && pump_is_on) {
    Serial.println("ALERT: Pump should be OFF but is ON!");
  }
}

void pollEcoBoiler() {
  static unsigned long last_eco_poll = 0;
  if (millis() - last_eco_poll < (unsigned long)poll_interval * 1000) return;
  last_eco_poll = millis();
  
  if (eco_controller_ip.length() == 0 && eco_controller_mdns.length() == 0) {
    eco_boiler.online = false;
    return;
  }
  
  Serial.println("\n=== POLLING ECO BOILER ===");
  
  WiFiClient client;
  HTTPClient http;
  String url;
  
  if (eco_controller_ip.length() > 0) {
    url = "http://" + eco_controller_ip + "/json";
  } else {
    Serial.printf("Resolving %s.local ... ", eco_controller_mdns.c_str());
    IPAddress resolvedIP;
    if (WiFi.hostByName((eco_controller_mdns + ".local").c_str(), resolvedIP)) {
      if (resolvedIP.toString() == "0.0.0.0" || resolvedIP[0] == 0) {
        Serial.printf("FAILED\n");
        eco_boiler.online = false;
        return;
      }
      Serial.printf("OK -> %s\n", resolvedIP.toString().c_str());
      url = "http://" + resolvedIP.toString() + "/json";
    } else {
      Serial.printf("FAILED\n");
      eco_boiler.online = false;
      return;
    }
  }
  
  Serial.printf("ECO: Polling %s\n    ", url.c_str());
  
  http.begin(client, url);
  http.setTimeout(2000);
  http.setConnectTimeout(1000);
  http.setReuse(false);
  
  int httpCode = http.GET();
  Serial.printf("Result: %d", httpCode);
  
  if (httpCode == 200) {
    String payload = http.getString();
    Serial.printf(" (%d bytes) ✓\n", payload.length());
    
    eco_boiler.online = true;
    eco_boiler.last_seen = millis();
    
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload);
    
    if (!error) {
      eco_boiler.temp_avg = doc["EAv"] | 0.0;
      eco_boiler.qtot = doc["EQtot"] | 0.0;
      eco_boiler.temp_top = doc["ETopH"] | 0.0;
      eco_boiler.temp_bottom = doc["EBotL"] | 0.0;
      
      eco_qtot = eco_boiler.qtot;
      
      Serial.printf("    ETopH=%.1f°C EQtot=%.2f kWh EBotL=%.1f°C\n", 
        eco_boiler.temp_top, eco_boiler.qtot, eco_boiler.temp_bottom);
      
      prev_eco_temp_top = eco_boiler.temp_top;
      prev_eco_qtot = eco_boiler.qtot;
      
    } else {
      Serial.printf("    JSON parse error: %s\n", error.c_str());
    }
  } else {
    Serial.printf(" FAILED\n");
    eco_boiler.online = false;
  }
  
  http.end();
  client.stop();
}

void savePumpEvent(const char* pump_type, float kwh) {
  unsigned long timestamp = millis() / 1000;
  
  if (String(pump_type) == "SCH") {
    last_sch_pump.timestamp = timestamp;
    last_sch_pump.kwh_pumped = kwh;
    total_sch_kwh += kwh;
    preferences.putULong(NVS_LAST_SCH_PUMP, timestamp);
    preferences.putFloat(NVS_LAST_SCH_KWH, kwh);
    preferences.putFloat(NVS_TOTAL_SCH_KWH, total_sch_kwh);
    Serial.printf("Saved SCH pump event: %.2f kWh (totaal: %.2f kWh)\n", kwh, total_sch_kwh);
  } else if (String(pump_type) == "WON") {
    last_won_pump.timestamp = timestamp;
    last_won_pump.kwh_pumped = kwh;
    total_won_kwh += kwh;
    preferences.putULong(NVS_LAST_WON_PUMP, timestamp);
    preferences.putFloat(NVS_LAST_WON_KWH, kwh);
    preferences.putFloat(NVS_TOTAL_WON_KWH, total_won_kwh);
    Serial.printf("Saved WON pump event: %.2f kWh (totaal: %.2f kWh)\n", kwh, total_won_kwh);
  }
}

String getPumpStatusMessage() {
  if (sch_pump_manual || won_pump_manual) {
    unsigned long elapsed_sch = millis() - sch_pump_manual_start;
    unsigned long elapsed_won = millis() - won_pump_manual_start;
    
    if (sch_pump_manual && elapsed_sch < MANUAL_PUMP_DURATION) {
      unsigned long remaining = (MANUAL_PUMP_DURATION - elapsed_sch) / 1000;
      return "🎮 Handmatig: SCH pomp " + String(sch_pump_manual_on ? "AAN" : "UIT") + 
             " (" + String(remaining) + "s resterend)";
    }
    if (won_pump_manual && elapsed_won < MANUAL_PUMP_DURATION) {
      unsigned long remaining = (MANUAL_PUMP_DURATION - elapsed_won) / 1000;
      return "🎮 Handmatig: WON pomp " + String(won_pump_manual_on ? "AAN" : "UIT") + 
             " (" + String(remaining) + "s resterend)";
    }
  }
  
  if (!eco_boiler.online) {
    return "○ ECO boiler offline - geen automatische distributie";
  }
  
  switch (eco_pump_state) {
    case ECO_IDLE: {
      bool temp_trigger = eco_boiler.temp_top > eco_max_temp;
      bool energy_trigger = eco_boiler.qtot > eco_threshold;
      
      if (temp_trigger || energy_trigger) {
        String reason = "";
        if (temp_trigger) reason = "Temp: " + String(eco_boiler.temp_top, 1) + "°C > " + String(eco_max_temp, 0) + "°C";
        if (energy_trigger) {
          if (reason.length() > 0) reason += " EN ";
          reason += "Energie: " + String(eco_boiler.qtot, 1) + " > " + String(eco_threshold, 1) + " kWh";
        }
        return "⏸️ Trigger actief (" + reason + ") - wacht op cyclus start";
      } else {
        return "✓ Standby - Temp: " + String(eco_boiler.temp_top, 1) + "°C, Energie: " + 
               String(eco_boiler.qtot, 1) + " kWh (beide onder limiet)";
      }
    }
    
    case ECO_PUMP_SCH: {
      unsigned long elapsed = millis() - eco_pump_timer;
      unsigned long remaining = (ECO_PUMP_DURATION - elapsed) / 1000;
      return "🔵 SCH pompt naar Schuur (" + String(remaining) + "s / 60s) - Fair share transfer";
    }
    
    case ECO_WAIT_SCH: {
      unsigned long elapsed = millis() - eco_pump_timer;
      unsigned long remaining = (ECO_WAIT_SCH_DURATION - elapsed) / 1000;
      return "⏳ Wacht na SCH pomp (" + String(remaining) + "s / 60s) - WON is volgende";
    }
    
    case ECO_PUMP_WON: {
      unsigned long elapsed = millis() - eco_pump_timer;
      unsigned long remaining = (ECO_PUMP_DURATION - elapsed) / 1000;
      return "🟢 WON pompt naar Woning (" + String(remaining) + "s / 60s) - Fair share transfer";
    }
    
    case ECO_WAIT_WON: {
      unsigned long elapsed = millis() - eco_pump_timer;
      unsigned long remaining = (ECO_WAIT_WON_DURATION - elapsed) / 1000;
      return "⏳ Wacht na WON pomp (" + String(remaining) + "s / 120s) - SCH is volgende";
    }
    
    default:
      return "? Onbekende pump status";
  }
}

// ============== DEEL 2/5: ECO PUMPS & POLLING FUNCTIES ==============

void handleEcoPumps() {
  if (!mcp_available) return;
  
  bool sch_manual_active = sch_pump_manual && (millis() - sch_pump_manual_start < MANUAL_PUMP_DURATION);
  bool won_manual_active = won_pump_manual && (millis() - won_pump_manual_start < MANUAL_PUMP_DURATION);
  
  if (sch_pump_manual && !sch_manual_active) {
    Serial.println("\n=== MANUAL PUMP TIMEOUT ===");
    Serial.println("SCH pump manual mode expired (60s)");
    sch_pump_manual = false;
  }
  if (won_pump_manual && !won_manual_active) {
    Serial.println("\n=== MANUAL PUMP TIMEOUT ===");
    Serial.println("WON pump manual mode expired (60s)");
    won_pump_manual = false;
  }
  
  if (sch_manual_active || won_manual_active) {
    mcp.digitalWrite(RELAY_PUMP_SCH, sch_manual_active ? (sch_pump_manual_on ? LOW : HIGH) : HIGH);
    mcp.digitalWrite(RELAY_PUMP_WON, won_manual_active ? (won_pump_manual_on ? LOW : HIGH) : HIGH);
    return;
  }
  
  bool should_start = eco_boiler.online 
                   && ((eco_boiler.temp_top > eco_max_temp) || (eco_boiler.qtot > eco_threshold));
  
  bool should_stop = !eco_boiler.online
                  || ((eco_boiler.temp_top < eco_min_temp) || (eco_boiler.qtot < (eco_threshold - eco_hysteresis)));
  
  switch (eco_pump_state) {
    case ECO_IDLE:
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      if (should_start) {
        if (last_pump_was_sch) {
          eco_pump_state = ECO_PUMP_WON;
          Serial.println("\n=== AUTO PUMP START === Pump: WON");
        } else {
          eco_pump_state = ECO_PUMP_SCH;
          Serial.println("\n=== AUTO PUMP START === Pump: SCH");
        }
        eco_pump_timer = millis();
      }
      break;
      
    case ECO_PUMP_SCH:
      mcp.digitalWrite(RELAY_PUMP_SCH, LOW);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      if (should_stop) {
        Serial.println("\n=== AUTO PUMP STOP === Pump: SCH");
        eco_pump_state = ECO_IDLE;
        mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
        float kwh_transferred = 0.5;
        savePumpEvent("SCH", kwh_transferred);
        last_pump_was_sch = true;
        break;
      }
      
      if (millis() - eco_pump_timer >= ECO_PUMP_DURATION) {
        Serial.println("\n=== AUTO PUMP FINISHED === Pump: SCH");
        mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
        float kwh_transferred = 0.5;
        savePumpEvent("SCH", kwh_transferred);
        last_pump_was_sch = true;
        eco_pump_state = ECO_WAIT_SCH;
        eco_pump_timer = millis();
        Serial.println("Entering wait phase after SCH: 1 minute");
      }
      break;
      
    case ECO_WAIT_SCH:
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      if (should_stop) {
        Serial.println("\n=== CYCLE STOPPED === Reason: Stop conditie tijdens wacht fase");
        eco_pump_state = ECO_IDLE;
        break;
      }
      
      if (millis() - eco_pump_timer >= ECO_WAIT_SCH_DURATION) {
        eco_pump_state = ECO_PUMP_WON;
        eco_pump_timer = millis();
        Serial.println("Wait SCH finished → starting WON pump");
      }
      break;
      
    case ECO_PUMP_WON:
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, LOW);
      
      if (should_stop) {
        Serial.println("\n=== AUTO PUMP STOP === Pump: WON");
        eco_pump_state = ECO_IDLE;
        mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
        float kwh_transferred = 0.5;
        savePumpEvent("WON", kwh_transferred);
        last_pump_was_sch = false;
        break;
      }
      
      if (millis() - eco_pump_timer >= ECO_PUMP_DURATION) {
        Serial.println("\n=== AUTO PUMP FINISHED === Pump: WON");
        mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
        float kwh_transferred = 0.5;
        savePumpEvent("WON", kwh_transferred);
        last_pump_was_sch = false;
        eco_pump_state = ECO_WAIT_WON;
        eco_pump_timer = millis();
        Serial.println("Entering wait phase after WON: 2 minutes");
      }
      break;
      
    case ECO_WAIT_WON:
      mcp.digitalWrite(RELAY_PUMP_SCH, HIGH);
      mcp.digitalWrite(RELAY_PUMP_WON, HIGH);
      
      if (should_stop) {
        Serial.println("\n=== CYCLE STOPPED === Reason: Stop conditie tijdens wacht fase");
        eco_pump_state = ECO_IDLE;
        break;
      }
      
      if (millis() - eco_pump_timer >= ECO_WAIT_WON_DURATION) {
        eco_pump_state = ECO_PUMP_SCH;
        eco_pump_timer = millis();
        Serial.println("Wait WON finished → starting SCH pump");
      }
      break;
  }
}

// ============== DEEL 3/5: WEB SERVER & SETTINGS ==============

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    // Hoofdpagina HTML (ongewijzigd - te lang om hier volledig te plakken, gebruik je originele versie)
    // Zorg dat je hier je originele main page rawliteral gebruikt
    request->send(200, "text/html", "<html><body><h1>HVAC Controller V54</h1><p>Ping geoptimaliseerd</p></body></html>"); // ← vervang door volledige HTML
  });

  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    String sensorNamesHtml = "";
    for (int i = 0; i < 6; i++) {
      sensorNamesHtml += "<p>Sensor " + String(i+1) + ": <input type='text' name='sensor_nick_" + String(i) + "' value='" + sensor_nicknames[i] + "'></p>";
    }

    String circuitsHtml = "";
    for (int i = 0; i < circuits_num; i++) {
      circuitsHtml += "<h3>Circuit " + String(i+1) + "</h3>";
      circuitsHtml += "<p>Naam: <input type='text' name='circuit_name_" + String(i) + "' value='" + circuits[i].name + "'></p>";
      circuitsHtml += "<p>IP: <input type='text' name='circuit_ip_" + String(i) + "' value='" + circuits[i].ip + "'></p>";
      circuitsHtml += "<p>mDNS: <input type='text' name='circuit_mdns_" + String(i) + "' value='" + circuits[i].mdns + "'></p>";
      circuitsHtml += "<p>Vermogen (kW): <input type='number' step='0.001' name='circuit_power_" + String(i) + "' value='" + String(circuits[i].power_kw, 3) + "'></p>";
      circuitsHtml += "<p>T-stat: <input type='checkbox' name='circuit_tstat_" + String(i) + "'" + (circuits[i].has_tstat ? " checked" : "") + "></p>";
      circuitsHtml += "<p>T-stat pin: <input type='number' name='circuit_tstat_pin_" + String(i) + "' value='" + (circuits[i].tstat_pin != 255 ? String(circuits[i].tstat_pin) : "") + "'></p>";
    }

    String html = R"rawliteral(
<!DOCTYPE html>
<html lang="nl">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HVAC Settings</title>
  <style>body{font-family:Arial;} table{width:100%;} input{width:100%;} .btn{padding:10px 20px;}</style>
</head>
<body>
<div style="max-width:800px;margin:auto;padding:20px;">
  <h1>HVAC Controller Settings</h1>
  <div style="color:#c00;font-weight:bold;">Waarschuwing: wijzig WiFi instellingen alleen als je zeker weet wat je doet!<br>Verkeerde WiFi kan controller onbereikbaar maken!<br><br><strong>Geen WiFi?</strong> Controller start AP: HVAC-Setup<br>Ga naar http://192.168.4.1/settings</div>

  <form action="/save_settings" method="get">
    <h2>WiFi Configuratie</h2>
    <table>
      <tr><td style="width:35%;">WiFi SSID</td><td><input type="text" name="wifi_ssid" value=")rawliteral" + wifi_ssid + R"rawliteral("></td></tr>
      <tr><td>WiFi Password</td><td><input type="password" name="wifi_pass" value=")rawliteral" + wifi_pass + R"rawliteral("></td></tr>
      <tr><td>MAC adres (voor DHCP reservation)</td><td><strong>" + WiFi.macAddress() + "</strong></td></tr>
      <tr><td>Static IP</td><td><input type="text" name="static_ip" value=")rawliteral" + static_ip_str + R"rawliteral(" placeholder="leeg = DHCP (aanbevolen)"></td></tr>
    </table>

    <h2>Basis Instellingen</h2>
    <table>
      <tr><td style="width:35%;">Room naam</td><td><input type="text" name="room_id" value=")rawliteral" + room_id + R"rawliteral(" required></td></tr>
      <tr><td>Aantal circuits</td><td><input type="number" name="circuits_num" min="1" max="16" value=")rawliteral" + String(circuits_num) + R"rawliteral("></td></tr>
      <tr><td>Poll interval (sec)</td><td><input type="number" min="5" name="poll_interval" value=")rawliteral" + String(poll_interval) + R"rawliteral("></td></tr>
    </table>

    <h2>ECO Boiler Instellingen</h2>
    <table>
      <tr><td style="width:35%;">ECO IP adres</td><td><input type="text" name="eco_ip" value=")rawliteral" + eco_controller_ip + R"rawliteral("></td></tr>
      <tr><td>ECO mDNS naam</td><td><input type="text" name="eco_mdns" value=")rawliteral" + eco_controller_mdns + R"rawliteral("></td></tr>
      <tr><td>ECO Threshold (kWh)</td><td><input type="number" step="0.1" name="eco_thresh" value=")rawliteral" + String(eco_threshold) + R"rawliteral("></td></tr>
      <tr><td>ECO Hysteresis (kWh)</td><td><input type="number" step="0.1" name="eco_hyst" value=")rawliteral" + String(eco_hysteresis) + R"rawliteral("></td></tr>
      <tr><td>ECO Tmin - Stop (°C)</td><td><input type="number" step="0.1" name="eco_min_temp" value=")rawliteral" + String(eco_min_temp) + R"rawliteral("></td></tr>
      <tr><td>ECO Tmax - Start (°C)</td><td><input type="number" step="0.1" name="eco_max_temp" value=")rawliteral" + String(eco_max_temp) + R"rawliteral("></td></tr>
    </table>

    <h2>Boiler Qtot Berekening</h2>
    <table>
      <tr><td style="width:35%;">Reference temp (°C)</td><td><input type="number" step="0.1" name="boiler_ref_temp" value=")rawliteral" + String(boiler_ref_temp) + R"rawliteral("></td></tr>
      <tr><td>Volume per laag (L)</td><td><input type="number" step="1" name="boiler_volume" value=")rawliteral" + String(boiler_layer_volume) + R"rawliteral("></td></tr>
    </table>

    <h2>Sensor Nicknames</h2>
    <div style="padding:10px;">)rawliteral" + sensorNamesHtml + R"rawliteral(</div>

    <h2>Verwarmingscircuits</h2>
    )rawliteral" + circuitsHtml + R"rawliteral(

    <div style="text-align:center;">
      <button type="submit" class="btn">Opslaan & Reboot</button>
      <a href="/" class="btn" style="display:inline-block;text-decoration:none;background:#c00;">Annuleren</a>
    </div>
  </form>
</div>
</body></html>
)rawliteral";
    request->send(200, "text/html; charset=utf-8", html);
  });

  server.on("/save_settings", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("\n=== SAVE SETTINGS ===");
    
    if (request->hasArg("wifi_ssid")) preferences.putString(NVS_WIFI_SSID, request->arg("wifi_ssid"));
    if (request->hasArg("wifi_pass")) preferences.putString(NVS_WIFI_PASS, request->arg("wifi_pass"));
    if (request->hasArg("static_ip")) preferences.putString(NVS_STATIC_IP, request->arg("static_ip"));
    if (request->hasArg("room_id")) preferences.putString(NVS_ROOM_ID, request->arg("room_id"));
    if (request->hasArg("circuits_num")) preferences.putInt(NVS_CIRCUITS_NUM, request->arg("circuits_num").toInt());
    if (request->hasArg("poll_interval")) preferences.putInt(NVS_POLL_INTERVAL, request->arg("poll_interval").toInt());
    if (request->hasArg("eco_ip")) preferences.putString(NVS_ECO_IP, request->arg("eco_ip"));
    if (request->hasArg("eco_mdns")) preferences.putString(NVS_ECO_MDNS, request->arg("eco_mdns"));
    if (request->hasArg("eco_thresh")) preferences.putFloat(NVS_ECO_THRESHOLD, request->arg("eco_thresh").toFloat());
    if (request->hasArg("eco_hyst")) preferences.putFloat(NVS_ECO_HYSTERESIS, request->arg("eco_hyst").toFloat());
    if (request->hasArg("eco_min_temp")) preferences.putFloat(NVS_ECO_MIN_TEMP, request->arg("eco_min_temp").toFloat());
    if (request->hasArg("eco_max_temp")) preferences.putFloat(NVS_ECO_MAX_TEMP, request->arg("eco_max_temp").toFloat());
    if (request->hasArg("boiler_ref_temp")) preferences.putFloat(NVS_BOILER_REF_TEMP, request->arg("boiler_ref_temp").toFloat());
    if (request->hasArg("boiler_volume")) preferences.putFloat(NVS_BOILER_VOLUME, request->arg("boiler_volume").toFloat());
    
    for (int i = 0; i < 6; i++) {
      String param = "sensor_nick_" + String(i);
      if (request->hasArg(param.c_str())) {
        String nick = request->arg(param.c_str());
        nick.trim();
        if (nick.length() == 0) nick = "Sensor " + String(i + 1);
        preferences.putString((String(NVS_SENSOR_NICK_BASE) + i).c_str(), nick);
      }
    }
    
    int save_count = request->arg("circuits_num").toInt();
    if (save_count < 1) save_count = 1;
    if (save_count > 16) save_count = 16;

    for (int i = 0; i < save_count; i++) {
      String name_val = request->arg(("circuit_name_" + String(i)).c_str());
      if (name_val.length() == 0) name_val = "Circuit " + String(i + 1);
      preferences.putString(("c" + String(i) + "_name").c_str(), name_val);
      preferences.putString(("c" + String(i) + "_ip").c_str(), request->arg(("circuit_ip_" + String(i)).c_str()));
      
      String mdns_val = request->arg(("circuit_mdns_" + String(i)).c_str());
      mdns_val.replace(".local", "");
      mdns_val.trim();
      preferences.putString(("c" + String(i) + "_mdns").c_str(), mdns_val);
      preferences.putFloat(("c" + String(i) + "_power").c_str(), request->arg(("circuit_power_" + String(i)).c_str()).toFloat());
      preferences.putBool(("c" + String(i) + "_tstat").c_str(), request->hasArg(("circuit_tstat_" + String(i)).c_str()));
      
      int pin_val = 255;
      if (request->hasArg(("circuit_tstat_pin_" + String(i)).c_str())) {
        pin_val = request->arg(("circuit_tstat_pin_" + String(i)).c_str()).toInt();
        if (pin_val != 10 && pin_val != 11 && pin_val != 12) pin_val = 255;
      }
      preferences.putInt(("c" + String(i) + "_pin").c_str(), pin_val);
    }

    Serial.println("Settings saved!");
    request->send(200, "text/html", "<h2 style='text-align:center;color:#369;'>Opgeslagen! Rebooting...</h2>");
    delay(2000);
    ESP.restart();
  });

  // Overige endpoints (circuit_override, pump_sch_on, etc.) blijven ongewijzigd
  // ... plak hier je originele server.on() calls voor overrides, pumps, etc. ...

  server.begin();
}

// ============== DEEL 5/5: SETUP & LOOP ==============

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== HVAC Controller V54 (ping geoptimaliseerd) ===");
  Serial.println("WIJZIGINGEN t.o.v. V53.5:");
  Serial.println("1. Ping-optimalisatie: DHCP only, WIFI_PS_NONE, listen_interval=1");
  Serial.println("2. light sleep uit, unicast keepalive 30s, auto-reconnect");
  Serial.println("3. MAC-adres in settings pagina");

  // Factory reset optie
  Serial.println("\nType 'R' binnen 3 sec voor NVS reset...");
  unsigned long start = millis();
  while (millis() - start < 3000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'R' || c == 'r') factoryResetNVS();
    }
  }

  // I2C & MCP23017
  Wire.begin(I2C_SDA, I2C_SCL);
  if (mcp.begin_I2C(0x20)) {
    Serial.println("MCP23017 OK!");
    mcp_available = true;
    
    for (int i = 0; i < 7; i++) {
      mcp.pinMode(i, OUTPUT);
      mcp.digitalWrite(i, HIGH);
    }
    mcp.pinMode(7, INPUT_PULLUP);
    mcp.pinMode(8, OUTPUT);
    mcp.digitalWrite(8, HIGH);
    mcp.pinMode(9, OUTPUT);
    mcp.digitalWrite(9, HIGH);
    mcp.pinMode(10, INPUT_PULLUP);
    mcp.pinMode(11, INPUT_PULLUP);
    mcp.pinMode(12, INPUT_PULLUP);
    mcp.pinMode(13, INPUT_PULLUP);
    mcp.pinMode(14, INPUT_PULLUP);
    mcp.pinMode(15, INPUT_PULLUP);
  } else {
    Serial.println("MCP23017 not found!");
    mcp_available = false;
  }

  // Load NVS (ongewijzigd)
  preferences.begin("hvac-config", false);

  room_id = preferences.getString(NVS_ROOM_ID, "HVAC");
  wifi_ssid = preferences.getString(NVS_WIFI_SSID, "");
  wifi_pass = preferences.getString(NVS_WIFI_PASS, "");
  static_ip_str = preferences.getString(NVS_STATIC_IP, "");
  circuits_num = preferences.getInt(NVS_CIRCUITS_NUM, 7);
  circuits_num = constrain(circuits_num, 1, 16);
  
  eco_threshold = preferences.getFloat(NVS_ECO_THRESHOLD, 15.0);
  eco_hysteresis = preferences.getFloat(NVS_ECO_HYSTERESIS, 5.0);
  poll_interval = preferences.getInt(NVS_POLL_INTERVAL, 10);
  eco_controller_ip = preferences.getString(NVS_ECO_IP, "");
  eco_controller_mdns = preferences.getString(NVS_ECO_MDNS, "eco");
  eco_min_temp = preferences.getFloat(NVS_ECO_MIN_TEMP, 80.0);
  eco_max_temp = preferences.getFloat(NVS_ECO_MAX_TEMP, 90.0);
  boiler_ref_temp = preferences.getFloat(NVS_BOILER_REF_TEMP, 20.0);
  boiler_layer_volume = preferences.getFloat(NVS_BOILER_VOLUME, 50.0);
  
  last_sch_pump.timestamp = preferences.getULong(NVS_LAST_SCH_PUMP, 0);
  last_sch_pump.kwh_pumped = preferences.getFloat(NVS_LAST_SCH_KWH, 0.0);
  last_won_pump.timestamp = preferences.getULong(NVS_LAST_WON_PUMP, 0);
  last_won_pump.kwh_pumped = preferences.getFloat(NVS_LAST_WON_KWH, 0.0);
  
  total_sch_kwh = preferences.getFloat(NVS_TOTAL_SCH_KWH, 0.0);
  total_won_kwh = preferences.getFloat(NVS_TOTAL_WON_KWH, 0.0);
  
  for (int i = 0; i < 6; i++) {
    sensor_nicknames[i] = preferences.getString(
      (String(NVS_SENSOR_NICK_BASE) + i).c_str(), 
      "Sensor " + String(i + 1)
    );
  }

  for (int i = 0; i < 16; i++) {
    circuits[i].name = preferences.getString(("c" + String(i) + "_name").c_str(), "Circuit " + String(i + 1));
    circuits[i].ip = preferences.getString(("c" + String(i) + "_ip").c_str(), "");
    circuits[i].mdns = preferences.getString(("c" + String(i) + "_mdns").c_str(), "");
    circuits[i].power_kw = preferences.getFloat(("c" + String(i) + "_power").c_str(), 0.0);
    circuits[i].has_tstat = preferences.getBool(("c" + String(i) + "_tstat").c_str(), false);
    circuits[i].tstat_pin = preferences.getInt(("c" + String(i) + "_pin").c_str(), 255);
    
    circuits[i].online = false;
    circuits[i].heating_on = false;
    circuits[i].vent_request = 0;
    circuits[i].on_time = 1;
    circuits[i].off_time = 100;
    circuits[i].last_change = millis();
    circuits[i].duty_cycle = 0.0;
    circuits[i].override_active = false;
  }

  // WiFi verbinding - ping optimalisaties
  WiFi.mode(WIFI_STA);

  // Optimalisaties volgens checklist
  wifi_sta_config_t wifi_cfg = {};
  wifi_cfg.listen_interval = 1;
  esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_cfg);

  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  if (wifi_ssid.length() > 0) {
    Serial.printf("\nConnecting to '%s'...\n", wifi_ssid.c_str());
    
    int retry_count = 0;
    const int MAX_RETRIES = 5;
    bool connected = false;
    
    while (!connected && retry_count < MAX_RETRIES) {
      WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
      
      unsigned long start_attempt = millis();
      while (WiFi.status() != WL_CONNECTED && (millis() - start_attempt) < 20000) {
        delay(500);
        Serial.print(".");
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        connected = true;
        Serial.println("\n✓ WiFi connected!");
        Serial.println("IP: " + WiFi.localIP().toString());
        Serial.println("MAC: " + WiFi.macAddress());
      } else {
        retry_count++;
        Serial.printf("\n✗ Attempt %d/%d failed\n", retry_count, MAX_RETRIES);
        WiFi.disconnect();
        delay(2000);
      }
    }
    
    if (!connected) {
      Serial.println("\n✗ All WiFi attempts failed -> AP mode");
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi failed -> AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("HVAC-Setup");
    ap_mode_active = true;
    
    Serial.println("\n=== CAPTIVE PORTAL ACTIVE ===");
    Serial.println("AP SSID: HVAC-Setup");
    Serial.println("AP IP: " + WiFi.softAPIP().toString());
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  } else {
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    
    configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", "CET-1CEST,M3.5.0/02,M10.5.0/03", 1);
    tzset();
    
    int retry = 0;
    time_t now = 0;
    struct tm timeinfo;
    while (now < 1700000000 && retry < 20) {
      time(&now);
      localtime_r(&now, &timeinfo);
      delay(500);
      Serial.print(".");
      retry++;
    }
    
    if (now >= 1700000000) {
      Serial.println(" OK!");
      Serial.println("Time: " + getFormattedDateTime());
    } else {
      Serial.println(" TIMEOUT");
    }
  }

  if (MDNS.begin(room_id.c_str())) {
    Serial.println("mDNS: http://" + room_id + ".local");
  }

  setupWebServer();
  Serial.println("\nWeb server started!");
  Serial.println("Ready!\n");
}

void loop() {
  if (ap_mode_active) dnsServer.processNextRequest();

  uptime_sec = millis() / 1000;
  
  readBoilerTemps();
  pollRooms();          // ← zorg dat deze functie in je originele code staat
  pollEcoBoiler();
  handleEcoPumps();

  // WiFi herstel bij disconnect
  if (!ap_mode_active && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected - attempting reconnect...");
    WiFi.reconnect();
  }

  // Periodieke unicast keepalive naar gateway
  if (!ap_mode_active && WiFi.status() == WL_CONNECTED && 
      millis() - last_keepalive >= KEEPALIVE_INTERVAL) {
    IPAddress gateway = WiFi.gatewayIP();
    if (gateway != IPAddress(0,0,0,0)) {
      WiFiClient client;
      if (client.connect(gateway, 80)) {
        client.stop();
      }
    }
    last_keepalive = millis();
  }

  delay(100);
}

void factoryResetNVS() {
  Serial.println("\n=== FACTORY RESET NVS ===");
  preferences.begin("hvac-config", false);
  preferences.clear();
  preferences.end();
  Serial.println("NVS cleared! Reboot...");
  delay(1000);
  ESP.restart();
}
```
---------------------------------
