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

Hieronder twee sketches die dit toepassen, om te integreren in beide System sketches:

/*************************************************
 * ESP32C6_HVACTEST – Always Online Network Profile
 * Ping-optimalisatie toegepast
 *************************************************/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <esp_pm.h>

const char* ssid     = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* hostname = "hvac";

WebServer server(80);

// ---------- KEEPALIVE ----------
unsigned long lastKeepAlive = 0;
const unsigned long KEEPALIVE_INTERVAL = 45000;

// ---------- WIFI ----------
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);

  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_pm_config_t pm_config = {
    .max_freq_mhz = 160,
    .min_freq_mhz = 160,
    .light_sleep_enable = false
  };
  esp_pm_configure(&pm_config);

  WiFi.begin(ssid, password);
}

// ---------- KEEPALIVE ----------
void networkKeepAlive() {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClient client;
  client.setTimeout(200);
  client.connect(WiFi.gatewayIP(), 80);
  client.stop();
}

// ---------- RECONNECT ----------
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(ssid, password);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  setupWiFi();

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }

  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  esp_netif_action_connected(netif, NULL, 0, NULL);

  MDNS.begin(hostname);

  server.on("/", []() {
    server.send(200, "text/plain", "HVAC controller online");
  });

  server.begin();
}

// ---------- LOOP ----------
void loop() {
  server.handleClient();
  ensureWiFi();

  unsigned long now = millis();
  if (now - lastKeepAlive > KEEPALIVE_INTERVAL) {
    lastKeepAlive = now;
    networkKeepAlive();
  }
}

-----------------------------------------------------

/*************************************************
 * ESP32C6_ECO-Boiler – Always Online Network Profile
 * Ping-optimalisatie toegepast
 *************************************************/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <esp_pm.h>

const char* ssid     = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* hostname = "eco";

WebServer server(80);

// ---------- KEEPALIVE ----------
unsigned long lastKeepAlive = 0;
const unsigned long KEEPALIVE_INTERVAL = 45000;

// ---------- WIFI ----------
void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(hostname);

  esp_wifi_set_ps(WIFI_PS_NONE);

  esp_pm_config_t pm_config = {
    .max_freq_mhz = 160,
    .min_freq_mhz = 160,
    .light_sleep_enable = false
  };
  esp_pm_configure(&pm_config);

  WiFi.begin(ssid, password);
}

// ---------- KEEPALIVE ----------
void networkKeepAlive() {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClient client;
  client.setTimeout(200);
  client.connect(WiFi.gatewayIP(), 80);
  client.stop();
}

// ---------- RECONNECT ----------
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;

  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(ssid, password);
}

// ---------- SETUP ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  setupWiFi();

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }

  esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  esp_netif_action_connected(netif, NULL, 0, NULL);

  MDNS.begin(hostname);

  server.on("/", []() {
    server.send(200, "text/plain", "ECO boiler controller online");
  });

  server.begin();
}

// ---------- LOOP ----------
void loop() {
  server.handleClient();
  ensureWiFi();

  unsigned long now = millis();
  if (now - lastKeepAlive > KEEPALIVE_INTERVAL) {
    lastKeepAlive = now;
    networkKeepAlive();
  }
}


----------------------------------------------------------------------------

