# ESP32C6_Experiments

Losse test- en proefsketches voor de ESP32-C6. **Hier horen experimenten thuis, niet in de productiesketches** (zie de ROOM-standaard in `ESP32C6_ROOMS`). Elke sketch staat in een eigen map met dezelfde naam, zodat de Arduino IDE hem rechtstreeks opent.

| Map | Wat |
|---|---|
| `ESP32C6_TEST/` | Pintest op een Freenove-breakoutbord met ledjes per pin (30- of 32-pinsmodule), met WiFi-scan en flashinfo. Voorbeeldrapport hieronder |
| `ESP32-C6_PIN_SELFTEST/` | Batchtest van alle GPIO's (v7, zonder GPIO16/17: USB-serial) |
| `ESP32_WIFI-Test/` | Eenvoudige WiFi-test |
| `ESP32_DS18B20_TEST/` | Proof of concept DS18B20 op de ESP32-C6 (de gewone OneWire-bibliotheek werkte niet) |
| `ESP32C6_DS18B20_ID-scanner/` | Welke DS18B20's hangen er op de bus? Toont de adressen |
| `Pixel_sequence_Test/` | Pixelvolgorde bepalen van een matrix in "serpentine"-volgorde |
| `docs/ESP32-C6_ping-optimalisatie.md` | Notities over ping- en WiFi-optimalisatie |
| `old/` | Matter-simulaties (ECO, HVAC, ROOM) en het Matter-stappenplan: niet meer in gebruik sinds Matter van de ESP32's verdween (nov 2026) |

---

## ESP32C6_TEST

`ESP32C6_TEST.ino` = Pin Test voor controller ESP32-C6 gemonteerd op een "Freenove Breakout Board" met ledjes per pin.

Beide types ESP32-C6 controllers kunnen getest worden.
In serial monitor: Typ '30' of '32' voor board versie. Daarna Typ aantal seconden per pin (1-20).
=> Alle pins worden rondom getest.

----------

* Voorbeeldrapport voor 32-pin module:

=== ESP32-C6 Volledige Test – Freenove Board ===
Typ '30' of '32' voor board versie:
32-pin gekozen

Typ aantal seconden per pin (1-20):
Elke pin knippert 2 seconden

--- WiFi scan ---
5 netwerken gevonden:
  Delannoy (RSSI: -58 dBm)
  SMA1930040367 (RSSI: -74 dBm)
  telenet-24 (RSSI: -85 dBm)
  Proximus Public Wi-Fi (RSSI: -91 dBm)
  WiFi-2.4-2960 (RSSI: -93 dBm)

--- Flash info ---
Flash grootte: 16384 KB
Vrije sketch space: 1280 KB

--- Chip info ---
Chip model: ESP32-C6 (revision 2)
CPU frequentie: 160 MHz
MAC address: 58:8C:81:32:

----------
* Voorbeeldrapport voor 30-pin module:



-----

Ter informatie: Dit zijn de twee pin configuraties:

"	30-pin RECHTS	30-pin LINKS	
R12	4	2	L14
R11	5	3	L13
R10	6	16	L12
R09	7	17	L11
R08	0	15	L10
R07	1	23	L09
R06	8	22	L08
R05	10	21	L07
R04	11	20	L06
R03	12	19	L05
R02	13	18	L04
		9	L03"

"	32-pin RECHTS	32-pin LINKS	
R12	4	16	L14
R11	5	17	L13
R10	6	15	L12
R09	7	23	L11
R08	0	22	L10
R07	1	21	L09
R06	8	20	L08
R05	10	19	L07
R04	11	18	L06
R03	2	9	L05
R02	3	13	L03
		12	L02"

