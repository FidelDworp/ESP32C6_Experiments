# Stappenplan: Matter/HomeKit integratie — Zarlar thuisautomatisering
**Sketches:** `ESP32_C6_MATTER_ROOM_v22.ino` · `ESP32_C6_HVAC_SIM.ino` · `ESP32_C6_ECO_BOILER.ino`
**Datum:** 01 mrt 2026 · **API:** arduino-esp32 3.3.2 · **Hardware:** ESP32-C6 16MB

---

## 1. Bestanden plaatsen

Elke sketch heeft een **eigen schetsmap** met twee bestanden:

| Schetsmap | Sketch | Partitietabel |
|---|---|---|
| `ESP32_C6_MATTER_ROOM/` | `ESP32_C6_MATTER_ROOM_v22.ino` | `partitions_16mb.csv` |
| `ESP32_C6_HVAC/` | `ESP32_C6_HVAC_SIM.ino` | `partitions_16mb.csv` |
| `ESP32_C6_ECO/` | `ESP32_C6_ECO_BOILER.ino` | `partitions_16mb.csv` |

⚠️ Arduino IDE **vereist** dat sketch en CSV in exact dezelfde map zitten. Kopieer de `partitions_16mb.csv` naar elke schetsmap apart — elk project heeft zijn eigen kopie nodig.

---

## 2. Arduino IDE instellen (per controller herhalen)

- **Board:** `ESP32C6 Dev Module` (of jouw C6-variant)
- **Partition Scheme:** `Custom` → selecteer de `partitions_16mb.csv` **uit de schetsmap**

> **Hoe custom CSV selecteren:**
> Kies in de dropdown "Custom" — Arduino IDE zoekt dan automatisch naar een `partitions_16mb.csv` in de schetsmap. Het bestand moet exact die naam hebben.

- **Flash Size:** `16MB`
- **Upload Speed / CPU Freq:** zoals je die al had voor TESTROOM

---

## 3. Matter transport instelling

Alle drie de sketches hebben een `matter_transport` instelling (`0 = WiFi`, `1 = Thread`).

**Laat deze altijd op WiFi (0) staan.**

Thread is een laag-energie mesh-protocol waarbij apparaten geen rechtstreekse WiFi-verbinding nodig hebben — ze communiceren via een **Thread Border Router** (Apple TV 4K gen3+ of HomePod 2e gen). De ESP32-C6 heeft de benodigde 802.15.4 radio ingebouwd, maar **arduino-esp32 3.3.2 Thread-support is nog niet productierijp**. De instelling heeft momenteel geen functioneel effect: Matter start altijd via WiFi. Toekomstig werk vereist OpenThread-initialisatie in de sketch + aanwezige border router.

---

## 4. Eerste keer flashen

**Doe dit via USB — niet via OTA.**

Een nieuwe partitietabel kan nooit over-the-air gezet worden. Na deze eerste USB-flash is OTA wél hersteld voor toekomstige sketch-updates.

Volgorde per controller:
1. Sluit ESP32-C6 aan via USB
2. Selecteer de juiste COM-poort in Arduino IDE
3. Flash de sketch
4. Open Serial Monitor (115200 baud)

---

## 5. Na het flashen — koppelen met Apple Home

De pairingcode verschijnt in de Serial Monitor bij opstarten. Hij is ook beschikbaar via:
- **ROOM:** `http://<IP>/matter`
- **HVAC / ECO:** serial monitor (geen webserver op die controllers)

**Koppelstappen in Apple Home:**
1. Open de **Home** app
2. Tik **+** → **Accessoire toevoegen** → **Meer opties**
3. Voer de pairingcode in

> Als de controller eerder al gepaard was met een ander Matter-experiment: typ eerst `reset-matter` in de Serial Monitor vóór je koppelt. Anders weigert Apple Home de koppeling.

---

## 6. Matter reset commando's (alle drie sketches)

Via Serial Monitor (115200 baud):

| Commando | Effect |
|---|---|
| `reset-matter` | Wist alleen Matter/HomeKit koppeling — ROOM/HVAC/ECO instellingen blijven intact |
| `reset-all` | Wist alles: instellingen + Matter (gebruik alleen bij volledige herstart) |
| `status` | Drukt statusrapport af in Serial Monitor |

---

## 7. Endpoints per controller — overzicht

### ROOM (`ESP32_C6_MATTER_ROOM_v22.ino`)
| Endpoint | Type | Variabele |
|---|---|---|
| Temperatuur | `MatterTemperatureSensor` | `room_temp` |
| Vochtigheid | `MatterHumiditySensor` | `humi` |
| PIR 1 | `MatterOccupancySensor` | `mov1_light` |
| PIR 2 | `MatterOccupancySensor` | `mov2_light` |
| CO₂ (fake) | `MatterTemperatureSensor` | `co2 ÷ 100` → hernoem "CO2 ÷100" |
| Lux (fake) | `MatterTemperatureSensor` | `sun_light ÷ 10` → hernoem "Lux ÷10" |
| Thermostaat | `MatterThermostat` | `heating_setpoint` |
| Sfeerverlichting | `MatterColorLight` | `neo_r/g/b` — kleurpicker only, on/off genegeerd |
| Bed | `MatterOnOffPlugin` | `bed` |
| Thuis/Weg | `MatterOnOffPlugin` | `home_mode` |
| PIR1 licht | `MatterOnOffLight` | `pixel_mode[0]` — stuurt NeoPixel aan |
| PIR2 licht | `MatterOnOffLight` | `pixel_mode[1]` — stuurt NeoPixel aan |

### HVAC (`ESP32_C6_HVAC_SIM.ino`)
| Endpoint | Type | Variabele |
|---|---|---|
| Boiler top/mid/bot | `MatterTemperatureSensor` × 3 | `sch_temps[0/2/5]` |
| kWh (fake) | `MatterTemperatureSensor` | `sch_qtot` → hernoem "kWh SCH" |
| Vermogen (fake) | `MatterTemperatureSensor` | `total_power` → hernoem "kW totaal" |
| Kringen 1–7 | `MatterOnOffPlugin` × 7 | `circuits[0..6].heating_on` — bidirectioneel + 3u override |
| Alles Auto | `MatterOnOffPlugin` | reset alle overrides |
| Ventilatie | `MatterFan` | `vent_percent` — snelheid + aan/uit |

### ECO Boiler (`ESP32_C6_ECO_BOILER.ino`)
| Endpoint | Type | Variabele |
|---|---|---|
| Pomp | `MatterFan` | `pwm_value` — snelheid 0–100% ↔ PWM 0–255 |

---

## 8. Aanbevolen testvolgorde (per controller)

1. **Read-only sensoren** valideren in Apple Home — verschijnen ze? Kloppen de waarden?
2. **Callbacks testen** — schakelaar in Home app → reageert serial monitor correct?
3. **Bidirectioneel testen** (HVAC kringen) — toggle in sketch → update zichtbaar in Home app?
4. **Override timeout** testen (HVAC) — na 3 uur terug naar auto?
5. **`reset-matter`** uitvoeren → opnieuw koppelen → alles nog intact?
6. Pas daarna op een productie-controller zetten

---

## 9. Naamgeving in Apple Home (aanbevolen)

Hernoem na koppelen in de Home app:

| Standaardnaam | Hernoem naar |
|---|---|
| Temperatuursensor (CO₂ fake) | `CO2 ÷100` |
| Temperatuursensor (Lux fake) | `Lux ÷10` |
| Temperatuursensor (kWh fake) | `kWh SCH` |
| Temperatuursensor (kW fake) | `kW totaal` |
| Fan (ECO boiler) | `ECO Pomp` |
| Fan (HVAC ventilatie) | `Ventilatie` |

---

*Zarlar thuisautomatisering — Filip Delannoy*
