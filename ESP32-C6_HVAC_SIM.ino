// =============================================================================
// ESP32-C6_HVAC_SIM.ino  –  v2.1 –  01mrt26
// Simulatie van HVAC Matter/HomeKit integratie voor ESP32-C6
// Filip Delannoy – Zarlar thuisautomatisering
//
// WIJZIGINGEN v2:
//   MatterOnOffLight → MatterOnOffPlugin voor alle kring-switches en Alles Auto
//     → semantisch correct: een verwarmingskring is een plugin, geen licht
//   MatterThermostat + MatterTemperatureSensor (vent display) → MatterFan
//     → 1 endpoint ipv 2, directe speed 0–100% ↔ vent_percent
//     → geen setpoint-truc meer nodig
//   Race condition fix in circuit callbacks:
//     override_start = millis() EERST, dan override_active = true
//     → zelfde principe als ECO-boiler v6.3: garandeert correcte starttijd
//       vóór check_overrides() de vlag ziet (Matter callbacks = aparte FreeRTOS-taak)
//   ignore_callbacks in MatterFan callbacks consistent toegepast (ECO-patroon)
//   update_matter_sensors() aanroep na sim_step() in setup() (ECO-patroon)
//
// WIJZIGINGEN v2.1:
//   FIX: setMode() samen met setSpeedPercent() in update_matter_sensors()
//     → Home app bevriest slider op 0 als fan in FAN_MODE_OFF staat;
//       setSpeedPercent() zonder mode-update wordt dan genegeerd
//     → auto mode: vent_percent>0 → FAN_MODE_HIGH, ==0 → FAN_MODE_OFF
//     → override mode: mode ongemoeid (gebruiker bepaalt via Home app)
//   FIX: resterende override-tijd toegevoegd aan print_status() ventilatie
//
// DOEL: Test de volledige HomeKit interface zonder echte sensoren/hardware.
//       Gesimuleerde boilertemperaturen en circuittoestanden fluctueren realistisch.
//       Circuit-switches zijn bidirectioneel:
//         - Automatisch: spiegelen circuits[i].heating_on → Home app
//         - Override: Home app toggle → 3 uur override, dan terug naar automaat
//
// FAKE sensors (geen eigen Matter type in Apple Home):
//   sch_qtot      → MatterTemperatureSensor  (waarde = kWh, hernoem naar "kWh SCH")
//   total_power   → MatterTemperatureSensor  (waarde = kW,  hernoem naar "kW totaal")
//
// ENDPOINTS (13 totaal — past op 4MB Huge App):
//   5 × MatterTemperatureSensor  (boiler top/mid/bot + qtot + vermogen)
//   7 × MatterOnOffPlugin        (circuits 1–7, bidirectioneel)
//   1 × MatterOnOffPlugin        (Alles Auto — reset alle overrides)
//   1 × MatterFan                (ventilatie: snelheid + aan/uit, hernoem "Ventilatie")
//
// API: arduino-esp32-master 3.3.2
// HARDWARE: ESP32-C6, 4MB Huge App partition
// =============================================================================

#include <WiFi.h>
#include <nvs_flash.h>
#include <Matter.h>
#include <MatterEndPoints/MatterTemperatureSensor.h>
#include <MatterEndPoints/MatterOnOffPlugin.h>
#include <MatterEndPoints/MatterFan.h>


// =============================================================================
// WiFi — iPhone personal hotspot
// =============================================================================
const char* WIFI_SSID = "iPhoneFilip";
const char* WIFI_PASS = "adnoh123";


// =============================================================================
// Matter endpoints
// =============================================================================
MatterTemperatureSensor matter_boiler_top;   // sch_temps[0]  — bovenste laag
MatterTemperatureSensor matter_boiler_mid;   // sch_temps[2]  — middelste laag
MatterTemperatureSensor matter_boiler_bot;   // sch_temps[5]  — onderste laag
MatterTemperatureSensor matter_sch_qtot;     // sch_qtot      — FAKE kWh (hernoem "kWh SCH")
MatterTemperatureSensor matter_total_power;  // total_power   — FAKE kW  (hernoem "kW totaal")

MatterOnOffPlugin       matter_circuit[7];   // Kringen 1–7, bidirectioneel
MatterOnOffPlugin       matter_alles_auto;   // Reset alle overrides → auto (hernoem "Alles Auto")

MatterFan               matter_vent;         // Ventilatie: snelheid + aan/uit (hernoem "Ventilatie")


// =============================================================================
// Gesimuleerde variabelen — zelfde namen als ESP32_HVAC.ino
// =============================================================================

// Boiler temperaturen (6 lagen, indices 0=top … 5=bottom)
float sch_temps[6]  = {82.0, 79.0, 75.0, 68.0, 55.0, 42.0};
float sch_qtot      = 12.5;   // kWh energieinhoud SCH-boiler

// Circuits — vereenvoudigde struct voor simulatie
struct SimCircuit {
  const char* name;
  float       power_kw;
  bool        heating_on;       // Automatische toestand (thermostaat)
  bool        override_active;
  bool        override_state;
  unsigned long override_start;
};

SimCircuit circuits[7] = {
  {"Woonkamer",   1.200, false, false, false, 0},
  {"Keuken",      0.800, false, false, false, 0},
  {"Badkamer",    0.600, false, false, false, 0},
  {"Slaapkamer1", 0.700, false, false, false, 0},
  {"Slaapkamer2", 0.700, false, false, false, 0},
  {"Bureau",      0.500, false, false, false, 0},
  {"Gang",        0.400, false, false, false, 0},
};

float total_power = 0.0;  // Som van power_kw van actieve kringen

// Ventilatie
int   vent_percent          = 0;    // Actuele fansnelheid (0–100%), max van alle circuit-verzoeken
int   vent_override_percent = 0;    // 0 = auto, 1–100 = override
bool  vent_override_active  = false;
unsigned long vent_override_start = 0;

// Override timeout: 3 uur (circuits), 3 uur (ventilatie)
const unsigned long OVERRIDE_DURATION     = 180UL * 60UL * 1000UL;
const unsigned long VENT_OVERRIDE_DURATION = 180UL * 60UL * 1000UL;

// Flag: "Alles Auto" knop ingedrukt → afhandelen in loop(), niet in callback
bool alles_auto_requested = false;

// Flag: programmatische setOnOff() / setSpeedPercent() → callbacks negeren
// (voorkomt feedback-loop; zelfde principe als ECO-boiler)
bool ignore_callbacks = false;

// Simulatie timers
unsigned long last_sim_step   = 0;
unsigned long last_serial_log = 0;


// =============================================================================
// Hulpfunctie: sinusoïdale oscillatie
// =============================================================================
float oscil(float center, float amplitude, float period_s, float offset_s) {
  float t = millis() / 1000.0f + offset_s;
  return center + amplitude * sinf(2.0f * PI * t / period_s);
}


// =============================================================================
// Bereken energieinhoud SCH-boiler (vereenvoudigd, identiek aan productie)
// =============================================================================
float calculateQtot() {
  const float Cp                = 1.16f;
  const float boiler_ref_temp   = 20.0f;
  const float boiler_layer_volume = 50.0f;
  float total_energy = 0.0f;

  float T_layer0 = (boiler_ref_temp + sch_temps[0]) / 2.0f;
  if (T_layer0 > boiler_ref_temp)
    total_energy += (T_layer0 - boiler_ref_temp) * boiler_layer_volume * Cp;

  for (int i = 1; i < 6; i++) {
    float T_layer = (sch_temps[i-1] + sch_temps[i]) / 2.0f;
    if (T_layer > boiler_ref_temp)
      total_energy += (T_layer - boiler_ref_temp) * boiler_layer_volume * Cp;
  }

  float T_layer6 = sch_temps[5];
  if (T_layer6 > boiler_ref_temp)
    total_energy += (T_layer6 - boiler_ref_temp) * boiler_layer_volume * Cp;

  return total_energy / 1000.0f;
}


// =============================================================================
// Simulatiestap — elke 5 seconden
// =============================================================================
void sim_step() {

  // Boilerlagen: stratificatie — elke laag eigen fase en amplitude
  sch_temps[0] = constrain(oscil(82.0f,  6.0f,  600.0f,   0.0f), 55.0f, 92.0f);
  sch_temps[1] = constrain(oscil(79.0f,  5.5f,  640.0f,  40.0f), 50.0f, 90.0f);
  sch_temps[2] = constrain(oscil(75.0f,  5.0f,  700.0f,  90.0f), 45.0f, 88.0f);
  sch_temps[3] = constrain(oscil(68.0f,  4.5f,  780.0f, 150.0f), 40.0f, 82.0f);
  sch_temps[4] = constrain(oscil(55.0f,  4.0f,  900.0f, 220.0f), 35.0f, 70.0f);
  sch_temps[5] = constrain(oscil(42.0f,  3.0f, 1100.0f, 310.0f), 25.0f, 55.0f);

  sch_qtot = calculateQtot();

  // Circuits: elke kring heeft eigen ritme (duty cycle simulatie)
  const float periods[7]    = {180.0f, 210.0f, 150.0f, 240.0f, 195.0f, 270.0f, 225.0f};
  const float offsets[7]    = {  0.0f,  30.0f,  60.0f,  90.0f, 120.0f, 150.0f, 180.0f};
  const float thresholds[7] = {  0.2f,   0.1f,   0.3f,  -0.1f,  0.15f,  0.25f,   0.0f};

  for (int i = 0; i < 7; i++) {
    float wave = oscil(0.0f, 1.0f, periods[i], offsets[i]);
    circuits[i].heating_on = (wave > thresholds[i]);
  }

  // Totaal gevraagd vermogen
  total_power = 0.0f;
  for (int i = 0; i < 7; i++) {
    bool effective = circuits[i].override_active
                     ? circuits[i].override_state
                     : circuits[i].heating_on;
    if (effective) total_power += circuits[i].power_kw;
  }

  // Ventilatie: max van alle circuit-verzoeken (gesimuleerd op basis van heating_on)
  if (!vent_override_active) {
    int max_vent = 0;
    for (int i = 0; i < 7; i++) {
      bool effective = circuits[i].override_active
                       ? circuits[i].override_state
                       : circuits[i].heating_on;
      if (effective) {
        int sim_vent = 30 + (int)oscil(25.0f, 20.0f, periods[i], offsets[i] + 45.0f);
        sim_vent = constrain(sim_vent, 0, 100);
        if (sim_vent > max_vent) max_vent = sim_vent;
      }
    }
    vent_percent = max_vent;
  } else {
    vent_percent = vent_override_percent;
  }
}


// =============================================================================
// Override timeouts bewaken
// =============================================================================
void check_overrides() {
  // Circuit overrides
  for (int i = 0; i < 7; i++) {
    if (circuits[i].override_active &&
        millis() - circuits[i].override_start > OVERRIDE_DURATION) {
      circuits[i].override_active = false;
      Serial.printf("[OVERRIDE] Kring %d '%s' — vervallen na 3u, terug naar auto\n",
                    i + 1, circuits[i].name);
      // Matter UI wordt bijgewerkt in update_matter_sensors() binnen 5s
    }
  }

  // Ventilatie override
  if (vent_override_active &&
      millis() - vent_override_start > VENT_OVERRIDE_DURATION) {
    vent_override_active  = false;
    vent_override_percent = 0;
    Serial.println(F("[OVERRIDE] Ventilatie — vervallen na 3u, terug naar auto"));
    // Fan speed wordt bijgewerkt in update_matter_sensors() binnen 5s
  }
}


// =============================================================================
// Matter sensor updates — alle read-only sensors + fan feedback → HomeKit
//
// REGEL: setSpeedPercent() mag hier (feedback, geen gebruikersingreep).
//        ignore_callbacks: voorkomt dat setSpeedPercent() de onChangeSpeedPercent
//        callback triggert en een nieuwe override aanmaakt (ECO-patroon).
// =============================================================================
void update_matter_sensors() {
  matter_boiler_top.setTemperature(sch_temps[0]);
  matter_boiler_mid.setTemperature(sch_temps[2]);
  matter_boiler_bot.setTemperature(sch_temps[5]);
  matter_sch_qtot.setTemperature(sch_qtot);       // kWh als "°C"
  matter_total_power.setTemperature(total_power); // kW als "°C"

  // Fan: snelheidsfeedback = actuele vent_percent terugpushen naar HomeKit
  // In auto mode updaten we ook de mode (ON/OFF), anders bevriest de slider
  // op 0 wanneer de fan in FAN_MODE_OFF staat — Home app negeert setSpeedPercent()
  // dan volledig. In override mode laten we de mode ongemoeid (gebruiker bepaalt).
  ignore_callbacks = true;
  matter_vent.setSpeedPercent((uint8_t)vent_percent);
  if (!vent_override_active) {
    matter_vent.setMode(vent_percent > 0
                        ? MatterFan::FAN_MODE_HIGH
                        : MatterFan::FAN_MODE_OFF);
  }
  ignore_callbacks = false;

  // Circuit-switches: alleen spiegelen als geen override actief
  ignore_callbacks = true;
  for (int i = 0; i < 7; i++) {
    if (!circuits[i].override_active) {
      matter_circuit[i].setOnOff(circuits[i].heating_on);
    }
  }
  ignore_callbacks = false;
}


// =============================================================================
// Serial statusrapport
// =============================================================================
void print_status() {
  Serial.println(F("\n╔══════════════════════════════════════════╗"));
  Serial.println(F(  "║     ESP32-C6_HVAC_SIM  –  v2  Status    ║"));
  Serial.printf(     "║  Uptime : %lu s\n", millis() / 1000);
  Serial.println(F(  "╠══════════════════════════════════════════╣"));
  Serial.println(F(  "║ BOILER SCH                                "));
  Serial.printf(     "║  Top [0]  : %5.1f °C\n", sch_temps[0]);
  Serial.printf(     "║  Mid [2]  : %5.1f °C\n", sch_temps[2]);
  Serial.printf(     "║  Bot [5]  : %5.1f °C\n", sch_temps[5]);
  Serial.printf(     "║  Qtot     : %5.2f kWh\n", sch_qtot);
  Serial.println(F(  "╠══════════════════════════════════════════╣"));
  Serial.println(F(  "║ CIRCUITS                                  "));
  for (int i = 0; i < 7; i++) {
    bool effective = circuits[i].override_active
                     ? circuits[i].override_state
                     : circuits[i].heating_on;
    const char* src = circuits[i].override_active ? "OVR" : "AUT";
    Serial.printf("║  c%d %-12s %.3fkW  %s [%s]",
                  i + 1,
                  circuits[i].name,
                  circuits[i].power_kw,
                  effective ? "AAN" : "UIT",
                  src);
    if (circuits[i].override_active) {
      unsigned long remaining = (OVERRIDE_DURATION -
                                (millis() - circuits[i].override_start)) / 60000UL;
      Serial.printf(" %lum resterend", remaining);
    }
    Serial.println();
  }
  Serial.printf(     "║  Totaal vermogen : %.3f kW\n", total_power);
  Serial.println(F(  "╠══════════════════════════════════════════╣"));
  Serial.println(F(  "║ VENTILATIE                                "));
  Serial.printf(     "║  vent_percent    : %d %%\n", vent_percent);
  if (vent_override_active) {
    unsigned long vent_rem = (VENT_OVERRIDE_DURATION -
                             (millis() - vent_override_start)) / 60000UL;
    Serial.printf(   "║  override        : %d %% [OVR] %lum resterend\n",
                     vent_override_percent, vent_rem);
  } else {
    Serial.println(F("║  override        : auto [AUT]"));
  }
  Serial.println(F(  "╠══════════════════════════════════════════╣"));
  Serial.printf(     "║  Free heap : %d %%\n",
                     (ESP.getFreeHeap() * 100) / ESP.getHeapSize());
  Serial.printf(     "║  WiFi RSSI : %d dBm\n", WiFi.RSSI());
  Serial.println(F(  "╚══════════════════════════════════════════╝\n"));
}


// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== ESP32-C6_HVAC_SIM v2 ==="));

  // ── WiFi ─────────────────────────────────────────────────────────────────
  Serial.printf("WiFi: verbinden met '%s' ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nWiFi OK: %s\n", WiFi.localIP().toString().c_str());

  // ── Temperatuursensoren ──────────────────────────────────────────────────
  matter_boiler_top.begin();
  matter_boiler_mid.begin();
  matter_boiler_bot.begin();
  matter_sch_qtot.begin();
  matter_total_power.begin();

  // ── Circuit-plugins: begin + callback ───────────────────────────────────
  for (int i = 0; i < 7; i++) {
    matter_circuit[i].begin();
    matter_circuit[i].setOnOff(circuits[i].heating_on);

    // Lambda met index capture — override activeren bij handmatige ingreep
    // Race condition fix: override_start EERST zetten vóór override_active = true
    // → Matter callbacks lopen in eigen FreeRTOS-taak; check_overrides() in loop-taak
    // → starttijd moet gegarandeerd klaar zijn vóór de vlag zichtbaar wordt
    matter_circuit[i].onChangeOnOff([i](bool on_off) -> bool {
      if (ignore_callbacks) return true;  // Programmatische update — negeren
      circuits[i].override_start  = millis();   // EERST
      circuits[i].override_active = true;       // DAN
      circuits[i].override_state  = on_off;
      Serial.printf("[HomeKit] Kring %d '%s' → override %s (3u)\n",
                    i + 1, circuits[i].name, on_off ? "AAN" : "UIT");
      // "Alles Auto" plugin uit: er is nu een actieve override
      ignore_callbacks = true;
      matter_alles_auto.setOnOff(false);
      ignore_callbacks = false;
      return true;
    });
  }

  // ── Alles Auto plugin ────────────────────────────────────────────────────
  matter_alles_auto.begin();
  matter_alles_auto.setOnOff(false);  // Altijd UIT in rust
  matter_alles_auto.onChangeOnOff([](bool on_off) -> bool {
    if (ignore_callbacks) return true;
    if (on_off) {
      // Niet hier afhandelen — vlag zetten, loop() doet de rest
      alles_auto_requested = true;
      Serial.println(F("[HomeKit] Alles Auto → aangevraagd, loop() handelt af"));
    }
    return true;
  });

  // ── MatterFan: ventilatiebediening ──────────────────────────────────────
  // FAN_MODE_SEQ_OFF_HIGH: enkel OFF en HIGH als modus-stappen
  // Begin met speed 0, mode OFF
  matter_vent.begin(0, MatterFan::FAN_MODE_OFF, MatterFan::FAN_MODE_SEQ_OFF_HIGH);

  // Speed slider bewogen in Home app → ventilatie override
  matter_vent.onChangeSpeedPercent([](uint8_t new_pct) -> bool {
    if (ignore_callbacks) return true;

    if (new_pct == 0) {
      // Snelheid naar 0 → terug naar auto
      vent_override_active  = false;
      vent_override_percent = 0;
      Serial.println(F("[HomeKit] Vent speed = 0% → terug naar auto"));
    } else {
      // Override activeren — starttijd EERST, dan vlag (race condition fix)
      vent_override_start   = millis();   // EERST
      vent_override_active  = true;       // DAN
      vent_override_percent = new_pct;
      vent_percent          = new_pct;
      Serial.printf("[HomeKit] Vent speed override → %d%%\n", new_pct);
    }
    return true;
  });

  // Mode gewijzigd in Home app (OFF-knop of modus-selectie)
  matter_vent.onChangeMode([](uint8_t new_mode) -> bool {
    if (ignore_callbacks) return true;

    if (new_mode == MatterFan::FAN_MODE_OFF) {
      // Expliciet UIT gezet → override wissen, terug naar auto
      vent_override_active  = false;
      vent_override_percent = 0;
      ignore_callbacks = true;
      matter_vent.setSpeedPercent(0);
      ignore_callbacks = false;
      Serial.println(F("[HomeKit] Vent mode OFF → terug naar auto, speed=0"));
    } else {
      // Mode AAN zonder specifieke speed → minimale override
      if (!vent_override_active) {
        vent_override_start   = millis();   // EERST
        vent_override_active  = true;       // DAN
        vent_override_percent = 30;         // Minimale ventilatie bij handmatig AAN
        ignore_callbacks = true;
        matter_vent.setSpeedPercent(30);
        ignore_callbacks = false;
        Serial.println(F("[HomeKit] Vent mode AAN → override 30%"));
      }
    }
    return true;
  });

  // ── Matter starten ───────────────────────────────────────────────────────
  Matter.begin();

  // ── Pairing check ────────────────────────────────────────────────────────
  Serial.println(F("\n══════════════════════════════════════════"));
  if (!Matter.isDeviceCommissioned()) {
    Serial.println(F("MATTER: Nog niet gepaard."));
    Serial.println(F("► Manuele code:"));
    Serial.println("    " + Matter.getManualPairingCode());
    Serial.println(F("► Home app → + → Accessoire → Meer opties → code invoeren"));
    Serial.println(F("Wacht op commissioning..."));
    while (!Matter.isDeviceCommissioned()) { delay(500); Serial.print("."); }
    Serial.println(F("\nGEPAARD!"));
  } else {
    Serial.println(F("MATTER: Al gepaard. Typ 'reset' om te wissen."));
  }
  Serial.println(F("══════════════════════════════════════════\n"));

  // Eerste simulatiestap en push naar Matter
  sim_step();
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

  // "Alles Auto" afhandelen: overrides wissen + circuits onmiddellijk spiegelen
  if (alles_auto_requested) {
    alles_auto_requested = false;
    int count = 0;
    ignore_callbacks = true;  // Voorkomt feedback-loop via circuit callbacks
    for (int i = 0; i < 7; i++) {
      if (circuits[i].override_active) {
        circuits[i].override_active = false;
        count++;
        matter_circuit[i].setOnOff(circuits[i].heating_on);
      }
    }
    // Ventilatie ook terugzetten naar auto
    if (vent_override_active) {
      vent_override_active  = false;
      vent_override_percent = 0;
      matter_vent.setSpeedPercent((uint8_t)vent_percent);
      count++;
    }
    ignore_callbacks = false;
    matter_alles_auto.setOnOff(false);
    Serial.printf("[AUTO] %d override(s) gewist — alle kringen + vent terug naar [AUT]\n", count);
  }

  // check_overrides() vóór de 5s timer: als vlag net gewist wordt,
  // pusht update_matter_sensors() onmiddellijk de correcte auto-toestand.
  check_overrides();

  // Simulatiestap + Matter update elke 5s
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
