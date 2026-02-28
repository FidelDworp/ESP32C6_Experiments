// =============================================================================
// ESP32-C6_HVAC_SIM.ino  –  v2  –  28feb26  19:30
// Simulatie van HVAC Matter/HomeKit integratie voor ESP32-C6
// Filip Delannoy – Zarlar thuisautomatisering
//
// DOEL: Test de volledige HomeKit interface zonder echte sensoren/hardware.
//       Gesimuleerde boilertemperaturen en circuittoestanden fluctueren realistisch.
//
// ENDPOINTS (14 totaal — getest op 4MB Huge App):
//   5 × MatterTemperatureSensor  (boiler top/mid/bot + kWh SCH + kW totaal)
//   1 × MatterTemperatureSensor  (vent_percent FAKE — hernoem "Vent %")
//   1 × MatterThermostat         (ventilatie override 0–100% — hernoem "Vent override")
//   7 × MatterOnOffLight         (circuits 1–7, bidirectioneel)
//   1 × MatterOnOffLight         (Alles Auto — wist alle circuit-overrides)
//
// FAKE sensors (geen eigen Matter type in Apple Home):
//   sch_qtot     → MatterTemperatureSensor  (waarde = kWh, hernoem "kWh SCH")
//   total_power  → MatterTemperatureSensor  (waarde = kW,  hernoem "kW totaal")
//   vent_percent → MatterTemperatureSensor  (waarde = %,   hernoem "Vent %")
//
// CIRCUIT-SWITCHES — bidirectioneel:
//   Automatisch : spiegelen circuits[i].heating_on naar Home app (via ignore_callbacks vlag)
//   Override    : Home app toggle → 3 uur override, daarna terug naar auto
//   Alles Auto  : één knop wist alle actieve overrides + spiegelt onmiddellijk
//
// VENTILATIE OVERRIDE — MatterThermostat:
//   Slider 7–30 (HomeKit hw-limiet) → gemapped naar 0–100% via schaalfactor (/ 23.0 * 100)
//   Setpoint op 7 (minimum) = auto, circuit-verzoeken bepalen opnieuw de fansnelheid
//   Mode OFF in Home app = override wissen, terug naar auto
//
// WIJZIGINGEN t.o.v. v1:
//   + matter_alles_auto: wist alle circuit-overrides via Home app knop
//   + ignore_callbacks vlag: voorkomt feedback-loop bij programmatische setOnOff()
//   + matter_vent_display: actuele fansnelheid zichtbaar in Home app
//   + matter_vent_control: ventilatie override via thermostaatwidget
//   + vent schaalfactor: slider 7–30 gemapped naar 0–100%
//   + onChangeMode: OFF zet vent override terug naar auto
//   - #define Serial Serial0 verwijderd (veroorzaakte undefined setup()/loop() link error)
//
// API: arduino-esp32-master 3.3.2
// HARDWARE: ESP32-C6, 4MB Huge App partition
// =============================================================================

#include <WiFi.h>
#include <nvs_flash.h>
#include <Matter.h>
#include <MatterEndPoints/MatterTemperatureSensor.h>
#include <MatterEndPoints/MatterOnOffLight.h>


// =============================================================================
// WiFi — iPhone personal hotspot
// =============================================================================
const char* WIFI_SSID = "iPhoneFilip";
const char* WIFI_PASS = "adnoh123";


// =============================================================================
// Matter endpoints
// =============================================================================
MatterTemperatureSensor matter_boiler_top;    // sch_temps[0]  — bovenste laag
MatterTemperatureSensor matter_boiler_mid;    // sch_temps[2]  — middelste laag
MatterTemperatureSensor matter_boiler_bot;    // sch_temps[5]  — onderste laag
MatterTemperatureSensor matter_sch_qtot;      // sch_qtot      — FAKE kWh (hernoem "kWh SCH")
MatterTemperatureSensor matter_total_power;   // total_power   — FAKE kW  (hernoem "kW totaal")
MatterTemperatureSensor matter_vent_display;  // vent_percent  — FAKE %   (hernoem "Vent %")
MatterThermostat        matter_vent_control;  // override vent % via setpoint (hernoem "Vent override")

MatterOnOffLight        matter_circuit[7];    // Kringen 1–7, bidirectioneel
MatterOnOffLight        matter_alles_auto;    // Reset alle overrides → auto (hernoem "Alles Auto")


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

// Override timeout: 3 uur
const unsigned long OVERRIDE_DURATION = 180UL * 60UL * 1000UL;

// Flag: "Alles Auto" knop ingedrukt → afhandelen in loop(), niet in callback
bool alles_auto_requested = false;

// Flag: programmatische setOnOff() → callbacks negeren (voorkomt feedback-loop)
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
  const float Cp             = 1.16f;
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
  // Top warmt snel op en koelt ook sneller af; bodem traag
  sch_temps[0] = constrain(oscil(82.0f,  6.0f, 600.0f,   0.0f), 55.0f, 92.0f);
  sch_temps[1] = constrain(oscil(79.0f,  5.5f, 640.0f,  40.0f), 50.0f, 90.0f);
  sch_temps[2] = constrain(oscil(75.0f,  5.0f, 700.0f,  90.0f), 45.0f, 88.0f);
  sch_temps[3] = constrain(oscil(68.0f,  4.5f, 780.0f, 150.0f), 40.0f, 82.0f);
  sch_temps[4] = constrain(oscil(55.0f,  4.0f, 900.0f, 220.0f), 35.0f, 70.0f);
  sch_temps[5] = constrain(oscil(42.0f,  3.0f,1100.0f, 310.0f), 25.0f, 55.0f);

  sch_qtot = calculateQtot();

  // Circuits: elke kring heeft eigen ritme (duty cycle simulatie)
  // Periodes lopen uiteen zodat ze niet allemaal synchroon aan/uit gaan
  const float periods[7] = {180.0f, 210.0f, 150.0f, 240.0f, 195.0f, 270.0f, 225.0f};
  const float offsets[7] = {  0.0f,  30.0f,  60.0f,  90.0f, 120.0f, 150.0f, 180.0f};

  for (int i = 0; i < 7; i++) {
    float wave = oscil(0.0f, 1.0f, periods[i], offsets[i]);
    // heating_on = true als golf boven drempel (varieert per kring → gevarieerde duty)
    const float thresholds[7] = {0.2f, 0.1f, 0.3f, -0.1f, 0.15f, 0.25f, 0.0f};
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
  // In productie komt dit van z_val via HTTP poll per kring
  if (!vent_override_active) {
    int max_vent = 0;
    for (int i = 0; i < 7; i++) {
      bool effective = circuits[i].override_active
                       ? circuits[i].override_state
                       : circuits[i].heating_on;
      // Simuleer vent_request per kring: AAN → willekeurig 30–80%
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
// Override timeout bewaken
// =============================================================================
void check_overrides() {
  for (int i = 0; i < 7; i++) {
    if (circuits[i].override_active &&
        millis() - circuits[i].override_start > OVERRIDE_DURATION) {
      circuits[i].override_active = false;
      Serial.printf(F("[OVERRIDE] Kring %d '%s' — override vervallen na 3u, terug naar auto\n"),
                    i + 1, circuits[i].name);
    }
  }
}


// =============================================================================
// Matter sensor updates — alle read-only sensors → HomeKit
// =============================================================================
void update_matter_sensors() {
  matter_boiler_top.setTemperature(sch_temps[0]);
  matter_boiler_mid.setTemperature(sch_temps[2]);
  matter_boiler_bot.setTemperature(sch_temps[5]);
  matter_sch_qtot.setTemperature(sch_qtot);       // kWh als "°C"
  matter_total_power.setTemperature(total_power); // kW als "°C"
  matter_vent_display.setTemperature((float)vent_percent);   // % als "°C"
  matter_vent_control.setLocalTemperature((float)vent_percent);

  // Circuit-switches: alleen spiegelen als geen override actief
  ignore_callbacks = true;  // setOnOff() hier is programmatisch, geen gebruikersingreep
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
  Serial.println(F(  "║     ESP32-C6_HVAC_SIM  –  Status        ║"));
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
    Serial.printf(   "║  override        : %d %% [OVR]\n", vent_override_percent);
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
  Serial.println(F("\n=== ESP32-C6_HVAC_SIM v1 ==="));

  // ── WiFi ─────────────────────────────────────────────────────────────────
  Serial.printf("WiFi: verbinden met '%s' ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nWiFi OK: %s\n", WiFi.localIP().toString().c_str());

  // ── Matter endpoints initialiseren ───────────────────────────────────────
  matter_boiler_top.begin();
  matter_boiler_mid.begin();
  matter_boiler_bot.begin();
  matter_sch_qtot.begin();
  matter_total_power.begin();

  // Ventilatie display (read-only)
  matter_vent_display.begin();

  // Ventilatie thermostat: setpoint = override %, local temp = actuele vent %
  // Setpoint 0 = auto, 1–100 = override
  // ControlSequenceOfOperation 2 = heating only (enkel setpoint zichtbaar, geen cooling)
  matter_vent_control.begin((MatterThermostat::ControlSequenceOfOperation_t)2);
  matter_vent_control.setLocalTemperature((float)vent_percent);
  matter_vent_control.setHeatingSetpoint(0.0f);  // Start in auto

  // Schaalfactor: slider range 7-30 (HomeKit limiet) -> 0-100% ventilatie
  // map: sp_pct = round((new_sp - 7.0) / (30.0 - 7.0) * 100)
  matter_vent_control.onChangeHeatingSetpoint([](double new_sp) -> bool {
    int sp_pct = constrain((int)round((new_sp - 7.0) / 23.0 * 100.0), 0, 100);
    vent_override_percent = sp_pct;
    vent_override_active  = (sp_pct > 0);
    if (vent_override_active) {
      vent_percent = sp_pct;
      Serial.printf("[HomeKit] Vent override -> %.1f (slider) = %d%% (effectief)\n", new_sp, sp_pct);
    } else {
      Serial.println(F("[HomeKit] Vent setpoint = 7 (min) -> terug naar auto"));
    }
    return true;
  });

  // Fix: OFF in Home app -> override wissen, terug naar auto
  matter_vent_control.onChangeMode([](uint8_t mode) -> bool {
    if (mode == 0) {
      vent_override_active  = false;
      vent_override_percent = 0;
      matter_vent_control.setHeatingSetpoint(0.0f);
      Serial.println(F("[HomeKit] Vent mode OFF -> override gewist, terug naar auto"));
    }
    return true;
  });

  // Circuit-switches: begin + callback
  for (int i = 0; i < 7; i++) {
    matter_circuit[i].begin();
    matter_circuit[i].setOnOff(circuits[i].heating_on);

    // Lambda met index capture — override activeren bij handmatige ingreep
    matter_circuit[i].onChangeOnOff([i](bool on_off) -> bool {
      if (ignore_callbacks) return true;  // Programmatische update — negeren
      circuits[i].override_active = true;
      circuits[i].override_state  = on_off;
      circuits[i].override_start  = millis();
      Serial.printf("[HomeKit] Kring %d '%s' → override %s (3u)\n",
                    i + 1, circuits[i].name, on_off ? "AAN" : "UIT");
      // "Alles Auto" knop uit: er is nu een actieve override
      matter_alles_auto.setOnOff(false);
      return true;
    });
  }

  // ── Alles Auto switch ────────────────────────────────────────────────────
  matter_alles_auto.begin();
  matter_alles_auto.setOnOff(false);  // Altijd UIT in rust
  matter_alles_auto.onChangeOnOff([](bool on_off) -> bool {
    if (on_off) {
      // Niet hier afhandelen — vlag zetten, loop() doet de rest
      alles_auto_requested = true;
      Serial.println(F("[HomeKit] Alles Auto → aangevraagd, loop() handelt af"));
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
    ignore_callbacks = false;
    matter_alles_auto.setOnOff(false);
    Serial.printf("[AUTO] %d override(s) gewist — alle kringen terug naar [AUT]\n", count);
  }

  // Override timeouts bewaken
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
