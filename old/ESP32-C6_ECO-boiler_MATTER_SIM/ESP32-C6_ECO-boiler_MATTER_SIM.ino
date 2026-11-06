// =============================================================================
// ESP32-C6_ECO-boiler_SIM.ino  –  v6.3  –  01mrt26 16:00
// Simulatie van ECO boiler Matter/HomeKit integratie voor ESP32-C6
// Filip Delannoy – Zarlar thuisautomatisering
//
// WIJZIGINGEN v6:
//   MatterTemperatureSensor voor EQtot vervangen door MatterHumiditySensor
//     → EQtot (kWh) gemapped naar 0–100% "boiler volheid"
//     → mapping: eq_pct = constrain(EQtot / EQ_MAX * 100, 0, 100)
//     → EQ_MAX = 25.0 kWh (volledig opgeladen boiler ~80°C gemiddeld)
//     → semantisch intuïtief: 0% = koud/leeg, 100% = volledig opgeladen
//     → bruikbaar in automatiseringsregel: "als boiler > 80% → stop pomp"
//     → hernoemingslabel: "% Boiler vol"
//   EQ_MAX nu dynamisch via calcEQmax() i.p.v. hardcoded 25 kWh
//     → calcEQmax() = BOILER_VOLUME_TOTAL * 1.16 * (TSUN_HIGH - 20) / 1000
//     → in productie: aanroepen na loadConfig() en na /settings wijziging
//     → schaal altijd correct ongeacht ingesteld volume of temperatuurdrempel
//
// WIJZIGINGEN v6.3:
//   FIX race condition: pump_override_start = millis() nu VOOR pump_override_active = true
//     → Matter callbacks lopen in eigen FreeRTOS-taak; check_overrides() in loop-taak
//     → volgorde garandeert dat starttijd klaar is vóór de vlag zichtbaar is
//
// WIJZIGINGEN v5.1:
//   FIX compilatiefout: onChangeFanMode() → onChangeMode()
//
// WIJZIGINGEN v5:
//   MatterThermostat + MatterOnOffLight vervangen door MatterFan (1 endpoint)
//     → Fan speed 0–100% ↔ PWM 0–255  (directe lineaire mapping, geen truc)
//     → Semantisch correct: een pomp IS een ventilator in Matter-termen
//
// ENDPOINTS (5 totaal — ruim op 4MB Huge App):
//   3 × MatterTemperatureSensor  (Tsun, ETopH, EBotH)
//   1 × MatterHumiditySensor     (EQtot als % boilervolheid, hernoem "% Boiler vol")
//   1 × MatterFan                (pomp: snelheid + aan/uit, hernoem "Pomp ECO")
//
// MAPPINGS:
//   pwm_to_pct(pwm)  = round(pwm / 255.0 * 100)            PWM 0–255 → % 0–100
//   pct_to_pwm(pct)  = round(pct / 100.0 * 255)            % 0–100   → PWM 0–255
//   eq_to_pct(kWh)   = constrain(kWh / 25.0 * 100, 0, 100) kWh       → % volheid
//
// INVARIANT: pump_relay == (pwm_value > 0)  — altijd, in alle paden.
//
// OVERRIDE: 60s timer — daarna terug naar automaat
//   Via speed slider: pct>0 → override aan, pct==0 → terug naar auto
//   Via mode OFF:     onmiddellijk terug naar auto
//
// API: arduino-esp32 3.3.7
// HARDWARE: ESP32-C6, 4MB Huge App partition
// =============================================================================

#include <WiFi.h>
#include <nvs_flash.h>
#include <Matter.h>
#include <MatterEndPoints/MatterTemperatureSensor.h>
#include <MatterEndPoints/MatterHumiditySensor.h>
#include <MatterEndPoints/MatterFan.h>


// =============================================================================
// WiFi — iPhone personal hotspot
// =============================================================================
const char* WIFI_SSID = "iPhoneFilip";
const char* WIFI_PASS = "adnoh123";


// =============================================================================
// Matter endpoints
// =============================================================================
MatterTemperatureSensor matter_tsun;   // Tsun  — collector  (hernoem "°C Collector")
MatterTemperatureSensor matter_etoph;  // ETopH — top laag   (hernoem "°C Top")
MatterTemperatureSensor matter_eboth;  // EBotH — onderste   (hernoem "°C Bodem")
MatterHumiditySensor    matter_eqpct;  // EQtot als % boilervolheid (hernoem "% Boiler vol")
MatterFan               matter_pomp;   // Pomp: snelheid + aan/uit (hernoem "Pomp ECO")


// =============================================================================
// Gesimuleerde variabelen — zelfde namen als ESP32C6_ECO-boiler.ino
// =============================================================================
float Tsun  = 45.0;
float ETopH = 65.0;
float ETopL = 60.0;   // intern
float EMidH = 55.0;   // intern
float EMidL = 50.0;   // intern
float EBotH = 38.0;   // dT-referentie
float EBotL = 34.0;   // intern
float EAv   = 0.0;
float dT    = 0.0;
float EQtot = 0.0;
// EQ_MAX dynamisch: zelfde formule als productie, afhankelijk van BOILER_VOLUME_TOTAL en TSUN_HIGH
// → in productie automatisch correct na loadConfig() + aanpassing via /settings
float EQ_MAX = 0.0f;  // berekend in setup() via calcEQmax()

bool  pump_relay = false;
int   pwm_value  = 0;

// Drempelwaarden (identiek aan productie defaults)
const float DT_START      = 3.0f;
const float DT_STOP       = 2.0f;
const int   PWM_MIN       = 80;
const int   PWM_MAX       = 200;
const int   PWM_OVERHEAT  = 255;
const float TSUN_OVERHEAT = 90.0f;
const float TSUN_MIN      = 22.0f;
const float TSUN_HIGH     = 75.0f;   // °C — normaal max (basis voor EQ_MAX)
const float BOILER_VOLUME_TOTAL = 490.0f; // L — identiek aan productie default

// Override
bool          pump_override_active = false;
int           pwm_override         = 0;
unsigned long pump_override_start  = 0;
const unsigned long OVERRIDE_DURATION = 60UL * 1000UL;  // 60s

// Hysteresis automaat
bool pump_on_auto = false;

// Flag: programmatische Matter-calls → callbacks negeren
bool ignore_callbacks = false;

// Timers
unsigned long last_sim_step   = 0;
unsigned long last_serial_log = 0;


// =============================================================================
// Mapping PWM ↔ fan speed percentage
// =============================================================================
uint8_t pwm_to_pct(int pwm) {
  return (uint8_t)constrain((int)round(pwm / 255.0f * 100.0f), 0, 100);
}

int pct_to_pwm(uint8_t pct) {
  return constrain((int)round(pct / 100.0f * 255.0f), 0, 255);
}

uint8_t eq_to_pct(float kWh) {
  return (uint8_t)constrain((int)round(kWh / EQ_MAX * 100.0f), 0, 100);
}

// Berekent maximale energieinhoud boiler op basis van volume en temperatuurdrempel
// Identiek toe te passen in productie na loadConfig() of na /settings wijziging
// Cp = 1.16 kWh/m³K, Tref = 20°C, T_max = TSUN_HIGH (praktisch maximum)
float calcEQmax() {
  return BOILER_VOLUME_TOTAL * 1.16f * (TSUN_HIGH - 20.0f) / 1000.0f;
}


// =============================================================================
// Hulpfunctie: sinusoïdale oscillatie
// =============================================================================
float oscil(float center, float amplitude, float period_s, float offset_s) {
  float t = millis() / 1000.0f + offset_s;
  return center + amplitude * sinf(2.0f * PI * t / period_s);
}


// =============================================================================
// Energieberekening boiler
// =============================================================================
float calculateEQtot() {
  const float Cp   = 1.16f;
  const float Tref = 20.0f;
  const float vol  = BOILER_VOLUME_TOTAL / 6.0f;
  float T[6] = {ETopH, ETopL, EMidH, EMidL, EBotH, EBotL};
  float total = 0.0f;
  float T0 = (Tref + T[0]) / 2.0f;
  if (T0 > Tref) total += (T0 - Tref) * vol * Cp / 1000.0f;
  for (int i = 1; i < 6; i++) {
    float Tm = (T[i-1] + T[i]) / 2.0f;
    if (Tm > Tref) total += (Tm - Tref) * vol * Cp / 1000.0f;
  }
  if (T[5] > Tref) total += (T[5] - Tref) * vol * Cp / 1000.0f;
  return total;
}


// =============================================================================
// Simulatiestap — elke 5s
// =============================================================================
void sim_step() {
  Tsun  = constrain(oscil(45.0f, 35.0f, 600.0f,   0.0f),  5.0f, 88.0f);
  ETopH = constrain(oscil(65.0f,  8.0f, 700.0f,  50.0f), 35.0f, 80.0f);
  ETopL = constrain(oscil(60.0f,  7.0f, 730.0f,  80.0f), 32.0f, 75.0f);
  EMidH = constrain(oscil(55.0f,  6.0f, 800.0f, 120.0f), 28.0f, 68.0f);
  EMidL = constrain(oscil(48.0f,  5.5f, 870.0f, 165.0f), 26.0f, 62.0f);
  EBotH = constrain(oscil(38.0f,  5.0f, 900.0f, 220.0f), 20.0f, 50.0f);
  EBotL = constrain(oscil(34.0f,  4.0f,1000.0f, 280.0f), 18.0f, 45.0f);
  EAv   = (ETopH + ETopL + EMidH + EMidL + EBotH + EBotL) / 6.0f;
  dT    = Tsun - EBotH;
  EQtot = calculateEQtot();

  if (!pump_override_active) {
    // Hysteresis automaat
    if (!pump_on_auto && dT > DT_START && Tsun > TSUN_MIN) pump_on_auto = true;
    else if (pump_on_auto && dT < DT_STOP)                 pump_on_auto = false;

    pump_relay = pump_on_auto;
    if (pump_on_auto) {
      if (Tsun >= TSUN_OVERHEAT) {
        pwm_value = PWM_OVERHEAT;
      } else {
        float ratio = constrain((dT - DT_START) / (15.0f - DT_START), 0.0f, 1.0f);
        pwm_value = (int)(PWM_MIN + ratio * (PWM_MAX - PWM_MIN));
      }
    } else {
      pwm_value = 0;
    }
  } else {
    pwm_value  = pwm_override;
    pump_relay = (pwm_value > 0);  // INVARIANT
  }
}


// =============================================================================
// Override timeout bewaken
// =============================================================================
void check_overrides() {
  if (pump_override_active &&
      millis() - pump_override_start > OVERRIDE_DURATION) {
    pump_override_active = false;
    pwm_override         = 0;
    Serial.println(F("[OVERRIDE] Pomp — vervallen na 60s, terug naar auto"));
    // Fan UI wordt bijgewerkt in update_matter_sensors() binnen 5s
  }
}


// =============================================================================
// Matter sensor updates — elke 5s
//
// REGEL: setSpeedPercent() mag hier (feedback, geen gebruikersingreep).
//        setMode() wordt hier NIET aangeroepen — enkel in callbacks en
//        check_overrides() — zelfde principe als setHeatingSetpoint() in v4.
// =============================================================================
void update_matter_sensors() {
  // Temperatuursensoren
  matter_tsun.setTemperature(Tsun);
  matter_etoph.setTemperature(ETopH);
  matter_eboth.setTemperature(EBotH);
  matter_eqpct.setHumidity(eq_to_pct(EQtot));

  // Fan: snelheidsfeedback = werkelijke pwm_value teruggemapped naar %
  // ignore_callbacks: voorkomt dat setSpeedPercent() de onChangeSpeedPercent
  // callback triggert en een nieuwe override aanmaakt
  ignore_callbacks = true;
  matter_pomp.setSpeedPercent(pwm_to_pct(pwm_value));
  ignore_callbacks = false;
}


// =============================================================================
// Serial statusrapport
// =============================================================================
void print_status() {
  const char* ovr = pump_override_active ? "OVR" : "AUT";
  Serial.println(F("\n╔══════════════════════════════════════════╗"));
  Serial.println(F(  "║  ESP32-C6_ECO-boiler_SIM v6.3 – Status  ║"));
  Serial.printf(     "║  Uptime : %lu s\n", millis() / 1000);
  Serial.println(F(  "╠══════════════════════════════════════════╣"));
  Serial.println(F(  "║ TEMPERATUREN                              "));
  Serial.printf(     "║  Tsun  (collector) : %6.1f °C\n", Tsun);
  Serial.printf(     "║  ETopH (top)       : %6.1f °C\n", ETopH);
  Serial.printf(     "║  EBotH (bodem)     : %6.1f °C\n", EBotH);
  Serial.printf(     "║  dT  (Tsun-EBotH)  : %6.1f °C\n", dT);
  Serial.printf(     "║  EAv (gemiddeld)   : %6.1f °C\n", EAv);
  Serial.printf(     "║  EQtot             : %6.2f kWh (%d%%)\n", EQtot, eq_to_pct(EQtot));
  Serial.println(F(  "╠══════════════════════════════════════════╣"));
  Serial.println(F(  "║ POMP                                      "));
  Serial.printf(     "║  pump_relay : %s [%s]\n", pump_relay ? "AAN" : "UIT", ovr);
  Serial.printf(     "║  pwm_value  : %d  (%d%%)\n", pwm_value, pwm_to_pct(pwm_value));
  if (pump_override_active) {
    unsigned long rem = (OVERRIDE_DURATION - (millis() - pump_override_start)) / 1000UL;
    Serial.printf(   "║  override   : %lus resterend\n", rem);
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
  Serial.println(F("\n=== ESP32-C6_ECO-boiler_SIM v6.3 ==="));

  // ── WiFi ──────────────────────────────────────────────────────────────────
  Serial.printf("WiFi: verbinden met '%s' ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nWiFi OK: %s\n", WiFi.localIP().toString().c_str());

  // ── EQ_MAX berekenen (dynamisch op basis van volume + temp drempel) ─────────
  EQ_MAX = calcEQmax();
  Serial.printf("EQ_MAX = %.1f kWh (%.0fL, Tsun_high=%.0f°C)\n",
                EQ_MAX, BOILER_VOLUME_TOTAL, TSUN_HIGH);

  // ── Temperatuursensoren ────────────────────────────────────────────────────
  matter_tsun.begin();
  matter_etoph.begin();
  matter_eboth.begin();
  matter_eqpct.begin();

  // ── MatterFan: pompbediening ───────────────────────────────────────────────
  // FAN_MODE_SEQ_OFF_HIGH: enkel OFF en HIGH beschikbaar als modus-stappen
  // → eenvoudigste sequentie, geen LOW/MED/AUTO verwarring in Home app
  // Begin met speed 0, mode OFF
  matter_pomp.begin(0, MatterFan::FAN_MODE_OFF, MatterFan::FAN_MODE_SEQ_OFF_HIGH);

  // Speed slider bewogen in Home app → PWM override
  matter_pomp.onChangeSpeedPercent([](uint8_t new_pct) -> bool {
    if (ignore_callbacks) return true;

    int pwm = pct_to_pwm(new_pct);
    pwm_override = pwm;

    if (pwm == 0) {
      // Snelheid naar 0 → terug naar auto
      pump_override_active = false;
      Serial.println(F("[HomeKit] Pomp speed = 0% → terug naar auto"));
    } else {
      // Override activeren — starttijd EERST zetten voor de vlag,
      // anders kan check_overrides() de vlag direct wissen (race condition)
      pump_override_start  = millis();
      pump_override_active = true;
      Serial.printf("[HomeKit] Pomp speed override → %d%% = PWM %d\n", new_pct, pwm);
    }
    return true;
  });

  // Mode gewijzigd in Home app (OFF-knop of modus-selectie)
  matter_pomp.onChangeMode([](uint8_t new_mode) -> bool {
    if (ignore_callbacks) return true;

    if (new_mode == MatterFan::FAN_MODE_OFF) {
      // Expliciet UIT gezet → override wissen, terug naar auto
      pump_override_active = false;
      pwm_override         = 0;
      ignore_callbacks = true;
      matter_pomp.setSpeedPercent(0);
      ignore_callbacks = false;
      Serial.println(F("[HomeKit] Pomp mode OFF → terug naar auto, speed=0"));
    } else {
      // Mode AAN (HIGH of ON) zonder specifieke speed → gebruik PWM_MIN
      if (!pump_override_active) {
        pwm_override         = PWM_MIN;
        pump_override_start  = millis();   // EERST
        pump_override_active = true;       // DAN
        ignore_callbacks = true;
        matter_pomp.setSpeedPercent(pwm_to_pct(PWM_MIN));
        ignore_callbacks = false;
        Serial.printf("[HomeKit] Pomp mode AAN → override PWM=%d (%d%%)\n",
                      PWM_MIN, pwm_to_pct(PWM_MIN));
      }
    }
    return true;
  });

  // ── Matter starten ────────────────────────────────────────────────────────
  Matter.begin();

  // ── Pairing check ─────────────────────────────────────────────────────────
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

  sim_step();
  update_matter_sensors();
  print_status();
}


// =============================================================================
// LOOP
// =============================================================================
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("reset")) {
      Serial.println(F("Matter pairing wissen + reboot..."));
      nvs_flash_erase();
      delay(300);
      ESP.restart();
    }
    if (cmd.equalsIgnoreCase("status")) print_status();
  }

  // check_overrides() vóór de 5s timer: als vlag net gewist wordt,
  // pusht update_matter_sensors() onmiddellijk de correcte auto-toestand.
  check_overrides();

  if (millis() - last_sim_step > 5000) {
    last_sim_step = millis();
    sim_step();
    update_matter_sensors();
  }

  if (millis() - last_serial_log > 15000) {
    last_serial_log = millis();
    print_status();
  }
}
