// =============================================================================
// ESP32-C6_ECO-boiler_MATTER_SIM.ino  –  v4  –  28feb26 20:00
// Simulatie van ECO boiler Matter/HomeKit integratie voor ESP32-C6
// Filip Delannoy – Zarlar thuisautomatisering
//
// WIJZIGINGEN v4 — volledige herschrijving, vereenvoudigd:
//   FIX KRITISCH: setHeatingSetpoint() volledig verwijderd uit update_matter_sensors()
//     → was oorzaak van "springt terug naar 27 en Verwarming ON":
//       elke 5s werd het setpoint overschreven, HomeKit reageerde erop
//     → setHeatingSetpoint() staat nu ENKEL in callbacks + check_overrides()
//       (identiek patroon als matter_vent_control in HVAC_SIM)
//   FIX KRITISCH: setLocalTemperature() altijd in 7–30 range via pwm_to_sp(pwm_value)
//     → ruwe 0–255 waarde liet HomeKit's interne thermostaat-logica ingrijpen
//     → nu: localTemp = pwm_to_sp(pwm_value) → zelfde schaal als setpoint → geen conflict
//   VEREENVOUDIGD: matter_pump_pwm sensor verwijderd (was redundant)
//     → localTemperature op de thermostat-tegel geeft al de feedback
//   VEREENVOUDIGD: update_matter_sensors() raakt thermostat setpoint niet aan
//
// EINDPUNTEN (6 totaal — past op 4MB Huge App):
//   4 × MatterTemperatureSensor  (Tsun, ETopH, EBotH, EQtot)
//   1 × MatterThermostat         (PWM-sturing,  hernoem "Pomp PWM")
//   1 × MatterOnOffLight         (pump_relay,   hernoem "Pomp relay")
//
// SCHAALFACTOR thermostat (identiek aan vent_control in HVAC_SIM):
//   Slider 7–30 → PWM 0–255 :  pwm = round((sp - 7) / 23 * 255)
//   PWM 0–255  → Slider 7–30:  sp  = (pwm / 255 * 23) + 7
//   Setpoint = 7 (minimum) = PWM 0 = auto modus
//   LocalTemp = pwm_to_sp(pwm_value) = feedback van werkelijke PWM op tegel
//
// INVARIANT: pump_relay == (pwm_value > 0)  — altijd, in alle paden.
//
// API: arduino-esp32-master 3.3.2
// HARDWARE: ESP32-C6, 4MB Huge App partition
// =============================================================================

#include <WiFi.h>
#include <nvs_flash.h>
#include <Matter.h>
#include <MatterEndPoints/MatterTemperatureSensor.h>
#include <MatterEndPoints/MatterOnOffLight.h>
#include <MatterEndPoints/MatterThermostat.h>


// =============================================================================
// WiFi — iPhone personal hotspot
// =============================================================================
const char* WIFI_SSID = "iPhoneFilip";
const char* WIFI_PASS = "adnoh123";


// =============================================================================
// Matter endpoints
// =============================================================================
MatterTemperatureSensor matter_tsun;          // Tsun  — collector (hernoem "°C Collector")
MatterTemperatureSensor matter_etoph;         // ETopH — bovenste laag (hernoem "°C Top")
MatterTemperatureSensor matter_eboth;         // EBotH — onderste inlaat (hernoem "°C Bodem")
MatterTemperatureSensor matter_eqtot;         // EQtot — kWh als "°C" (hernoem "kWh ECO")
MatterThermostat        matter_pump_control;  // PWM-sturing (hernoem "Pomp PWM")
MatterOnOffLight        matter_pump_relay;    // Schakelaar (hernoem "Pomp relay")


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

bool  pump_relay = false;
int   pwm_value  = 0;

// Drempelwaarden (identiek aan productie defaults)
const float DT_START = 3.0f;
const float DT_STOP  = 2.0f;
const int   PWM_MIN      = 80;
const int   PWM_MAX      = 200;
const int   PWM_OVERHEAT = 255;
const float TSUN_OVERHEAT = 90.0f;
const float TSUN_MIN      = 22.0f;   // thermosifon preventie

// Override
bool  pump_override_active = false;
int   pwm_override         = 0;
unsigned long pump_override_start = 0;
const unsigned long OVERRIDE_DURATION = 60UL * 1000UL;  // 60s

// Hysteresis
bool pump_on_auto = false;

// Flag: voorkomt feedback-loop bij programmatische Matter-calls
bool ignore_callbacks = false;

// Timers
unsigned long last_sim_step   = 0;
unsigned long last_serial_log = 0;


// =============================================================================
// Schaalconversie PWM ↔ thermostat setpoint (7–30 HomeKit range)
// =============================================================================
float pwm_to_sp(int pwm) {
  return (pwm / 255.0f * 23.0f) + 7.0f;
}

int sp_to_pwm(double sp) {
  return constrain((int)round((sp - 7.0) / 23.0 * 255.0), 0, 255);
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
  const float Cp        = 1.16f;
  const float Tref      = 20.0f;
  const float vol       = 490.0f / 6.0f;
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
// Override timeout — wist vlag + Serial
// Roept setHeatingSetpoint() aan zodat Home app de terugkeer naar auto ziet.
// =============================================================================
void check_overrides() {
  if (pump_override_active &&
      millis() - pump_override_start > OVERRIDE_DURATION) {
    pump_override_active = false;
    pwm_override         = 0;
    Serial.println(F("[OVERRIDE] Pomp — vervallen na 60s, terug naar auto"));
    ignore_callbacks = true;
    matter_pump_control.setHeatingSetpoint(7.0f);   // minimum = auto
    matter_pump_relay.setOnOff(false);              // voorlopig UIT; sim_step corrigeert
    ignore_callbacks = false;
  }
}


// =============================================================================
// Matter sensor updates — elke 5s
//
// REGEL (identiek aan HVAC_SIM): setHeatingSetpoint() staat NOOIT hier.
// Enkel sensoren + localTemperature + relay worden hier bijgewerkt.
// setHeatingSetpoint() wordt uitsluitend aangepast in:
//   - onChangeHeatingSetpoint callback (gebruikersingreep)
//   - onChangeMode callback (OFF-knop)
//   - check_overrides() (timer vervallen)
// =============================================================================
void update_matter_sensors() {
  // Temperatuursensoren
  matter_tsun.setTemperature(Tsun);
  matter_etoph.setTemperature(ETopH);
  matter_eboth.setTemperature(EBotH);
  matter_eqtot.setTemperature(EQtot);

  // Thermostat localTemp = werkelijke PWM teruggemapped naar 7–30
  // → zichtbaar als "huidige waarde" op de thermostat-tegel in Home app
  // → blijft in dezelfde schaal als het setpoint → geen conflict met HomeKit
  matter_pump_control.setLocalTemperature(pwm_to_sp(pwm_value));

  // Relay spiegelen (programmatisch → ignore_callbacks)
  ignore_callbacks = true;
  matter_pump_relay.setOnOff(pump_relay);
  ignore_callbacks = false;
}


// =============================================================================
// Serial statusrapport
// =============================================================================
void print_status() {
  const char* ovr = pump_override_active ? "OVR" : "AUT";
  Serial.println(F("\n╔══════════════════════════════════════════╗"));
  Serial.println(F(  "║  ESP32-C6_ECO-boiler_SIM v4  –  Status  ║"));
  Serial.printf(     "║  Uptime : %lu s\n", millis() / 1000);
  Serial.println(F(  "╠══════════════════════════════════════════╣"));
  Serial.println(F(  "║ TEMPERATUREN                              "));
  Serial.printf(     "║  Tsun  (collector) : %6.1f °C\n", Tsun);
  Serial.printf(     "║  ETopH (top)       : %6.1f °C\n", ETopH);
  Serial.printf(     "║  EBotH (bodem)     : %6.1f °C\n", EBotH);
  Serial.printf(     "║  dT  (Tsun-EBotH)  : %6.1f °C\n", dT);
  Serial.printf(     "║  EAv (gemiddeld)   : %6.1f °C\n", EAv);
  Serial.printf(     "║  EQtot             : %6.2f kWh\n", EQtot);
  Serial.println(F(  "╠══════════════════════════════════════════╣"));
  Serial.println(F(  "║ POMP                                      "));
  Serial.printf(     "║  pump_relay : %s [%s]\n", pump_relay ? "AAN" : "UIT", ovr);
  Serial.printf(     "║  pwm_value  : %d\n", pwm_value);
  Serial.printf(     "║  sp_feedback: %.2f (slider)\n", pwm_to_sp(pwm_value));
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
  Serial.println(F("\n=== ESP32-C6_ECO-boiler_SIM v4 ==="));

  // ── WiFi ──────────────────────────────────────────────────────────────────
  Serial.printf("WiFi: verbinden met '%s' ...\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nWiFi OK: %s\n", WiFi.localIP().toString().c_str());

  // ── Temperatuursensoren ────────────────────────────────────────────────────
  matter_tsun.begin();
  matter_etoph.begin();
  matter_eboth.begin();
  matter_eqtot.begin();

  // ── Thermostat: PWM-sturing ────────────────────────────────────────────────
  // ControlSequenceOfOperation 2 = heating only
  // Setpoint minimum (7) = auto modus, maximum (30) = PWM 255
  matter_pump_control.begin((MatterThermostat::ControlSequenceOfOperation_t)2);
  matter_pump_control.setLocalTemperature(7.0f);  // pwm=0 → sp=7
  matter_pump_control.setHeatingSetpoint(7.0f);   // start: auto

  // Slider bewogen in Home app
  matter_pump_control.onChangeHeatingSetpoint([](double new_sp) -> bool {
    if (ignore_callbacks) return true;

    int pwm = sp_to_pwm(new_sp);
    pwm_override = pwm;

    if (pwm == 0) {
      // Setpoint op minimum → terug naar auto
      pump_override_active = false;
      ignore_callbacks = true;
      matter_pump_relay.setOnOff(false);   // direct UIT (auto corrigeert daarna)
      ignore_callbacks = false;
      Serial.println(F("[HomeKit] Pomp setpoint = 7 → auto, relay UIT"));
    } else {
      // Override: pomp op gevraagd PWM
      pump_override_active = true;
      pump_override_start  = millis();
      ignore_callbacks = true;
      matter_pump_relay.setOnOff(true);    // INVARIANT: pwm>0 → relay AAN
      ignore_callbacks = false;
      Serial.printf("[HomeKit] Pomp PWM override → slider %.1f = PWM %d\n", new_sp, pwm);
    }
    return true;
  });

  // OFF-knop op thermostat → override wissen, terug naar auto
  matter_pump_control.onChangeMode([](uint8_t mode) -> bool {
    if (ignore_callbacks) return true;
    if (mode == 0) {
      pump_override_active = false;
      pwm_override         = 0;
      ignore_callbacks = true;
      matter_pump_control.setHeatingSetpoint(7.0f);
      matter_pump_relay.setOnOff(false);
      ignore_callbacks = false;
      Serial.println(F("[HomeKit] Pomp mode OFF → auto, relay UIT"));
    }
    return true;
  });

  // ── Relay switch: bidirectioneel ───────────────────────────────────────────
  matter_pump_relay.begin();
  matter_pump_relay.setOnOff(false);

  matter_pump_relay.onChangeOnOff([](bool on_off) -> bool {
    if (ignore_callbacks) return true;

    pump_override_active = true;
    pump_override_start  = millis();

    if (on_off) {
      // AAN via switch → PWM_MIN als startwaarde, thermostat meezetten
      pwm_override = PWM_MIN;
      ignore_callbacks = true;
      matter_pump_control.setHeatingSetpoint(pwm_to_sp(PWM_MIN));
      ignore_callbacks = false;
      Serial.printf("[HomeKit] Relay AAN → PWM=%d (slider %.1f)\n",
                    PWM_MIN, pwm_to_sp(PWM_MIN));
    } else {
      // UIT via switch → PWM 0, thermostat naar minimum
      pwm_override = 0;
      ignore_callbacks = true;
      matter_pump_control.setHeatingSetpoint(7.0f);
      ignore_callbacks = false;
      Serial.println(F("[HomeKit] Relay UIT → PWM=0 (60s override)"));
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

  // check_overrides() vóór de 5s timer: als de vlag net gewist wordt,
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
