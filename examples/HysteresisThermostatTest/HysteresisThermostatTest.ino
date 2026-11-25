#include <Arduino.h>
#include <BitRegisterIOAdapter.h>
#include <SerialLogger.h>
#include <TemperatureFake.h>
#include <ThermostatHysteresis.h>

namespace {

// Mega/Uno LED: active HIGH. Change this for an active-LOW LED or relay.
constexpr bool kInvertOutput = false;
constexpr float kMinimumTemperatureC = 20.0f;
constexpr float kMaximumTemperatureC = 24.0f;
constexpr float kTemperatureStepC = 0.5f;
constexpr uint32_t kInputIntervalSeconds = 4;
constexpr uint32_t kErrorHoldSeconds = 8;

enum class DemoPhase : uint8_t {
  kStartup,
  kRampDown,
  kRampUp,
  kBeforeError,
  kReadingError,
  kRecovering
};

// Logged objects are constructed in setup(), after Serial.begin().
TemperatureFake* g_temperature = nullptr;
ThermostatHysteresis* g_thermostat = nullptr;
BitRegisterIOAdapter* g_led_output = nullptr;
bool g_initialized = false;
DemoPhase g_demo_phase = DemoPhase::kStartup;
float g_simulated_temperature_c = kMaximumTemperatureC;
uint32_t g_completed_cycles = 0;

// Raw clock samples use millis() for rollover-safe timing; durations use
// seconds.
uint32_t g_last_input_at_ms = 0;
uint32_t g_error_started_at_ms = 0;
uint32_t g_last_failure_report_at_ms = 0;

void PrintStatus() {
  const ThermostatHysteresis::State state = g_thermostat->GetState();
  const TemperatureBase::State sensor = g_temperature->GetState();
  Serial.print(F("[Demo] t="));
  Serial.print(millis() / 1000UL);
  Serial.print(F(" s, cycles="));
  Serial.print(g_completed_cycles);
  Serial.print(F(", temperature="));
  Serial.print(sensor.reading.temperatureC, 2);
  Serial.print(sensor.status == TemperatureBase::Status::Ok
                   ? F(" C")
                   : F(" C (last good)"));
  Serial.print(F(", setpoint="));
  Serial.print(g_thermostat->GetConfig().target_c, 2);
  Serial.print(F(" C, status="));
  Serial.print(ThermostatBase::StatusText(state.status));
  Serial.print(F(", demand="));
  Serial.print(state.demand ? F("ON") : F("OFF"));
  Serial.print(F(", output="));
  Serial.print(state.output ? F("ON") : F("OFF"));
  Serial.print(F(", LED bit="));
  Serial.println(g_led_output->Get() ? F("ON") : F("OFF"));
}

void OnThermostatUpdate(const ThermostatHysteresis&,
                        const ThermostatHysteresis::State& state, void*) {
  if (state.startup_testing) {
    Serial.println(F("[Demo] Startup test: forced ON for 3 s."));
  } else if (state.pending) {
    Serial.println(
        state.demand ? F("[Demo] ON delay: heating requested, waiting 2 s.")
                     : F("[Demo] OFF delay: heating satisfied, waiting 2 s."));
  } else if (state.status == ThermostatBase::Status::kSensorError) {
    Serial.println(F("[Demo] Reading error: configured error output is ON."));
  } else {
    Serial.print(F("[Demo] Logical LED command: "));
    Serial.println(state.output ? F("ON") : F("OFF"));
  }
}

bool StartDemo(uint32_t now_ms) {
  ThermostatHysteresis::Config config;
  config.mode = ThermostatBase::Mode::kHeating;
  config.target_c = 22.0f;
  config.hysteresis_c = 1.0f;  // ON below 21.5 C; OFF at or above 22.5 C.
  config.minimum_target_c = 10.0f;
  config.maximum_target_c = 30.0f;
  config.evaluation_interval_seconds =
      0;  // Evaluate each loop; input every 4 s.
  config.turn_on_delay_seconds = 2;
  config.turn_off_delay_seconds = 2;
  config.startup_test_seconds = 3;
  config.maximum_reading_age_seconds = 8;
  config.initial_demand = false;
  config.initial_output = false;
  config.error_output = true;

  if (!g_thermostat->SetConfig(config) || !g_temperature->Begin()) {
    return false;
  }
  g_simulated_temperature_c = kMaximumTemperatureC;
  g_demo_phase = DemoPhase::kStartup;
  g_completed_cycles = 0;
  g_last_input_at_ms = now_ms;
  g_error_started_at_ms = now_ms;

  // A healthy warm reading makes the startup test independent of heating
  // demand.
  g_temperature->SetTemperature(g_simulated_temperature_c);
  g_temperature->Update(now_ms);
  if (!g_thermostat->Begin(now_ms)) {
    return false;
  }
  g_thermostat->Update(now_ms);
  Serial.println(F("[Demo] Holding 24 C through startup and the OFF delay."));
  return true;
}

void AdvanceInput(uint32_t now_ms) {
  if (g_demo_phase == DemoPhase::kStartup) {
    const ThermostatHysteresis::State state = g_thermostat->GetState();
    if (state.startup_testing || state.pending) {
      g_temperature->SetTemperature(g_simulated_temperature_c);
      return;
    }
    g_demo_phase = DemoPhase::kRampDown;
    Serial.println(F("[Demo] Ramp down: -0.5 C every 4 s."));
  }

  switch (g_demo_phase) {
    case DemoPhase::kRampDown:
      g_simulated_temperature_c -= kTemperatureStepC;
      if (g_simulated_temperature_c <= kMinimumTemperatureC) {
        g_simulated_temperature_c = kMinimumTemperatureC;
        g_demo_phase = DemoPhase::kRampUp;
        Serial.println(F("[Demo] Minimum reached. Ramp up: +0.5 C every 4 s."));
      }
      g_temperature->SetTemperature(g_simulated_temperature_c);
      break;

    case DemoPhase::kRampUp:
      g_simulated_temperature_c += kTemperatureStepC;
      if (g_simulated_temperature_c >= kMaximumTemperatureC) {
        g_simulated_temperature_c = kMaximumTemperatureC;
        ++g_completed_cycles;
        g_demo_phase = DemoPhase::kBeforeError;
        Serial.println(
            F("[Demo] Full down/up cycle complete; hold warm for 4 s."));
      }
      g_temperature->SetTemperature(g_simulated_temperature_c);
      break;

    case DemoPhase::kBeforeError:
      // At 24 C normal heating is OFF. The next error visibly forces it ON.
      g_demo_phase = DemoPhase::kReadingError;
      g_error_started_at_ms = now_ms;
      g_temperature->SetError(TemperatureBase::Status::InvalidReading);
      Serial.println(
          F("[Demo] Inject reading error for 8 s; error output is ON."));
      break;

    case DemoPhase::kReadingError:
      if (static_cast<uint32_t>(now_ms - g_error_started_at_ms) >=
          kErrorHoldSeconds * 1000UL) {
        // SetTemperature() clears the fake's configured error.
        g_temperature->SetTemperature(g_simulated_temperature_c);
        g_demo_phase = DemoPhase::kRecovering;
        Serial.println(
            F("[Demo] Fresh 24 C reading: recover, then OFF after 2 s."));
      } else {
        g_temperature->SetError(TemperatureBase::Status::InvalidReading);
      }
      break;

    case DemoPhase::kRecovering:
      g_demo_phase = DemoPhase::kRampDown;
      g_simulated_temperature_c -= kTemperatureStepC;
      g_temperature->SetTemperature(g_simulated_temperature_c);
      Serial.println(F("[Demo] New cycle. Ramp down: -0.5 C every 4 s."));
      break;

    case DemoPhase::kStartup:
      break;
  }
}

void HandleSerial(uint32_t now_ms) {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    if (command == 'r') {
      g_initialized = StartDemo(now_ms);
    } else if (!g_initialized) {
      continue;
    } else if (command == '+' || command == '-') {
      const float step =
          command == '+' ? kTemperatureStepC : -kTemperatureStepC;
      g_thermostat->SetTargetC(g_thermostat->GetConfig().target_c + step);
      Serial.print(F("[Demo] Setpoint: "));
      Serial.println(g_thermostat->GetConfig().target_c, 2);
    } else if (command == 'e' || command == 'd') {
      g_thermostat->SetEnabled(command == 'e');
      Serial.println(command == 'e' ? F("[Demo] Enabled.")
                                    : F("[Demo] Disabled."));
    } else if (command == '?') {
      PrintStatus();
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  Serial.println(F("\nHeating thermostat: 24 -> 20 -> 24 C, 0.5 C every 4 s."));
  Serial.println(F("Startup test = 3 s; normal ON/OFF delays = 2 s each."));
  Serial.println(F("Initial LED pulse = 3 s test + 2 s OFF delay."));
  Serial.println(F("Default setpoint = 22 C; hysteresis width = 1 C."));

  // Debug is required: TemperatureBase reports accepted readings at Debug
  // level. Always enabled for both libraries, including when compiling for Uno.
  static SerialLogger logger(Serial, LogLevel::Debug);
  // Zero interval: acquire only when this demo explicitly calls Update().
  static TemperatureFake sensor(&logger, "Room", 0);
  static BitRegisterIOAdapter led(
      LED_BUILTIN, BitRegisterIOAdapter::OUTPUT_MODE, &logger, "LED", kInvertOutput);
  static ThermostatHysteresis controller(sensor, led, &logger, "Heating");

  g_temperature = &sensor;
  g_thermostat = &controller;
  g_led_output = &led;

  if (!controller.OnUpdate(OnThermostatUpdate)) {
    Serial.println(F("FAILED: thermostat callback registration"));
    return;
  }
  g_initialized = StartDemo(millis());
  if (!g_initialized) {
    Serial.println(F("FAILED: thermostat initialization"));
    return;
  }
  PrintStatus();
}

void loop() {
  const uint32_t now_ms = millis();
  HandleSerial(now_ms);
  if (!g_initialized) {
    if (static_cast<uint32_t>(now_ms - g_last_failure_report_at_ms) >= 2000UL) {
      g_last_failure_report_at_ms = now_ms;
      Serial.println(F("Initialization failed. Send r to retry."));
    }
    return;
  }

  const bool input_due = static_cast<uint32_t>(now_ms - g_last_input_at_ms) >=
                         kInputIntervalSeconds * 1000UL;
  if (input_due) {
    g_last_input_at_ms = now_ms;
    AdvanceInput(now_ms);
    g_temperature->Update(now_ms);
  }

  // Keep servicing the controller between samples: never block with
  // delay(4000).
  g_thermostat->Update(now_ms);
  if (input_due) {
    PrintStatus();
  }
}
