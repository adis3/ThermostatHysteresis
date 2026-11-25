#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <string>

#include <TemperatureFake.h>
#include <ThermostatHysteresis.h>
#define THERMOSTAT_HYSTERESIS_TEST_CHECK(condition)                \
  do {                                                             \
    if (!(condition)) {                                            \
      fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
      exit(1);                                                     \
    }                                                              \
  } while (0)

namespace {

int g_pin_levels[256] = {};
int g_pin_write_counts[256] = {};
}  // namespace

void pinMode(uint8_t, int) {}
void digitalWrite(uint8_t pin, uint8_t value) {
  g_pin_levels[pin] = value;
  ++g_pin_write_counts[pin];
}
int digitalRead(uint8_t pin) { return g_pin_levels[pin]; }

namespace {

using Thermostat = ThermostatHysteresis;
using Status = ThermostatBase::Status;
class TestLogger final : public ILogger {
 public:
  void Log(LogLevel level, const char*, const char* message) override {
    if (level == LogLevel::Error) {
      ++error_count_;
    }
    messages_ += message;
    messages_ += '\n';
  }
  int error_count() const { return error_count_; }
  const std::string& messages() const { return messages_; }

 private:
  int error_count_ = 0;
  std::string messages_;
};
class Fixture {
 public:
  Fixture(bool inverted = false)
      : sensor_(&logger_, "sensor", 0),
        bit_(9, BitRegisterIOAdapter::OUTPUT_MODE, &logger_, "bit", inverted),
        thermostat_(sensor_, bit_, &logger_, "thermostat") {
    THERMOSTAT_HYSTERESIS_TEST_CHECK(sensor_.Begin());
    Thermostat::Config config;
    config.hysteresis_c = 1.0f;
    config.evaluation_interval_seconds = 0;
    THERMOSTAT_HYSTERESIS_TEST_CHECK(thermostat_.SetConfig(config));
  }
  void Input(float value, uint32_t now_ms) {
    sensor_.SetTemperature(value);
    sensor_.Update(now_ms);
  }
  void Tick(float value, uint32_t now_ms) {
    Input(value, now_ms);
    thermostat_.Update(now_ms);
  }
  void Start(float value, uint32_t now_ms = 0) {
    Input(value, now_ms);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(thermostat_.Begin(now_ms));
    thermostat_.Update(now_ms);
  }
  TemperatureFake& sensor() { return sensor_; }
  BitRegisterIOAdapter& bit() { return bit_; }
  Thermostat& thermostat() { return thermostat_; }

 private:
  TestLogger logger_;
  TemperatureFake sensor_;
  BitRegisterIOAdapter bit_;
  Thermostat thermostat_;
};
int g_test_group_count = 0;
void TestAdapterValidation() {
  TestLogger logger;
  TemperatureFake sensor;
  BitRegisterIOAdapter input(7, BitRegisterIOAdapter::INPUT_MODE);
  Thermostat invalid(sensor, input, &logger);
  const int before = g_pin_write_counts[7];
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!invalid.Begin(0));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(invalid.GetStatus() ==
                                   Status::kInitializationFailed);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(logger.error_count() == 1);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      logger.messages().find("OUTPUT_MODE or virtual") != std::string::npos);
  invalid.Update(1000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(g_pin_write_counts[7] == before);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!input.Get());

  BitRegisterIOAdapter hardware(8, BitRegisterIOAdapter::OUTPUT_MODE);
  Thermostat valid(sensor, hardware);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(valid.Begin(0));
  BitRegisterIOAdapter virtual_bit(false);
  Thermostat virtual_output(sensor, virtual_bit);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(virtual_output.Begin(0));
  ++g_test_group_count;
}
void TestInitialAndAdapterPolarity() {
  for (int initial = 0; initial < 2; ++initial) {
    for (int invert = 0; invert < 2; ++invert) {
      Fixture fixture(invert != 0);
      auto config = fixture.thermostat().GetConfig();
      config.initial_output = initial != 0;
      THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
      THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().Begin(1234));
      auto state = fixture.thermostat().GetState();
      THERMOSTAT_HYSTERESIS_TEST_CHECK(state.status == Status::kStarting &&
                                       state.output == config.initial_output);
      THERMOSTAT_HYSTERESIS_TEST_CHECK(state.output_changed_at_seconds == 1);
      THERMOSTAT_HYSTERESIS_TEST_CHECK(
          fixture.bit().Get() ==
          config.initial_output);
      THERMOSTAT_HYSTERESIS_TEST_CHECK(g_pin_levels[9] ==
          ((config.initial_output != (invert != 0)) ? HIGH : LOW));
      fixture.thermostat().Update(1234);
      THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetStatus() ==
                                       Status::kNoData);
      THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.bit().Get() ==
                                       false);
      THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().Begin(2000));
      THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output ==
                                       config.initial_output);
    }
  }
  ++g_test_group_count;
}
void TestThresholds() {
  Fixture fixture;
  fixture.Start(20.5f);  // At lower boundary with initial_demand=false.
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().output);
  fixture.Tick(20.49f, 1);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output);
  fixture.Tick(21.0f, 2);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output);
  fixture.Tick(21.5f, 3);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().output);
  fixture.Tick(20.5f, 4);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().output);

  auto config = fixture.thermostat().GetConfig();
  config.mode = Thermostat::Mode::kCooling;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  fixture.Tick(20.5f, 5);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().output);
  fixture.Tick(21.5f, 6);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output);
  fixture.Tick(21.0f, 7);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output);
  fixture.Tick(20.49f, 8);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().output);
  config.hysteresis_c = 0;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  fixture.Tick(21.0f, 9);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output);
  ++g_test_group_count;
}
void TestDelaysAndCancellation() {
  Fixture fixture;
  auto config = fixture.thermostat().GetConfig();
  config.turn_on_delay_seconds = 2;
  config.turn_off_delay_seconds = 3;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  fixture.Start(19);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().pending &&
                                   !fixture.bit().Get());
  fixture.Tick(19, 1999);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.bit().Get());
  fixture.Tick(19, 2000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.bit().Get());
  fixture.Tick(24, 2001);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().pending &&
                                   fixture.bit().Get());
  fixture.Tick(19, 3000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().pending &&
                                   fixture.bit().Get());
  fixture.Tick(24, 4000);
  fixture.Tick(24, 6999);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.bit().Get());
  fixture.Tick(24, 7000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.bit().Get());
  ++g_test_group_count;
}
void TestExpiryRechecksInput() {
  Fixture fixture;
  auto config = fixture.thermostat().GetConfig();
  config.turn_on_delay_seconds = 2;
  config.evaluation_interval_seconds = 10;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  fixture.Start(19);
  fixture.Tick(24, 2000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.bit().Get() &&
                                   !fixture.thermostat().GetState().pending);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().demand);
  ++g_test_group_count;
}
void TestFaultAndRecovery() {
  for (int inverted = 0; inverted < 2; ++inverted) {
    Fixture fixture(inverted != 0);
    auto config = fixture.thermostat().GetConfig();
    config.turn_on_delay_seconds = 2;
    config.turn_off_delay_seconds = 2;
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
    fixture.Start(19);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(
        fixture.sensor().SetError(TemperatureBase::Status::Disconnected));
    fixture.sensor().Update(2000);
    fixture.thermostat().Update(2000);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetStatus() ==
                                     Status::kSensorError);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().output &&
                                     !fixture.thermostat().GetState().pending);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.bit().Get() ==
                                     false);
    fixture.Tick(19, 3000);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().pending);
    fixture.Tick(19, 4999);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().output);
    fixture.Tick(19, 5000);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output);

    config.error_output = true;
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
    THERMOSTAT_HYSTERESIS_TEST_CHECK(
        fixture.sensor().SetError(TemperatureBase::Status::Timeout));
    fixture.sensor().Update(5001);
    fixture.thermostat().Update(5001);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output);
    fixture.Tick(24, 6000);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().pending);
    fixture.Tick(24, 7999);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output);
    fixture.Tick(24, 8000);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().output);
  }
  ++g_test_group_count;
}
void TestStaleAndInvalid() {
  Fixture fixture;
  auto config = fixture.thermostat().GetConfig();
  config.maximum_reading_age_seconds = 2;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  fixture.Start(19);
  fixture.thermostat().Update(2000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetStatus() ==
                                   Status::kOk);
  fixture.thermostat().Update(2001);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetStatus() ==
                                       Status::kStaleReading &&
                                   !fixture.bit().Get());
  fixture.Tick(19, 3000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.bit().Get());
  fixture.Tick(NAN, 3001);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetStatus() ==
                                       Status::kSensorError &&
                                   !fixture.bit().Get());
  ++g_test_group_count;
}
void TestConfiguration() {
  Fixture fixture;
  auto config = fixture.thermostat().GetConfig();
  config.turn_on_delay_seconds = 2;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  fixture.Start(19);
  fixture.Tick(19, 1000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(
      config));  // Equal config must not restart the timer.
  auto invalid = config;
  invalid.turn_on_delay_seconds = Thermostat::kMaximumDurationSeconds + 1;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().SetConfig(invalid));
  invalid = config;
  invalid.hysteresis_c = -1;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().SetConfig(invalid));
  invalid = config;
  invalid.target_c = INFINITY;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().SetConfig(invalid));
  fixture.Tick(19, 2000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.bit().Get());
  ThermostatBase& base = fixture.thermostat();
  THERMOSTAT_HYSTERESIS_TEST_CHECK(base.SetTargetC(100));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(base.GetConfig().target_c ==
                                   config.maximum_target_c);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.thermostat().GetConfig().turn_on_delay_seconds == 2);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!base.SetTargetC(NAN));
  ++g_test_group_count;
}
void TestSetpointMemoryAndDisable() {
  Fixture fixture;
  fixture.Start(19);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetTargetC(21.1f));
  fixture.Tick(21.1f, 1000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().demand &&
                                   fixture.bit().Get());

  auto config = fixture.thermostat().GetConfig();
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  fixture.Tick(21.1f, 1001);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output &&
                                   fixture.bit().Get());
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetEnabled(false));
  fixture.thermostat().Update(1002);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetStatus() ==
                                   Status::kDisabled);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().output &&
                                   !fixture.bit().Get());
  ++g_test_group_count;
}
void TestStartup() {
  Fixture fixture;
  auto config = fixture.thermostat().GetConfig();
  config.startup_test_seconds = 2;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().Begin(0));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      !fixture.bit().Get());  // Deterministic initial output.
  fixture.Tick(24, 5000);     // Delayed first Update starts a full test.
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.bit().Get() && fixture.thermostat().GetState().startup_testing);
  fixture.Tick(24, 6999);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.bit().Get());
  fixture.Tick(24, 7000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      !fixture.bit().Get() && !fixture.thermostat().GetState().startup_testing);

  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().Begin(8000));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.sensor().SetError(TemperatureBase::Status::Disconnected));
  fixture.sensor().Update(8000);
  fixture.thermostat().Update(8000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.bit().Get());
  fixture.Tick(24, 9000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      !fixture.thermostat().GetState().startup_testing && !fixture.bit().Get());
  ++g_test_group_count;
}
void TestRollover() {
  Fixture fixture;
  auto config = fixture.thermostat().GetConfig();
  config.turn_on_delay_seconds = 2;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  const uint32_t start = 0xFFFFFF00U;
  fixture.Start(19, start);
  fixture.Tick(19, static_cast<uint32_t>(start + 1999U));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.bit().Get());
  fixture.Tick(19, static_cast<uint32_t>(start + 2000U));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.bit().Get());

  config.startup_test_seconds = 2;
  config.turn_on_delay_seconds = 0;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().Begin(start));
  fixture.Tick(24, start);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.thermostat().GetState().startup_testing);
  fixture.Tick(24, static_cast<uint32_t>(start + 2000U));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      !fixture.thermostat().GetState().startup_testing && !fixture.bit().Get());
  ++g_test_group_count;
}
struct ObserverContext {
  Thermostat* thermostat;
  int first;
  int later;
};
void LaterObserver(const Thermostat&, const Thermostat::State&, void* context) {
  ++static_cast<ObserverContext*>(context)->later;
}
void FirstObserver(const Thermostat& thermostat, const Thermostat::State& state,
                   void* context) {
  auto& config = *static_cast<ObserverContext*>(context);
  ++config.first;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(thermostat.GetState().status ==
                                   state.status);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!config.thermostat->SetEnabled(false));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!config.thermostat->Begin(0));
  config.thermostat->Update(0);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      config.thermostat->RemoveUpdateCallback(FirstObserver, context));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      config.thermostat->OnUpdate(LaterObserver, context));
}
void TestObservers() {
  Fixture fixture;
  ObserverContext context{&fixture.thermostat(), 0, 0};
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.thermostat().OnUpdate(FirstObserver, &context));
  fixture.Input(19, 0);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().Begin(0));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(context.first == 1 && context.later == 0);
  fixture.thermostat().Update(0);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(context.first == 1 && context.later == 1);
  fixture.thermostat().Update(0);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(context.later == 1);
  fixture.thermostat().ClearCallbacks();
  ++g_test_group_count;
}
Thermostat* g_raw_thermostat = nullptr;
int g_raw_callback_count = 0;
void RawObserver(bool raw, uint8_t) {
  ++g_raw_callback_count;
  const auto state = g_raw_thermostat->GetState();
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      raw == state.output);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!state.pending);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!g_raw_thermostat->SetEnabled(false));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!g_raw_thermostat->Begin(0));
}
void TestAdapterReentry() {
  Fixture fixture;
  auto config = fixture.thermostat().GetConfig();
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  g_raw_thermostat = &fixture.thermostat();
  g_raw_callback_count = 0;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.bit().OnStateChange(RawObserver));
  fixture.Start(19);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(g_raw_callback_count == 1);
  fixture.bit().ClearCallbacks();
  g_raw_thermostat = nullptr;
  ++g_test_group_count;
}

void TestPendingDemandReversals() {
  // Reversals must cancel both directions even with a long evaluation interval.
  for (int initially_on = 0; initially_on < 2; ++initially_on) {
    Fixture fixture;
    auto config = fixture.thermostat().GetConfig();
    config.initial_output = initially_on != 0;
    config.evaluation_interval_seconds = 10;
    config.turn_on_delay_seconds = 2;
    config.turn_off_delay_seconds = 2;
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
    const float original_temperature = initially_on ? 24.0f : 19.0f;
    const float reversing_temperature = initially_on ? 19.0f : 24.0f;
    fixture.Start(original_temperature);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().pending);
    fixture.Tick(reversing_temperature, 1000);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().pending);
    fixture.Tick(original_temperature, 1500);
    fixture.Tick(original_temperature, 2000);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output ==
                                     (initially_on != 0));
    // Following cancellation, ordinary evaluation scheduling resumes.
    fixture.Tick(original_temperature, 11000);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().pending);
    fixture.Tick(original_temperature, 12999);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output ==
                                     (initially_on != 0));
    fixture.Tick(original_temperature, 13000);
    THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output !=
                                     (initially_on != 0));
  }
  ++g_test_group_count;
}

void TestRestartPreservesOutputTimestamp() {
  Fixture fixture;
  fixture.Start(24.0f, 1000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.thermostat().GetState().output_changed_at_seconds == 1);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().Begin(5000));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.thermostat().GetState().output_changed_at_seconds == 1);
  fixture.Tick(19.0f, 6000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.thermostat().GetState().output_changed_at_seconds == 6);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().Begin(9000));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.thermostat().GetState().output_changed_at_seconds == 9);
  ++g_test_group_count;
}

void EmptyObserver(const Thermostat&, const Thermostat::State&, void*) {}

void TestObserverCapacity() {
  Fixture fixture;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().OnUpdate(nullptr));
  for (int i = 0; i < 4; ++i) {
    THERMOSTAT_HYSTERESIS_TEST_CHECK(
        fixture.thermostat().OnUpdate(EmptyObserver));
  }
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      !fixture.thermostat().OnUpdate(EmptyObserver));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.thermostat().RemoveUpdateCallback(EmptyObserver));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.thermostat().OnUpdate(EmptyObserver));
  fixture.thermostat().ClearCallbacks();
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      !fixture.thermostat().RemoveUpdateCallback(EmptyObserver));
  ++g_test_group_count;
}

void TestCommonConfigurationAndFaultPrecedence() {
  Fixture fixture;
  auto config = fixture.thermostat().GetConfig();
  config.hysteresis_c = 2.0f;
  config.error_output = true;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().SetConfig(config));
  fixture.Start(19.0f);
  ThermostatBase& base = fixture.thermostat();
  ThermostatBase::Config common = base.GetConfig();
  common.target_c = 23.0f;
  THERMOSTAT_HYSTERESIS_TEST_CHECK(base.SetConfig(common));
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.thermostat().GetConfig().hysteresis_c == 2.0f);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(base.GetConfig().target_c == 23.0f);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(
      fixture.sensor().SetError(TemperatureBase::Status::Disconnected));
  fixture.sensor().Update(1000);
  fixture.thermostat().Update(1000);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(fixture.thermostat().GetState().output);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(base.SetEnabled(false));
  fixture.thermostat().Update(1001);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(base.GetStatus() == Status::kDisabled);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.thermostat().GetState().output);
  THERMOSTAT_HYSTERESIS_TEST_CHECK(!fixture.bit().Get());
  ++g_test_group_count;
}

}  // namespace

int main() {
  TestAdapterValidation();
  TestInitialAndAdapterPolarity();
  TestThresholds();
  TestDelaysAndCancellation();
  TestExpiryRechecksInput();
  TestFaultAndRecovery();
  TestStaleAndInvalid();
  TestConfiguration();
  TestSetpointMemoryAndDisable();
  TestStartup();
  TestRollover();
  TestObservers();
  TestAdapterReentry();
  TestPendingDemandReversals();
  TestRestartPreservesOutputTimestamp();
  TestObserverCapacity();
  TestCommonConfigurationAndFaultPrecedence();
  printf("PASS: %d thermostat behavior groups\n", g_test_group_count);
}
