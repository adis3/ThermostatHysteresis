#include "ThermostatHysteresis.h"

#include <math.h>
#include <stdio.h>

constexpr uint8_t ThermostatHysteresis::kMaxCallbacks;

ThermostatHysteresis::ThermostatHysteresis(TemperatureBase& sensor,
                                           BitRegisterIOAdapter& output,
                                           ILogger* logger, const char* name)
    : ThermostatBase(sensor, logger, name),
      output_(output),
      config_{},
      state_{Status::kNotStarted, false, false, false, false, 0},
      callbacks_{},
      callback_count_(0),
      begun_(false),
      busy_(false),
      reset_requested_(false),
      reseed_requested_(false),
      evaluated_(false),
      has_demand_(false),
      test_available_(false),
      evaluated_at_ms_(0),
      pending_since_ms_(0),
      test_started_at_ms_(0) {}

ThermostatBase::Status ThermostatHysteresis::GetStatus() const {
  return state_.status;
}

const ThermostatHysteresis::Config& ThermostatHysteresis::GetConfig() const {
  return config_;
}

ThermostatHysteresis::State ThermostatHysteresis::GetState() const {
  return state_;
}

bool ThermostatHysteresis::IsValidConfig(const Config& c) {
  return ThermostatBase::IsValidConfig(c) && isfinite(c.hysteresis_c) &&
         c.hysteresis_c >= 0.0f &&
         isfinite(c.minimum_target_c - c.hysteresis_c * 0.5f) &&
         isfinite(c.maximum_target_c + c.hysteresis_c * 0.5f) &&
         c.evaluation_interval_seconds <= kMaximumDurationSeconds &&
         c.turn_on_delay_seconds <= kMaximumDurationSeconds &&
         c.turn_off_delay_seconds <= kMaximumDurationSeconds &&
         c.startup_test_seconds <= kMaximumDurationSeconds;
}

bool ThermostatHysteresis::SameConfig(const Config& a, const Config& b) {
  return ThermostatBase::SameConfig(a, b) && a.hysteresis_c == b.hysteresis_c &&
         a.evaluation_interval_seconds == b.evaluation_interval_seconds &&
         a.turn_on_delay_seconds == b.turn_on_delay_seconds &&
         a.turn_off_delay_seconds == b.turn_off_delay_seconds &&
         a.startup_test_seconds == b.startup_test_seconds &&
         a.initial_demand == b.initial_demand &&
         a.initial_output == b.initial_output &&
         a.error_output == b.error_output;
}

bool ThermostatHysteresis::SetConfig(const Config& config) {
  if (busy_) {
    return false;
  }
  if (!IsValidConfig(config)) {
    busy_ = true;
    Log(LogLevel::Error, "Invalid hysteresis thermostat configuration");
    busy_ = false;
    return false;
  }
  if (SameConfig(config_, config)) {
    return true;
  }
  reseed_requested_ = reseed_requested_ || config_.mode != config.mode ||
                      config_.initial_demand != config.initial_demand;
  config_ = config;
  reset_requested_ = true;
  return true;
}

bool ThermostatHysteresis::SetConfig(const ThermostatBase::Config& config) {
  Config next = config_;
  static_cast<ThermostatBase::Config&>(next) = config;
  return SetConfig(next);
}

bool ThermostatHysteresis::Begin(uint32_t now_ms) {
  if (busy_) {
    return false;
  }
  busy_ = true;
  const State previous = state_;
  const bool was_begun = begun_;
  begun_ = false;
  state_.demand = false;
  state_.pending = false;
  state_.startup_testing = false;

  if (!output_.IsVirtual() &&
      output_.GetMode() != BitRegisterIOAdapter::OUTPUT_MODE) {
    state_.status = Status::kInitializationFailed;
    Log(LogLevel::Error,
        "Output adapter must be OUTPUT_MODE or virtual; initialization "
        "rejected");
    Publish(previous, true, false);  // Never call Set() on an input adapter.
    busy_ = false;
    return false;
  }

  begun_ = true;
  reset_requested_ = false;
  reseed_requested_ = false;
  evaluated_ = false;
  has_demand_ = false;
  test_available_ = config_.enabled && config_.startup_test_seconds != 0;
  state_.status = config_.enabled ? Status::kStarting : Status::kDisabled;
  if (!was_begun) {
    state_.output_changed_at_seconds = now_ms / 1000UL;
  }
  SetOutput(config_.enabled ? config_.initial_output : false, now_ms);
  Publish(previous, true, true);  // Force initial physical/virtual mapping.
  busy_ = false;
  return true;
}

void ThermostatHysteresis::Update(uint32_t now_ms) {
  if (!begun_ || busy_) {
    return;
  }
  busy_ = true;
  const State previous = state_;
  if (reset_requested_) {
    reset_requested_ = false;
    evaluated_ = false;
    test_available_ = false;
    state_.pending = false;
    state_.startup_testing = false;
    if (reseed_requested_) {
      has_demand_ = false;
    }
    reseed_requested_ = false;
  }
  Step(now_ms);
  Publish(previous, false, true);
  busy_ = false;
}

void ThermostatHysteresis::Step(uint32_t now_ms) {
  float temperature_c = 0.0f;
  const Status input_status = ReadTemperature(now_ms, temperature_c);
  state_.status = input_status;

  // Always check input health before tests, evaluation intervals, or delays.
  if (input_status != Status::kOk) {
    state_.demand = false;
    state_.pending = false;
    state_.startup_testing = false;
    evaluated_ = false;
    has_demand_ = false;
    test_available_ = false;
    SetOutput(input_status == Status::kDisabled ? false : config_.error_output,
              now_ms);
    return;
  }

  if (test_available_) {
    if (!state_.startup_testing) {
      state_.startup_testing = true;
      test_started_at_ms_ = now_ms;
    }
    if (static_cast<uint32_t>(now_ms - test_started_at_ms_) <
        Milliseconds(config_.startup_test_seconds)) {
      state_.status = Status::kStarting;
      state_.demand = false;
      state_.pending = false;
      SetOutput(true, now_ms);
      return;
    }
    test_available_ = false;
    state_.startup_testing = false;
    evaluated_ = false;
    has_demand_ = false;
  }

  // Observe every pending request so a brief reversal cancels its delay.
  if (!evaluated_ || state_.pending ||
      static_cast<uint32_t>(now_ms - evaluated_at_ms_) >=
          Milliseconds(config_.evaluation_interval_seconds)) {
    state_.demand = ComputeDemand(
        temperature_c, has_demand_ ? state_.demand : config_.initial_demand);
    evaluated_at_ms_ = now_ms;
    evaluated_ = true;
    has_demand_ = true;
  }

  if (state_.demand == state_.output) {
    state_.pending = false;
    return;
  }
  if (!state_.pending) {
    state_.pending = true;
    pending_since_ms_ = now_ms;
  }
  const uint32_t requested_delay_seconds = state_.demand
                                               ? config_.turn_on_delay_seconds
                                               : config_.turn_off_delay_seconds;
  if (static_cast<uint32_t>(now_ms - pending_since_ms_) >=
      Milliseconds(requested_delay_seconds)) {
    SetOutput(state_.demand, now_ms);
    state_.pending = false;
  }
}

bool ThermostatHysteresis::ComputeDemand(float temperature_c,
                                         bool previous_demand) const {
  const float lower = config_.target_c - config_.hysteresis_c * 0.5f;
  const float upper = config_.target_c + config_.hysteresis_c * 0.5f;
  if (temperature_c < lower) {
    return config_.mode == Mode::kHeating;
  }
  if (temperature_c >= upper) {
    return config_.mode == Mode::kCooling;
  }
  return previous_demand;
}

void ThermostatHysteresis::SetOutput(bool output, uint32_t now_ms) {
  if (state_.output != output) {
    state_.output = output;
    state_.output_changed_at_seconds = now_ms / 1000UL;
  }
}

bool ThermostatHysteresis::SameState(const State& a, const State& b) {
  return a.status == b.status && a.demand == b.demand && a.output == b.output &&
         a.pending == b.pending && a.startup_testing == b.startup_testing &&
         a.output_changed_at_seconds == b.output_changed_at_seconds;
}

bool ThermostatHysteresis::OnUpdate(Callback callback, void* context) {
  if (!callback || callback_count_ == kMaxCallbacks) {
    return false;
  }
  callbacks_[callback_count_++] = {callback, context};
  return true;
}

bool ThermostatHysteresis::RemoveUpdateCallback(Callback callback,
                                                void* context) {
  for (uint8_t i = 0; i < callback_count_; ++i) {
    if (callbacks_[i].callback == callback &&
        callbacks_[i].context == context) {
      for (uint8_t j = i + 1; j < callback_count_; ++j) {
        callbacks_[j - 1] = callbacks_[j];
      }
      callbacks_[--callback_count_] = {nullptr, nullptr};
      return true;
    }
  }
  return false;
}

void ThermostatHysteresis::ClearCallbacks() {
  callback_count_ = 0;
  for (auto& entry : callbacks_) {
    entry = {nullptr, nullptr};
  }
}

void ThermostatHysteresis::Publish(const State& previous, bool force,
                                   bool write_output) {
  const State snapshot = state_;
  const bool notify = force || !SameState(previous, snapshot);
  const uint8_t count = notify ? callback_count_ : 0;
  Registration subscribers[kMaxCallbacks];
  for (uint8_t i = 0; i < count; ++i) {
    subscribers[i] = callbacks_[i];
  }

  // State and callback snapshot are committed before Set(), which can invoke
  // application callbacks synchronously. All output paths share this mapping.
  if (write_output) {
    output_.Set(snapshot.output);
  }
  if (!notify) {
    return;
  }

  char message[100];
  snprintf(message, sizeof(message),
           "Status=%s demand=%u output=%u pending=%u test=%u",
           StatusText(snapshot.status), static_cast<unsigned>(snapshot.demand),
           static_cast<unsigned>(snapshot.output),
           static_cast<unsigned>(snapshot.pending),
           static_cast<unsigned>(snapshot.startup_testing));
  const bool fault = snapshot.status == Status::kNoData ||
                     snapshot.status == Status::kSensorError ||
                     snapshot.status == Status::kStaleReading ||
                     snapshot.status == Status::kInitializationFailed;
  Log(fault ? LogLevel::Warning : LogLevel::Info, message);
  for (uint8_t i = 0; i < count; ++i) {
    subscribers[i].callback(*this, snapshot, subscribers[i].context);
  }
}
