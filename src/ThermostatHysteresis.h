#ifndef THERMOSTAT_HYSTERESIS_SRC_THERMOSTAT_HYSTERESIS_H_
#define THERMOSTAT_HYSTERESIS_SRC_THERMOSTAT_HYSTERESIS_H_

#include <BitRegisterIOAdapter.h>
#include <ThermostatBase.h>

class ThermostatHysteresis final : public ThermostatBase {
 public:
  struct Config : ThermostatBase::Config {
    float hysteresis_c = 0.5f;  // Full width, centered on target_c.
    uint32_t evaluation_interval_seconds = 1;
    uint32_t turn_on_delay_seconds = 0;
    uint32_t turn_off_delay_seconds = 0;
    uint32_t startup_test_seconds = 0;
    bool initial_demand = false;  // Hysteresis memory inside the band.
    bool initial_output =
        false;  // Applied immediately by a successful Begin().
    bool error_output = false;
  };

  struct State {
    Status status;
    bool demand;
    bool output;  // Logical ON/OFF command, not hardware feedback.
    bool pending;
    bool startup_testing;
    // millis()/1000 at the last output change (or the first successful
    // Begin()).
    uint32_t output_changed_at_seconds;
  };

  using Callback = void (*)(const ThermostatHysteresis& thermostat,
                            const State& state, void* context);

  // The adapter must be a physical OUTPUT_MODE or a virtual bit.
  // The thermostat is its writer. All dependencies must outlive this object.
  ThermostatHysteresis(TemperatureBase& sensor, BitRegisterIOAdapter& output,
                       ILogger* logger = nullptr, const char* name = nullptr);

  // Validates the adapter, applies initial_output, and publishes initial state.
  // Control starts on the next Update(); unavailable input then uses
  // error_output.
  bool Begin(uint32_t now_ms) override;
  void Update(uint32_t now_ms) override;

  Status GetStatus() const override;
  const Config& GetConfig() const override;
  State GetState() const;

  bool SetConfig(const Config& config);
  bool SetConfig(const ThermostatBase::Config& config) override;
  static bool IsValidConfig(const Config& config);

  // State changes plus each Begin(); no immediate call on subscription.
  // Four slots, duplicate registrations allowed. Subscription changes affect
  // later notifications. Contexts must survive the current notification.
  // Recursive updates and commands during execution/notification are rejected.
  bool OnUpdate(Callback callback, void* context = nullptr);
  bool RemoveUpdateCallback(Callback callback, void* context = nullptr);
  void ClearCallbacks();

 private:
  static constexpr uint8_t kMaxCallbacks = 4;
  struct Registration {
    Callback callback;
    void* context;
  };

  static bool SameConfig(const Config& a, const Config& b);
  static bool SameState(const State& a, const State& b);
  bool ComputeDemand(float temperature_c, bool previous_demand) const;
  void Step(uint32_t now_ms);
  void SetOutput(bool output, uint32_t now_ms);
  void Publish(const State& previous, bool force, bool write_output);

  BitRegisterIOAdapter& output_;
  Config config_;
  State state_;
  Registration callbacks_[kMaxCallbacks];
  uint8_t callback_count_;
  bool begun_;
  bool busy_;
  bool reset_requested_;
  bool reseed_requested_;
  bool evaluated_;
  bool has_demand_;
  bool test_available_;

  // Private clock samples stay in milliseconds to match TemperatureBase and
  // preserve elapsed-time arithmetic across millis() rollover. Not settings.
  uint32_t evaluated_at_ms_;
  uint32_t pending_since_ms_;
  uint32_t test_started_at_ms_;
};

#endif  // THERMOSTAT_HYSTERESIS_SRC_THERMOSTAT_HYSTERESIS_H_
