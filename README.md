# ThermostatHysteresis

A complete C++11 heating/cooling thermostat with a binary output. Instantiate a
concrete temperature sensor, a BitRegisterIOAdapter, and ThermostatHysteresis:

~~~cpp
ThermostatHysteresis thermostat(sensor, output, logger, "Room");
~~~

The common contract is in [ThermostatBase](../ThermostatBase/README.md).
Startup testing, hysteresis memory, transition delays, initial/error outputs,
inversion, and typed callbacks are all implemented in this concrete library.

## Installation and ownership

Install ThermostatBase 2.0.0 or later, TemperatureBase, BitRegisterIOAdapter, and
ILogger as sibling libraries. The example additionally needs TemperatureFake
and SerialLogger. These dependencies are not downloaded by the test build.

References, optional logger, name, and callback contexts are non-owning.
Dependencies must outlive the thermostat. The application updates the sensor
before the thermostat, using the same millis() value. The thermostat is the
adapter's writer. The adapter must be a physical OUTPUT_MODE or a virtual bit.

## Configuration

Copy GetConfig() before editing and check SetConfig()'s result:

~~~cpp
auto config = thermostat.GetConfig();
config.mode = ThermostatBase::Mode::kHeating;
config.target_c = 22.0f;
config.hysteresis_c = 1.0f;
config.turn_on_delay_seconds = 2;
config.turn_off_delay_seconds = 2;
config.startup_test_seconds = 3;
config.error_output = true;
thermostat.SetConfig(config);
~~~

Common settings are documented in the base library. Variant settings are:

| Setting | Default | Meaning |
| --- | --- | --- |
| hysteresis_c | 0.5 | Full band width centered on the target |
| evaluation_interval_seconds | 1 | Evaluation interval when no transition is pending |
| turn_on_delay_seconds | 0 | Persistence delay before logical ON |
| turn_off_delay_seconds | 0 | Persistence delay before logical OFF |
| startup_test_seconds | 0 | Optional forced-ON test |
| initial_demand | false | Initial/recovered hysteresis memory inside the band |
| initial_output | false | Logical output applied by Begin() |
| error_output | false | Logical output on unusable input |

Temperatures must be finite, hysteresis nonnegative, and threshold arithmetic
finite. All durations are seconds and must not exceed
ThermostatBase::kMaximumDurationSeconds (2147483). Zero evaluation interval
means every Update(); zero transition delay applies the command immediately.

Invalid configuration leaves settings and timers unchanged. Equal settings are
a no-op. Effective changes apply on the next Update(), cancel delays/testing,
and trigger evaluation. Ordinary setpoint changes preserve hysteresis memory;
direction or initial_demand changes reseed it. Enabling/configuration changes
do not rerun the startup test. Commands during execution or callbacks are rejected.

## Hysteresis and delays

For target T and full width H, lower = T-H/2 and upper = T+H/2:

| Temperature | Heating demand | Cooling demand |
| --- | --- | --- |
| Below lower | ON | OFF |
| At or above upper | OFF | ON |
| Between boundaries, including lower | Preserve previous demand | Preserve previous demand |

Demand and commanded output are separate. ON/OFF delays require an evaluated
request to persist before changing output. While pending, demand is checked
every Update(), so an observed reversal cancels the delay even if the ordinary
evaluation interval is long. After cancellation, normal evaluation scheduling
resumes. Changes that occur entirely between Update() calls cannot be observed.
These delays are not minimum equipment run/rest times.

## Initialization, faults, and output

A successful Begin(now_ms) validates the adapter, applies initial_output (OFF if
disabled), and notifies observers. Control starts on the next Update().
An input-mode adapter is rejected and logged at Error when a logger exists;
neither Begin() nor subsequent Update() writes to that rejected adapter.

The optional startup test starts on the first healthy Update() after Begin().
Missing/unhealthy/stale input aborts testing, including before it starts; recovery
does not restart it. On test expiry, normal control resumes and the ordinary OFF
delay may keep output ON. A 3-second test followed by a 2-second OFF delay
therefore gives an approximately 5-second initial ON pulse above the heating band.

Disabled forces logical OFF. Missing, unhealthy, or stale input applies
error_output without transition delays. Recovery immediately evaluates demand
from initial_demand and starts the full applicable transition delay.

All settings/state are logical: true requests ON. Every output path calls
adapter.Set(logical_output). Configure active-low physical outputs with the
adapter constructor's final invert_output argument. The adapter's Get() and
callbacks also report logical values, not measured equipment feedback. Physical
outputs start logical OFF when constructed; Begin() applies initial_output.

## State, observers, and clock

State exposes status, demand, output, pending, startup_testing, and
output_changed_at_seconds. The timestamp is millis()/1000 at the last logical
output change or first successful Begin(); restarting an unchanged output
preserves it. It wraps with millis() and is intended for reporting.

OnUpdate() supports four callback/context slots, including duplicates. It reports
state changes and every Begin(), with no immediate callback on registration.
RemoveUpdateCallback() removes the first matching pair. ClearCallbacks() clears
future registrations. Current dispatch uses a snapshot prepared before writing
the adapter. Getters see committed state even inside synchronous adapter callbacks.

Configuration and recursive Begin()/Update() are rejected while busy. Subscription
changes affect later notifications; contexts must survive the current dispatch.
Use one loop/task, not simultaneous task or interrupt calls.

Begin()/Update() take raw milliseconds to match TemperatureBase. Configuration
and reported timestamps use seconds. Elapsed calculations stay in unsigned
milliseconds for rollover safety; call Update() regularly.

## Example

Open examples/HysteresisThermostatTest/HysteresisThermostatTest.ino and use
Serial Monitor at 115200 baud. Its LED BitRegisterIOAdapter is passed directly
to a heating thermostat. Logged objects are constructed after Serial.begin().

The fake temperature ramps 24 -> 20 -> 24 C in 0.5 C steps every 4 seconds.
Startup is 3 seconds ON; both transition delays are 2 seconds. After a full
down/up cycle and warm hold, an 8-second InvalidReading error forces ON.
A fresh warm reading clears the error, followed by the OFF delay and another
cycle. Both library loggers are always enabled at Debug level.

Commands: + / - adjust target by 0.5 C, e / d enable/disable, ? reports status,
and r restores defaults/restarts. Manual commands change the default scenario.
Mega is preferred for full logging on AVR; Uno has limited SRAM headroom.

## Tests and review results

All host tests live in this library. See [tests/README.md](tests/README.md).
They exercise both libraries through actual implementations with mocked GPIO.

The review corrected two behaviors:

- Pending requests now observe reversals before their deadline when evaluation
  intervals are longer than the delay.
- Repeated Begin() preserves the last output-change timestamp when output is
  unchanged.

Regressions also cover mode validation/logging, initial output, heating/cooling
boundaries, cancellation, startup, fault recovery, freshness, inversion, disabled
precedence, base-interface configuration, callback snapshots/capacity/reentry,
and millis() rollover. Hardware operation is not emulated by these tests.

## Migration from the combined ThermostatBase library

Hysteresis sources, examples, and tests moved here. Install both libraries.
The include remains ThermostatHysteresis.h. Methods retain their names.

| Previous public name | Google-style name |
| --- | --- |
| Mode::Heating / Cooling | Mode::kHeating / kCooling |
| Status::Ok (and other values) | Status::kOk (all gain k prefixes) |
| targetC | target_c |
| minimumTargetC / maximumTargetC | minimum_target_c / maximum_target_c |
| maximumReadingAgeSeconds | maximum_reading_age_seconds |
| hysteresisC | hysteresis_c |
| evaluationIntervalSeconds | evaluation_interval_seconds |
| turnOnDelaySeconds / turnOffDelaySeconds | turn_on_delay_seconds / turn_off_delay_seconds |
| startupTestSeconds | startup_test_seconds |
| initialDemand / initialOutput | initial_demand / initial_output |
| errorOutput | error_output |
| startupTesting | startup_testing |
| outputChangedAtSeconds | output_changed_at_seconds |

TemperatureBase, BitRegisterIOAdapter, and ILogger APIs were not renamed.
Their enum values and fields retain their existing spelling.

## Style and package metadata

.clang-format uses [Google C++ style](https://google.github.io/styleguide/cppguide.html).
Public data fields use snake_case, enum constants use k prefixes, and methods
use UpperCamelCase. Arduino CamelCase header filenames, .cpp extensions, global
API classes, AVR-compatible C headers, and C++11 are explicit platform conventions.

keywords.txt uses real tab-separated columns. library.properties identifies this
package and depends on ThermostatBase instead of bundling its implementation.
The project URL is blank until a repository URL is assigned.

Output polarity has moved from ThermostatHysteresis::Config::invert_output to
the final BitRegisterIOAdapter physical constructor argument. Remove the former
setting and construct active-low outputs with (pin, OUTPUT_MODE, logger, name, true).
