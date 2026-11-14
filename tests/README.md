# Native thermostat tests

These tests compile the real ThermostatHysteresis, ThermostatBase,
TemperatureBase, TemperatureFake, and BitRegisterIOAdapter sources.
Only Arduino GPIO calls are mocked. No board or Arduino core is needed.

## Run

Install CMake and a host C++ compiler (MSVC, GCC, or Clang). With the Arduino
libraries installed as sibling folders, run from ThermostatHysteresis:

~~~sh
cmake -S tests -B build/tests
cmake --build build/tests --config Debug
ctest --test-dir build/tests -C Debug --output-on-failure
~~~

For a different dependency location, configure with
-DARDUINO_LIBRARIES_ROOT=/path/to/Arduino/libraries. THERMOSTAT_BASE_ROOT can
override the base library separately, which is useful for staged reviews.

The target enables compiler warnings as errors. Checks use a project-specific
macro, not a redefinition of assert; their expressions still run with NDEBUG.

## Coverage

The executable reports 17 behavior groups covering:

- Physical-output/virtual acceptance and input-mode rejection with logging.
- Initial output, inversion, and repeated initialization.
- Heating/cooling threshold equality and zero hysteresis.
- ON/OFF delays, cancellations, and long-interval pending reversals.
- Sensor errors, stale/nonfinite readings, immediate fallback, and recovery.
- Configuration validation, clamping, equal-config timing, and memory retention.
- Common configuration through ThermostatBase without losing variant settings.
- Disabled precedence over configured error output.
- Startup duration, abort/recovery, and rollover.
- Observer snapshots, duplicates/capacity, removal, and reentrant calls.
- Preservation of output-change timestamps across unchanged restarts.

tests/ is outside src/, so Arduino does not compile the host executable or
its mock Arduino.h into sketches. stubs/Arduino.h must only be on the host
test include path. These checks do not verify electrical hardware behavior.
