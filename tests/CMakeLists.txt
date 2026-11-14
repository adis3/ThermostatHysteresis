cmake_minimum_required(VERSION 3.16)
project(ThermostatHysteresisTests LANGUAGES CXX)

get_filename_component(_default_libraries_root
  "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(ARDUINO_LIBRARIES_ROOT "${_default_libraries_root}" CACHE PATH
  "Arduino libraries folder containing the dependencies")
set(THERMOSTAT_BASE_ROOT "${ARDUINO_LIBRARIES_ROOT}/ThermostatBase" CACHE PATH
  "ThermostatBase library folder")

set(_dependency_sources
  "${THERMOSTAT_BASE_ROOT}/src/ThermostatBase.cpp"
  "${ARDUINO_LIBRARIES_ROOT}/TemperatureBase/src/TemperatureBase.cpp"
  "${ARDUINO_LIBRARIES_ROOT}/TemperatureFake/src/TemperatureFake.cpp"
  "${ARDUINO_LIBRARIES_ROOT}/BitRegisterIOAdapter/src/BitRegisterIOAdapter.cpp")
foreach(_source IN LISTS _dependency_sources)
  if(NOT EXISTS "${_source}")
    message(FATAL_ERROR "Missing dependency source: ${_source}")
  endif()
endforeach()

add_executable(thermostat_hysteresis_test
  thermostat_hysteresis_test.cpp
  ../src/ThermostatHysteresis.cpp
  ${_dependency_sources})
target_compile_features(thermostat_hysteresis_test PRIVATE cxx_std_11)
target_include_directories(thermostat_hysteresis_test PRIVATE
  stubs
  ../src
  "${THERMOSTAT_BASE_ROOT}/src"
  "${ARDUINO_LIBRARIES_ROOT}/TemperatureBase/src"
  "${ARDUINO_LIBRARIES_ROOT}/TemperatureFake/src"
  "${ARDUINO_LIBRARIES_ROOT}/BitRegisterIOAdapter/src"
  "${ARDUINO_LIBRARIES_ROOT}/ILogger/src")
if(MSVC)
  target_compile_options(thermostat_hysteresis_test PRIVATE /W4 /WX /EHsc)
  target_compile_definitions(thermostat_hysteresis_test PRIVATE
    _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(thermostat_hysteresis_test PRIVATE
    -Wall -Wextra -Werror)
endif()

enable_testing()
add_test(NAME thermostat_hysteresis_behavior COMMAND thermostat_hysteresis_test)
