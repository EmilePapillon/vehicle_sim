// Headless scenario runner: JSON scenario -> vehicle + maneuver -> telemetry, metrics, CSV.
//
// {
//   "name": "step_steer",
//   "vehicle": "../vehicles/sedan.json",       // optional, relative to the scenario file
//   "params": {"suspension.front.arb_rate": 15000},
//   "duration": 6, "output_rate": 100, "initial_speed": 22.2,
//   "maneuver": {"steer": {...}, "speed": {...}},
//   "sweep": [{"param": "suspension.front.arb_rate", "values": [0, 10000, 20000]}]
// }
#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "vd/driver.hpp"

namespace vd {

struct SweepAxis {
  std::string param;
  std::vector<double> values;
};

struct Scenario {
  std::string name = "scenario";
  std::string vehicleFile;  // resolved path, may be empty
  std::vector<std::pair<std::string, double>> params;
  double duration = 10.0;
  double outputRate = 100.0;
  double initialSpeed = 0.0;
  json::Value maneuver;
  std::vector<SweepAxis> sweep;
};

using ParamSet = std::vector<std::pair<std::string, double>>;

struct RunResult {
  bool ok = false;
  std::string error;
  ParamSet overrides;               // sweep values used for this run
  std::vector<Telemetry> samples;
  std::map<std::string, double> metrics;
};

bool parseScenario(const json::Value& root, const std::string& baseDir, Scenario& out, std::string* error);
bool loadScenarioFile(const std::string& path, Scenario& out, std::string* error);

// Cartesian product of the sweep axes (a single empty set when there is no sweep).
std::vector<ParamSet> expandSweep(const std::vector<SweepAxis>& sweep);

// `extra` are applied after the scenario params (CLI --set, then sweep values).
RunResult runScenario(const Scenario& sc, const ParamSet& extra);

// Runs all sweep combinations on `threads` worker threads (0 = hardware concurrency).
std::vector<RunResult> runBatch(const Scenario& sc, const ParamSet& cliOverrides, unsigned threads,
                                const std::function<void(size_t done, size_t total)>& progress = {});

std::map<std::string, double> computeMetrics(const Scenario& sc, const Vehicle& v, const std::vector<Telemetry>& s);

std::string telemetryCsvHeader();
std::string telemetryCsvRow(const Telemetry& t);
bool writeTelemetryCsv(const std::string& path, const std::vector<Telemetry>& samples);

}  // namespace vd
