// vdsim — headless batch front-end for VehicleCore.
#include <sys/stat.h>

#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "vd/scenario.hpp"

namespace {

void usage() {
  std::puts(
      "vdsim — vehicle dynamics simulator (headless)\n"
      "\n"
      "usage:\n"
      "  vdsim run <scenario.json>... [options]   run scenarios (and their sweeps)\n"
      "      -o, --out <dir>        output directory (default: out)\n"
      "      -j, --jobs <n>         worker threads (default: all cores)\n"
      "      --set key=value        override a parameter (repeatable)\n"
      "      --vehicle <file>       override the scenario's vehicle file\n"
      "      --no-csv               only write summary.csv\n"
      "      -q, --quiet            no per-run output\n"
      "  vdsim params [--vehicle <file>]          list all parameters (key, value, range, unit)\n"
      "  vdsim export-vehicle [file]              write the default vehicle as JSON\n"
      "  vdsim tire [--axle front|rear] [--fz N] [--kappa k]   print the Fy(alpha) curve as CSV\n");
}

bool loadVehicle(vd::Vehicle& v, const std::string& path) {
  std::string text, err;
  if (!vd::json::readFile(path, text)) {
    std::fprintf(stderr, "error: cannot read %s\n", path.c_str());
    return false;
  }
  const auto root = vd::json::parse(text, &err);
  std::vector<std::string> unknown;
  if (!err.empty() || !v.loadParams(root, &unknown, &err)) {
    std::fprintf(stderr, "error: %s: %s\n", path.c_str(), err.c_str());
    return false;
  }
  for (const auto& k : unknown) std::fprintf(stderr, "warning: %s: unknown key '%s'\n", path.c_str(), k.c_str());
  return true;
}

std::string sanitize(std::string s) {
  for (char& c : s)
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_' || c == '.')) c = '_';
  return s;
}

std::string fmtNum(double v) {
  if (!std::isfinite(v)) return "nan";
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.6g", v);
  return buf;
}

int cmdRun(int argc, char** argv) {
  std::vector<std::string> files;
  std::string outDir = "out", vehicleOverride;
  vd::ParamSet sets;
  unsigned jobs = 0;
  bool csv = true, quiet = false;
  for (int i = 0; i < argc; ++i) {
    const std::string a = argv[i];
    const auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: %s needs a value\n", a.c_str());
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "-o" || a == "--out") outDir = next();
    else if (a == "-j" || a == "--jobs") jobs = unsigned(std::max(0, std::atoi(next().c_str())));
    else if (a == "--vehicle") vehicleOverride = next();
    else if (a == "--no-csv") csv = false;
    else if (a == "-q" || a == "--quiet") quiet = true;
    else if (a == "--set") {
      const std::string kv = next();
      const size_t eq = kv.find('=');
      if (eq == std::string::npos) {
        std::fprintf(stderr, "error: --set expects key=value\n");
        return 2;
      }
      sets.emplace_back(kv.substr(0, eq), std::atof(kv.c_str() + eq + 1));
    } else if (!a.empty() && a[0] == '-') {
      std::fprintf(stderr, "error: unknown option %s\n", a.c_str());
      return 2;
    } else {
      files.push_back(a);
    }
  }
  if (files.empty()) {
    std::fprintf(stderr, "error: no scenario files given\n");
    return 2;
  }
  ::mkdir(outDir.c_str(), 0755);

  // summary.csv: one row per run with sweep values and all metrics.
  struct Row {
    std::string scenario, csvFile;
    vd::RunResult res;
  };
  std::vector<Row> rows;
  int failures = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (const auto& file : files) {
    vd::Scenario sc;
    std::string err;
    if (!vd::loadScenarioFile(file, sc, &err)) {
      std::fprintf(stderr, "error: %s\n", err.c_str());
      ++failures;
      continue;
    }
    if (!vehicleOverride.empty()) sc.vehicleFile = vehicleOverride;
    const size_t total = vd::expandSweep(sc.sweep).size();
    if (!quiet) std::printf("%s: %zu run%s\n", sc.name.c_str(), total, total == 1 ? "" : "s");
    auto results = vd::runBatch(sc, sets, jobs);
    for (size_t i = 0; i < results.size(); ++i) {
      auto& r = results[i];
      Row row{sc.name, "", std::move(r)};
      if (!row.res.ok) {
        std::fprintf(stderr, "  run %zu failed: %s\n", i, row.res.error.c_str());
        ++failures;
      } else if (csv) {
        std::string name = sanitize(sc.name);
        if (total > 1) name += "_" + std::to_string(i);
        row.csvFile = name + ".csv";
        if (!vd::writeTelemetryCsv(outDir + "/" + row.csvFile, row.res.samples)) {
          std::fprintf(stderr, "error: cannot write %s/%s\n", outDir.c_str(), row.csvFile.c_str());
          ++failures;
        }
      }
      if (!quiet && row.res.ok) {
        std::printf("  [%zu]", i);
        for (const auto& [k, v] : row.res.overrides) std::printf(" %s=%s", k.c_str(), fmtNum(v).c_str());
        const auto& m = row.res.metrics;
        for (const char* key : {"ss_ay_g", "ss_yaw_rate_dps", "understeer_gradient_deg_per_g", "yaw_rate_response_time",
                                "stopping_distance", "time_0_100kmh", "max_roll_deg"}) {
          const auto it = m.find(key);
          if (it != m.end() && std::isfinite(it->second) && std::abs(it->second) > 1e-6) std::printf("  %s=%s", key, fmtNum(it->second).c_str());
        }
        std::printf("\n");
      }
      rows.push_back(std::move(row));
    }
  }

  // Union of parameter and metric columns, in first-seen order.
  std::vector<std::string> paramCols, metricCols;
  std::set<std::string> seenP, seenM;
  for (const auto& r : rows) {
    for (const auto& [k, v] : r.res.overrides)
      if (seenP.insert(k).second) paramCols.push_back(k);
    for (const auto& [k, v] : r.res.metrics)
      if (seenM.insert(k).second) metricCols.push_back(k);
  }
  std::string summary = "scenario,run,ok,csv";
  for (const auto& c : paramCols) summary += "," + c;
  for (const auto& c : metricCols) summary += "," + c;
  summary += "\n";
  std::string last;
  int idx = 0;
  for (const auto& r : rows) {
    idx = (r.scenario == last) ? idx + 1 : 0;
    last = r.scenario;
    summary += r.scenario + "," + std::to_string(idx) + "," + (r.res.ok ? "1" : "0") + "," + r.csvFile;
    for (const auto& c : paramCols) {
      std::string v;
      for (const auto& [k, val] : r.res.overrides)
        if (k == c) v = fmtNum(val);
      summary += "," + v;
    }
    for (const auto& c : metricCols) {
      const auto it = r.res.metrics.find(c);
      summary += "," + (it == r.res.metrics.end() ? std::string() : fmtNum(it->second));
    }
    summary += "\n";
  }
  vd::json::writeFile(outDir + "/summary.csv", summary);
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  if (!quiet) std::printf("%zu runs in %.2f s -> %s/summary.csv\n", rows.size(), secs, outDir.c_str());
  return failures ? 1 : 0;
}

int cmdParams(int argc, char** argv) {
  vd::Vehicle v;
  for (int i = 0; i < argc; ++i)
    if (std::strcmp(argv[i], "--vehicle") == 0 && i + 1 < argc && !loadVehicle(v, argv[++i])) return 1;
  std::string group;
  for (const auto& d : v.registry().all()) {
    if (d.group != group) {
      group = d.group;
      std::printf("\n[%s]\n", group.c_str());
    }
    std::printf("  %-42s %12s  [%g .. %g] %s%s%s\n", d.key.c_str(), fmtNum(d.get()).c_str(), d.min, d.max,
                d.unit.c_str(), d.choices.empty() ? "" : "  ", d.choices.c_str());
  }
  return 0;
}

int cmdExport(int argc, char** argv) {
  vd::Vehicle v;
  const std::string text = vd::json::dump(v.registry().save()) + "\n";
  if (argc > 0) {
    if (!vd::json::writeFile(argv[0], text)) {
      std::fprintf(stderr, "error: cannot write %s\n", argv[0]);
      return 1;
    }
  } else {
    std::fputs(text.c_str(), stdout);
  }
  return 0;
}

int cmdTire(int argc, char** argv) {
  vd::Vehicle v;
  int axle = vd::Front;
  double fz = 4000, kappa = 0;
  for (int i = 0; i + 1 < argc; ++i) {
    if (std::strcmp(argv[i], "--axle") == 0) axle = std::strcmp(argv[++i], "rear") == 0 ? vd::Rear : vd::Front;
    else if (std::strcmp(argv[i], "--fz") == 0) fz = std::atof(argv[++i]);
    else if (std::strcmp(argv[i], "--kappa") == 0) kappa = std::atof(argv[++i]);
    else if (std::strcmp(argv[i], "--vehicle") == 0 && !loadVehicle(v, argv[++i])) return 1;
  }
  std::printf("alpha_deg,fx,fy,mz\n");
  for (double a = -15; a <= 15.0001; a += 0.25) {
    const auto f = v.evalTire(axle, vd::TireInput{fz, kappa, vd::deg2rad(a), 0.0});
    std::printf("%.2f,%.1f,%.1f,%.2f\n", a, f.fx, f.fy, f.mz);
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || std::strcmp(argv[1], "-h") == 0 || std::strcmp(argv[1], "--help") == 0) {
    usage();
    return argc < 2 ? 2 : 0;
  }
  const std::string cmd = argv[1];
  if (cmd == "run") return cmdRun(argc - 2, argv + 2);
  if (cmd == "params") return cmdParams(argc - 2, argv + 2);
  if (cmd == "export-vehicle") return cmdExport(argc - 2, argv + 2);
  if (cmd == "tire") return cmdTire(argc - 2, argv + 2);
  std::fprintf(stderr, "unknown command '%s'\n\n", cmd.c_str());
  usage();
  return 2;
}
