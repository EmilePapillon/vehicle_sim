#include <chrono>
#include <cstring>

#include "test.hpp"

int main(int argc, char** argv) {
  const char* filter = argc > 1 ? argv[1] : nullptr;
  int run = 0, failedCases = 0;
  const auto t0 = std::chrono::steady_clock::now();
  for (const auto& c : vdtest::registry()) {
    if (filter && !std::strstr(c.name, filter)) continue;
    const int before = vdtest::failures();
    const auto start = std::chrono::steady_clock::now();
    c.fn();
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    const bool ok = vdtest::failures() == before;
    std::printf("[%s] %-48s %8.1f ms\n", ok ? " ok " : "FAIL", c.name, ms);
    ++run;
    failedCases += !ok;
  }
  const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("\n%d tests, %d failed, %d failed checks (%.2f s)\n", run, failedCases, vdtest::failures(), s);
  return failedCases ? 1 : 0;
}
