// Tiny self-contained test framework (no external dependencies).
#pragma once

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace vdtest {

struct Case {
  const char* name;
  std::function<void()> fn;
};

inline std::vector<Case>& registry() {
  static std::vector<Case> r;
  return r;
}

struct Registrar {
  Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

inline int& failures() {
  static int f = 0;
  return f;
}

inline void report(const char* file, int line, const std::string& msg) {
  ++failures();
  std::printf("    FAIL %s:%d: %s\n", file, line, msg.c_str());
}

}  // namespace vdtest

#define VD_CAT2(a, b) a##b
#define VD_CAT(a, b) VD_CAT2(a, b)
#define TEST(name)                                                              \
  static void name();                                                           \
  static vdtest::Registrar VD_CAT(reg_, name)(#name, name);                     \
  static void name()

#define CHECK(cond)                                                             \
  do {                                                                          \
    if (!(cond)) vdtest::report(__FILE__, __LINE__, "CHECK(" #cond ")");        \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                   \
  do {                                                                          \
    const double va_ = (a), vb_ = (b), vt_ = (tol);                             \
    if (!(std::abs(va_ - vb_) <= vt_)) {                                        \
      char buf_[256];                                                           \
      std::snprintf(buf_, sizeof buf_, "%s = %.6g, expected %.6g +- %.3g", #a, va_, vb_, vt_); \
      vdtest::report(__FILE__, __LINE__, buf_);                                 \
    }                                                                           \
  } while (0)

// Relative tolerance check: |a - b| <= rel * |b|
#define CHECK_REL(a, b, rel) CHECK_NEAR(a, b, std::abs((rel) * (b)))
