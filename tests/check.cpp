#include "check.hpp"

#include <cstdio>
#include <cstring>
#include <exception>

namespace check {
namespace {
int g_failures_in_case = 0;
int g_total_failures = 0;
}  // namespace

std::vector<TestCase>& registry() {
  static std::vector<TestCase> cases;
  return cases;
}

void fail(const char* file, int line, const std::string& message) {
  ++g_failures_in_case;
  ++g_total_failures;
  std::fprintf(stderr, "    %s:%d: %s\n", file, line, message.c_str());
}

int run(int argc, char** argv) {
  const char* filter = argc > 1 ? argv[1] : nullptr;
  int ran = 0;
  int failed_cases = 0;

  for (const TestCase& c : registry()) {
    if (filter != nullptr && std::strcmp(filter, c.suite) != 0) continue;
    ++ran;
    g_failures_in_case = 0;
    std::fprintf(stderr, "[ run  ] %s.%s\n", c.suite, c.name);
    try {
      c.fn();
    } catch (const std::exception& e) {
      fail(__FILE__, __LINE__, std::string("uncaught exception: ") + e.what());
    } catch (...) {
      fail(__FILE__, __LINE__, "uncaught exception of unknown type");
    }
    if (g_failures_in_case == 0) {
      std::fprintf(stderr, "[  ok  ] %s.%s\n", c.suite, c.name);
    } else {
      ++failed_cases;
      std::fprintf(stderr, "[ FAIL ] %s.%s (%d assertions)\n", c.suite, c.name,
                   g_failures_in_case);
    }
  }

  if (ran == 0) {
    std::fprintf(stderr, "no test case matched filter '%s'\n",
                 filter != nullptr ? filter : "");
    return 2;
  }
  std::fprintf(stderr, "%d case(s) run, %d failed, %d assertion failure(s)\n",
               ran, failed_cases, g_total_failures);
  return failed_cases == 0 ? 0 : 1;
}

}  // namespace check

int main(int argc, char** argv) { return check::run(argc, argv); }
