// Minimal assert based test runner. Deliberately dependency free: the project
// must build on a clean machine with a C++20 compiler and CMake and nothing
// else, so a test framework fetched at configure time is not an option.
#pragma once

#include <concepts>
#include <cstdint>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

namespace check {

struct TestCase {
  const char* suite;
  const char* name;
  void (*fn)();
};

std::vector<TestCase>& registry();
void fail(const char* file, int line, const std::string& message);
int run(int argc, char** argv);

struct Registrar {
  Registrar(const char* suite, const char* name, void (*fn)()) {
    registry().push_back(TestCase{suite, name, fn});
  }
};

template <class T>
std::string show(const T& value) {
  if constexpr (requires(std::ostream& os, const T& v) { os << v; }) {
    std::ostringstream ss;
    ss << value;
    return ss.str();
  } else {
    return "<not printable>";
  }
}

}  // namespace check

#define WP_TEST(suite_name, test_name)                                        \
  static void wp_test_##suite_name##_##test_name();                           \
  static ::check::Registrar wp_reg_##suite_name##_##test_name(                \
      #suite_name, #test_name, &wp_test_##suite_name##_##test_name);          \
  static void wp_test_##suite_name##_##test_name()

#define CHECK_TRUE(expr)                                                      \
  do {                                                                        \
    if (!(expr)) ::check::fail(__FILE__, __LINE__, "expected true: " #expr);  \
  } while (0)

#define CHECK_FALSE(expr)                                                     \
  do {                                                                        \
    if (expr) ::check::fail(__FILE__, __LINE__, "expected false: " #expr);    \
  } while (0)

#define CHECK_EQ(lhs, rhs)                                                    \
  do {                                                                        \
    const auto& wp_l = (lhs);                                                 \
    const auto& wp_r = (rhs);                                                 \
    if (!(wp_l == wp_r)) {                                                    \
      ::check::fail(__FILE__, __LINE__,                                       \
                    std::string(#lhs " == " #rhs " | left = ") +              \
                        ::check::show(wp_l) + ", right = " +                  \
                        ::check::show(wp_r));                                 \
    }                                                                         \
  } while (0)
