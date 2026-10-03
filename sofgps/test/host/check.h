// Minimal test harness: TEST(name) registers a case, CHECK records failures.
#pragma once

#include <cstdint>
#include <cstdio>

struct TestCase {
  const char* name;
  void (*fn)();
  TestCase* next;
};

TestCase*& test_registry();
void test_fail(const char* file, int line, const char* expr);

struct TestReg {
  TestReg(const char* name, void (*fn)()) {
    static TestCase cases[256];
    static int n = 0;
    TestCase* c = &cases[n++];
    c->name = name;
    c->fn = fn;
    c->next = test_registry();
    test_registry() = c;
  }
};

#define TEST(name)                       \
  static void name();                    \
  static TestReg reg_##name(#name, name); \
  static void name()

#define CHECK(cond)                                    \
  do {                                                 \
    if (!(cond)) test_fail(__FILE__, __LINE__, #cond); \
  } while (0)

#define CHECK_EQ(a, b)                                                                \
  do {                                                                                \
    auto va_ = (a);                                                                   \
    auto vb_ = (b);                                                                   \
    if (!(va_ == vb_)) {                                                              \
      char msg_[256];                                                                 \
      snprintf(msg_, sizeof msg_, "%s == %s (%lld vs %lld)", #a, #b,                  \
               static_cast<long long>(va_), static_cast<long long>(vb_));             \
      test_fail(__FILE__, __LINE__, msg_);                                            \
    }                                                                                 \
  } while (0)
