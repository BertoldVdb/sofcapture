#include <cstdio>

#include "check.h"

TestCase*& test_registry() {
  static TestCase* head = nullptr;
  return head;
}

static int s_failures = 0;
static const char* s_current = "";

void test_fail(const char* file, int line, const char* expr) {
  s_failures++;
  fprintf(stderr, "FAIL %s: %s:%d: %s\n", s_current, file, line, expr);
}

int main() {
  int count = 0;
  for (TestCase* c = test_registry(); c; c = c->next) {
    s_current = c->name;
    int before = s_failures;
    c->fn();
    printf("%s %s\n", s_failures == before ? "ok  " : "FAIL", c->name);
    count++;
  }
  printf("%d tests, %d failures\n", count, s_failures);
  return s_failures ? 1 : 0;
}
