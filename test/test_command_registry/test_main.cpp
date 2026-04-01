#include <Arduino.h>
#include <unity.h>

#include "CommandRegistry.h"

void test_alias_normalization_max_rpm() {
    char cmd[32];
    CommandId id = CommandId::Unknown;
    bool ok = CommandRegistry::normalizeAndValidate("max-rpm", "600", cmd, sizeof(cmd), &id);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_STRING("max_rpm", cmd);
    TEST_ASSERT_EQUAL((int)CommandId::MaxRpm, (int)id);
}

void test_reject_invalid_boolean() {
    char cmd[32];
    bool ok = CommandRegistry::normalizeAndValidate("freerun", "yes", cmd, sizeof(cmd), nullptr);
    TEST_ASSERT_FALSE(ok);
}

void test_accept_window_shift_alias() {
    char cmd[32];
    bool ok = CommandRegistry::normalizeAndValidate("windows_shift", "1.0", cmd, sizeof(cmd), nullptr);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_STRING("window_shift", cmd);
}

void test_reject_out_of_range_int() {
    char cmd[32];
    bool ok = CommandRegistry::normalizeAndValidate("max_rpm", "9999", cmd, sizeof(cmd), nullptr);
    TEST_ASSERT_FALSE(ok);
}

void setup() {
    delay(500);
    UNITY_BEGIN();
    RUN_TEST(test_alias_normalization_max_rpm);
    RUN_TEST(test_reject_invalid_boolean);
    RUN_TEST(test_accept_window_shift_alias);
    RUN_TEST(test_reject_out_of_range_int);
    UNITY_END();
}

void loop() {
}
