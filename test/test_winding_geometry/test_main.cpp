#include <Arduino.h>
#include <unity.h>

#include "WindingGeometry.h"

void test_turns_per_pass_positive() {
    WindingGeometry g;
    g.totalWidth_mm = 17.0f;
    g.flangeBottom_mm = 1.5f;
    g.flangeTop_mm = 1.5f;
    g.margin_mm = 0.5f;
    g.wireDiameter_mm = 0.065f;
    TEST_ASSERT_GREATER_THAN(0, (int)g.turnsPerPass());
}

void test_effective_width_non_negative() {
    WindingGeometry g;
    g.totalWidth_mm = 3.0f;
    g.flangeBottom_mm = 2.0f;
    g.flangeTop_mm = 2.0f;
    g.margin_mm = 1.0f;
    TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(0.0f, g.effectiveWidth());
}

void test_traverse_ratio_nominal() {
    WindingGeometry g;
    g.totalWidth_mm = 17.0f;
    g.flangeBottom_mm = 1.5f;
    g.flangeTop_mm = 1.5f;
    g.margin_mm = 0.5f;
    g.wireDiameter_mm = 0.065f;
    // ratio = turnsPerPass / effectiveWidth
    float ratio = g.computeTraverseRatio();
    float expected = (float)g.turnsPerPass() / g.effectiveWidth();
    TEST_ASSERT_FLOAT_WITHIN(0.001f, expected, ratio);
    TEST_ASSERT_GREATER_THAN(0.0f, ratio);
}

void test_traverse_ratio_zero_width() {
    WindingGeometry g;
    g.totalWidth_mm = 4.0f;
    g.flangeBottom_mm = 2.0f;
    g.flangeTop_mm = 2.0f;
    g.margin_mm = 0.0f;
    // effectiveWidth = 0 → ratio should return 0 without divide-by-zero
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, g.computeTraverseRatio());
}

void test_problematic_ratio_integer() {
    // Exact integer ratios (3.0, 4.0, 5.0) should be flagged as problematic
    TEST_ASSERT_TRUE(WindingGeometry::isTraverseRatioProblematic(3.0f));
    TEST_ASSERT_TRUE(WindingGeometry::isTraverseRatioProblematic(4.0f));
    TEST_ASSERT_TRUE(WindingGeometry::isTraverseRatioProblematic(5.0f));
    // Near-integer within tolerance
    TEST_ASSERT_TRUE(WindingGeometry::isTraverseRatioProblematic(3.10f));
    TEST_ASSERT_TRUE(WindingGeometry::isTraverseRatioProblematic(3.90f));
}

void test_problematic_ratio_non_integer() {
    // Non-integer ratios well away from integers should NOT be flagged
    TEST_ASSERT_FALSE(WindingGeometry::isTraverseRatioProblematic(3.5f));
    TEST_ASSERT_FALSE(WindingGeometry::isTraverseRatioProblematic(4.3f));
    TEST_ASSERT_FALSE(WindingGeometry::isTraverseRatioProblematic(2.7f));
}

void test_problematic_ratio_zero_or_negative() {
    // Edge cases: zero or negative ratio should not be flagged
    TEST_ASSERT_FALSE(WindingGeometry::isTraverseRatioProblematic(0.0f));
    TEST_ASSERT_FALSE(WindingGeometry::isTraverseRatioProblematic(-1.0f));
}

void setup() {
    delay(500);
    UNITY_BEGIN();
    RUN_TEST(test_turns_per_pass_positive);
    RUN_TEST(test_effective_width_non_negative);
    RUN_TEST(test_traverse_ratio_nominal);
    RUN_TEST(test_traverse_ratio_zero_width);
    RUN_TEST(test_problematic_ratio_integer);
    RUN_TEST(test_problematic_ratio_non_integer);
    RUN_TEST(test_problematic_ratio_zero_or_negative);
    UNITY_END();
}

void loop() {
}
