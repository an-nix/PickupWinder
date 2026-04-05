#include <Arduino.h>
#include <unity.h>

#include "WindingRecipeStore.h"
#include "Protocol.h"

void test_reject_future_recipe_version() {
    WindingRecipeStore store;
    WindingRecipe recipe;
    const char* json = "{\"version\":999,\"targetTurns\":8000}";
    TEST_ASSERT_FALSE(store.fromJson(json, recipe));
}

void test_accept_current_recipe_version() {
    WindingRecipeStore store;
    WindingRecipe recipe;
    char json[128];
    snprintf(json, sizeof(json), "{\"version\":%u,\"targetTurns\":9000}", (unsigned)PICKUP_RECIPE_FORMAT_VERSION);
    TEST_ASSERT_TRUE(store.fromJson(json, recipe));
    TEST_ASSERT_EQUAL(9000, recipe.targetTurns);
}

void setup() {
    delay(500);
    UNITY_BEGIN();
    RUN_TEST(test_reject_future_recipe_version);
    RUN_TEST(test_accept_current_recipe_version);
    UNITY_END();
}

void loop() {
}
