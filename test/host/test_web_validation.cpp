#include "web_validation.h"

#include <cmath>
#include <cstdio>

static int g_fail = 0;
#define CHECK(cond, msg) do { if (!(cond)) { std::printf("FAIL: %s\n", msg); ++g_fail; } } while (0)

int main() {
    float value = 0.0f;
    int integer = 0;
    CHECK(webval::parseFiniteFloat("12.5", value) && value == 12.5f, "parse finite float");
    CHECK(!webval::parseFiniteFloat("", value), "reject empty float");
    CHECK(!webval::parseFiniteFloat("12mm", value), "reject float suffix");
    CHECK(!webval::parseFiniteFloat("nan", value), "reject NaN");
    CHECK(!webval::parseFiniteFloat("inf", value), "reject infinity");
    CHECK(webval::parseInt("255", integer) && integer == 255, "parse integer");
    CHECK(!webval::parseInt("0x10", integer), "reject non-decimal integer");
    CHECK(!webval::parseInt("3axis", integer), "reject integer suffix");
    CHECK(webval::validFeed(20.0f), "valid feed");
    CHECK(!webval::validFeed(1.0f) && !webval::validFeed(200.0f), "reject feed boundaries");
    bool enabled = false;
    CHECK(webval::parseBool01("1", enabled) && enabled, "parse enable true");
    CHECK(webval::parseBool01("0", enabled) && !enabled, "parse enable false");
    CHECK(!webval::parseBool01("2", enabled), "reject non-boolean integer");
    if (g_fail == 0) { std::printf("ALL PASSED (web validation)\n"); return 0; }
    return 1;
}
