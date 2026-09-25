// Host test for the wing mixer's arithmetic.
//
// This compiles src/mix_math.h directly — the same header mixer.cpp includes —
// so it exercises the arithmetic the firmware flashes rather than a restatement
// of it. It is deliberately not a PlatformIO unit test: there is no `native`
// platform installed for this project, and `pio test` would need one downloaded.
// See README.md in this directory.
//
// What it checks is the shape of the mix: which way each surface moves, that the
// two elevons are mirror images in roll, and that nothing escapes its limits.
// What it does not check is the tuning. The gains below are the values the
// firmware uses today, but the assertions hold for any positive gain, and a
// retune that changed their sign or their symmetry would be a mixer bug worth
// failing on, while a retune that changed their magnitude would not.

#include <cstdio>
#include <cmath>

#include "../src/mix_math.h"

namespace {

int g_checks = 0;
int g_failures = 0;

void checkNear(const char* what, float got, float want) {
  ++g_checks;
  if (std::fabs(got - want) > 1e-6f) {
    std::printf("FAIL %s: got %f, want %f\n", what, got, want);
    ++g_failures;
  }
}

void checkTrue(const char* what, bool ok) {
  ++g_checks;
  if (!ok) {
    std::printf("FAIL %s\n", what);
    ++g_failures;
  }
}

// The firmware's values, from src/config.h and the profiles. Only the sign and
// the symmetry of these matter to the assertions.
const float kNeutral = 0.5f, kLo = 0.0f, kHi = 1.0f;
const float kPitchGain = 0.5f, kRollGain = 0.5f, kYawGain = 0.5f;

void testNeutralIsNeutral() {
  float l, r;
  akElevonMix(0.0f, 0.0f, kPitchGain, kRollGain, kNeutral, kLo, kHi, l, r);
  checkNear("neutral elevon left", l, kNeutral);
  checkNear("neutral elevon right", r, kNeutral);

  float ml, mr;
  akDifferentialThrust(0.0f, 0.0f, kYawGain, kLo, kHi, ml, mr);
  checkNear("neutral motor left", ml, 0.0f);
  checkNear("neutral motor right", mr, 0.0f);

  checkNear("neutral common throttle", akCommonThrottle(0.0f, kLo, kHi), 0.0f);
}

void testPitchMovesBothElevonsTogether() {
  float l, r;
  akElevonMix(1.0f, 0.0f, kPitchGain, kRollGain, kNeutral, kLo, kHi, l, r);
  checkNear("nose-up left elevon", l, 1.0f);
  checkNear("nose-up right elevon", r, 1.0f);

  akElevonMix(-0.5f, 0.0f, kPitchGain, kRollGain, kNeutral, kLo, kHi, l, r);
  checkNear("nose-down left elevon", l, 0.25f);
  checkNear("nose-down right elevon", r, 0.25f);
}

void testRollMovesElevonsOppositeWays() {
  float l, r;
  akElevonMix(0.0f, 0.5f, kPitchGain, kRollGain, kNeutral, kLo, kHi, l, r);
  checkNear("right-roll left elevon", l, 0.25f);
  checkNear("right-roll right elevon", r, 0.75f);
  checkTrue("right roll drops the left elevon", l < kNeutral);
  checkTrue("right roll raises the right elevon", r > kNeutral);

  akElevonMix(0.0f, -0.5f, kPitchGain, kRollGain, kNeutral, kLo, kHi, l, r);
  checkTrue("left roll raises the left elevon", l > kNeutral);
  checkTrue("left roll drops the right elevon", r < kNeutral);
}

void testElevonsMirrorInRollForAnyGain() {
  const float gains[] = {0.1f, 0.25f, 0.33f, 0.5f, 1.0f};
  const float pitches[] = {-0.4f, 0.0f, 0.3f};
  const float rolls[] = {0.0f, 0.2f, 0.45f};
  for (float pg : gains) {
    for (float rg : gains) {
      for (float p : pitches) {
        for (float roll : rolls) {
          float lPos, rPos, lNeg, rNeg;
          akElevonMix(p, roll, pg, rg, kNeutral, kLo, kHi, lPos, rPos);
          akElevonMix(p, -roll, pg, rg, kNeutral, kLo, kHi, lNeg, rNeg);
          // Unclamped here: |pitch*pg| + |roll*rg| <= 0.4 + 0.45 < 0.5.
          checkTrue("elevons mirror in roll", std::fabs(lPos - rNeg) < 1e-6f &&
                                              std::fabs(rPos - lNeg) < 1e-6f);
        }
      }
    }
  }
}

void testNothingEscapesItsLimits() {
  const float extremes[] = {-100.0f, -2.0f, -1.0f, 1.0f, 2.0f, 100.0f};
  for (float p : extremes) {
    for (float roll : extremes) {
      float l, r;
      akElevonMix(p, roll, kPitchGain, kRollGain, kNeutral, kLo, kHi, l, r);
      checkTrue("elevon left within limits", l >= kLo && l <= kHi);
      checkTrue("elevon right within limits", r >= kLo && r <= kHi);
    }
  }
  for (float t : extremes) {
    for (float y : extremes) {
      float ml, mr;
      akDifferentialThrust(t, y, kYawGain, kLo, kHi, ml, mr);
      checkTrue("motor left within limits", ml >= kLo && ml <= kHi);
      checkTrue("motor right within limits", mr >= kLo && mr <= kHi);
      checkTrue("common throttle within limits",
                akCommonThrottle(t, kLo, kHi) >= kLo &&
                akCommonThrottle(t, kLo, kHi) <= kHi);
    }
  }
}

void testClampingIsAtTheLimitNotNearIt() {
  float l, r;
  akElevonMix(0.0f, 2.0f, kPitchGain, kRollGain, kNeutral, kLo, kHi, l, r);
  checkNear("saturated right-roll left elevon", l, kLo);
  checkNear("saturated right-roll right elevon", r, kHi);

  float ml, mr;
  akDifferentialThrust(1.0f, 1.0f, kYawGain, kLo, kHi, ml, mr);
  checkNear("saturated motor left", ml, kHi);
  checkNear("saturated motor right", mr, 0.5f);
}

void testDifferentialThrustDirection() {
  float ml, mr;
  akDifferentialThrust(0.5f, 0.4f, kYawGain, kLo, kHi, ml, mr);
  checkNear("nose-right left motor", ml, 0.7f);
  checkNear("nose-right right motor", mr, 0.3f);
  checkTrue("nose-right speeds the left motor", ml > mr);

  akDifferentialThrust(0.5f, -0.4f, kYawGain, kLo, kHi, ml, mr);
  checkTrue("nose-left speeds the right motor", mr > ml);

  akDifferentialThrust(0.5f, 0.0f, kYawGain, kLo, kHi, ml, mr);
  checkNear("no yaw, matched motors left", ml, 0.5f);
  checkNear("no yaw, matched motors right", mr, 0.5f);
}

}  // namespace

int main() {
  testNeutralIsNeutral();
  testPitchMovesBothElevonsTogether();
  testRollMovesElevonsOppositeWays();
  testElevonsMirrorInRollForAnyGain();
  testNothingEscapesItsLimits();
  testClampingIsAtTheLimitNotNearIt();
  testDifferentialThrustDirection();

  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
