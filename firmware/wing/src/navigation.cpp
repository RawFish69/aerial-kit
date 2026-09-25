#include "navigation.h"

#include <math.h>

#include "guidance.h"

static NavigationState s_nav;

static double toRad(double deg) {
  return deg * M_PI / 180.0;
}

static double toDeg(double rad) {
  return rad * 180.0 / M_PI;
}

static float wrapDeg(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

void navigationReset() {
  s_nav = NavigationState();
}

void navigationUpdate(const GpsData& gps, NavigationState& nav) {
  if (!gps.fix) {
    return;
  }

  if (nav.valid) {
    float dtSec = (millis() - nav.prevTimeMs) / 1000.0f;
    if (dtSec > 0.1f) {
      double dNorth = (gps.lat - nav.prevLat) * 111320.0;
      double dEast = (gps.lon - nav.prevLon) * 111320.0 * cos(toRad(gps.lat));
      if (fabs(dNorth) + fabs(dEast) > 1.0) {
        nav.courseDeg = static_cast<float>(toDeg(atan2(dEast, dNorth)));
      }
    }
  }

  nav.prevLat = gps.lat;
  nav.prevLon = gps.lon;
  nav.prevTimeMs = millis();
  nav.valid = true;
}

void guidanceToTarget(const GpsData& gps, const NavigationState& nav,
                      const GuidanceTarget& target, ControlInput& out) {
  if (!gps.fix || !nav.valid) {
    out = ControlInput();
    return;
  }

  double dNorth = (target.lat - gps.lat) * 111320.0;
  double dEast = (target.lon - gps.lon) * 111320.0 * cos(toRad(gps.lat));
  float distanceM = static_cast<float>(sqrt(dNorth * dNorth + dEast * dEast));
  float bearingDeg = static_cast<float>(toDeg(atan2(dEast, dNorth)));
  float headingErrorDeg = wrapDeg(bearingDeg - nav.courseDeg);
  float crossTrackM = distanceM * sinf(headingErrorDeg * M_PI / 180.0f);

  float bankRad = l1BankCommandRad(crossTrackM, gps.groundspeedMps, 50.0f);
  out.roll = constrain(bankRad / 0.785398f, -1.0f, 1.0f);

  float altitudeErrorM = target.altM - gps.altM;
  float airspeedErrorMps = 15.0f - gps.groundspeedMps;
  float pitchCmd = 0.0f;
  float throttleCmd = 0.0f;
  tecsEnergyCommand(altitudeErrorM, airspeedErrorMps, pitchCmd, throttleCmd);
  out.pitch = pitchCmd;
  out.throttle = throttleCmd;
  out.yaw = 0.0f;
}
