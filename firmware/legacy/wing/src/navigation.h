#pragma once

#include <Arduino.h>

#include "control_input.h"
#include "sensors.h"

struct GuidanceTarget {
  double lat = 0.0;
  double lon = 0.0;
  float altM = 0.0f;

  GuidanceTarget() = default;
  GuidanceTarget(double lat_, double lon_, float altM_)
      : lat(lat_), lon(lon_), altM(altM_) {}
};

struct NavigationState {
  double prevLat = 0.0;
  double prevLon = 0.0;
  uint32_t prevTimeMs = 0;
  float courseDeg = 0.0f;
  bool valid = false;
};

void navigationReset();
void navigationUpdate(const GpsData& gps, NavigationState& nav);
void guidanceToTarget(const GpsData& gps, const NavigationState& nav,
                      const GuidanceTarget& target, ControlInput& out);
