#pragma once

#include <Arduino.h>

struct ImuData {
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
};

struct GpsData {
  float lat = 0.0f;
  float lon = 0.0f;
  float altM = 0.0f;
  float groundspeedMps = 0.0f;
  bool fix = false;
};

void sensorsInit();
bool imuRead(ImuData& out);
bool gpsRead(GpsData& out);
