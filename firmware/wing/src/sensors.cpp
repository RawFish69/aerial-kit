#include "sensors.h"
#include "config.h"

#include <Wire.h>
#include <math.h>

#if !defined(ARDUINO_ARCH_STM32)
HardwareSerial& gpsSerial = Serial1;
#endif

namespace {

String g_nmeaLine;

constexpr uint8_t kMpu6050Address = 0x68;
constexpr uint8_t kPwrMgmt1 = 0x6B;
constexpr uint8_t kGyroConfig = 0x1B;
constexpr uint8_t kAccelConfig = 0x1C;
constexpr uint8_t kAccelXoutH = 0x3B;

bool mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kMpu6050Address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuRead(uint8_t reg, uint8_t* buffer, size_t length) {
  Wire.beginTransmission(kMpu6050Address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(kMpu6050Address, static_cast<uint8_t>(length)) != length) {
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

bool mpu6050Init() {
  // Wake the MPU6050 and select ±8 g accel / ±500 deg/s gyro.
  if (!mpuWrite(kPwrMgmt1, 0x00)) return false;
  if (!mpuWrite(kGyroConfig, 0x08)) return false;   // FS_SEL=1 -> 500 deg/s
  if (!mpuWrite(kAccelConfig, 0x10)) return false;  // AFS_SEL=2 -> 8 g
  return true;
}

int16_t combineInt16(uint8_t high, uint8_t low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

String nmeaField(const String& line, int index) {
  int start = 0;
  for (int i = 0; i < index; ++i) {
    int comma = line.indexOf(',', start);
    if (comma < 0) return "";
    start = comma + 1;
  }
  int end = line.indexOf(',', start);
  if (end < 0) end = line.length();
  return line.substring(start, end);
}

float nmeaCoordToDecimal(const String& coord, char hemisphere) {
  if (coord.length() < 4) return 0.0f;
  int dot = coord.indexOf('.');
  if (dot < 0) dot = coord.length();
  String degPart = coord.substring(0, dot - 2);
  String minPart = coord.substring(dot - 2);
  float value = degPart.toFloat() + minPart.toFloat() / 60.0f;
  if (hemisphere == 'S' || hemisphere == 'W') value = -value;
  return value;
}

void parseNmea(const String& line, GpsData& out) {
  if (line.startsWith("$GPGGA")) {
    String lat = nmeaField(line, 2);
    String ns = nmeaField(line, 3);
    String lon = nmeaField(line, 4);
    String ew = nmeaField(line, 5);
    String fix = nmeaField(line, 6);
    String alt = nmeaField(line, 9);
    out.fix = fix.toInt() > 0;
    if (lat.length() > 0 && lon.length() > 0) {
      out.lat = nmeaCoordToDecimal(lat, ns.length() ? ns[0] : 'N');
      out.lon = nmeaCoordToDecimal(lon, ew.length() ? ew[0] : 'E');
    }
    out.altM = alt.toFloat();
  } else if (line.startsWith("$GPRMC")) {
    String speedKnots = nmeaField(line, 7);
    out.groundspeedMps = speedKnots.toFloat() * 0.514444f;
  }
}

}  // namespace

void sensorsInit() {
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  if (!mpu6050Init()) {
    // IMU is optional; the controller falls back to direct bench commands.
  }
#if defined(ARDUINO_ARCH_ESP32)
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#elif !defined(ARDUINO_ARCH_STM32)
  gpsSerial.begin(GPS_BAUD);
#endif
}

bool imuRead(ImuData& out) {
  uint8_t buffer[14];
  if (!mpuRead(kAccelXoutH, buffer, 14)) {
    return false;
  }

  constexpr float kAccelScale = 4096.0f;  // LSB/g at ±8 g
  constexpr float kGyroScale = 65.5f;     // LSB/(deg/s) at ±500 deg/s

  float ax = combineInt16(buffer[0], buffer[1]) / kAccelScale;
  float ay = combineInt16(buffer[2], buffer[3]) / kAccelScale;
  float az = combineInt16(buffer[4], buffer[5]) / kAccelScale;
  float gx = combineInt16(buffer[8], buffer[9]) / kGyroScale;
  float gy = combineInt16(buffer[10], buffer[11]) / kGyroScale;
  float gz = combineInt16(buffer[12], buffer[13]) / kGyroScale;

  out.rollDeg = atan2f(ay, az) * 57.2958f;
  out.pitchDeg = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2958f;
  out.gx = gx;
  out.gy = gy;
  out.gz = gz;
  return true;
}

bool gpsRead(GpsData& out) {
#if defined(ARDUINO_ARCH_STM32)
  (void)out;
  return false;
#else
  bool updated = false;
  while (gpsSerial.available() > 0) {
    char c = static_cast<char>(gpsSerial.read());
    if (c == '\n') {
      if (g_nmeaLine.length() > 0) {
        parseNmea(g_nmeaLine, out);
        updated = true;
      }
      g_nmeaLine = "";
    } else if (c != '\r') {
      g_nmeaLine += c;
    }
  }
  return updated;
#endif
}
