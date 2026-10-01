#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#ifdef BUILD_TX  // Only include for transmitter

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

/**
 * IMU Handler for Roll, Pitch, and Yaw Control
 * 
 * This module handles MPU6050 and BNO085 IMU initialization, calibration, and reading.
 * It provides roll, pitch, and yaw control values based on actual orientation,
 * perfect for intuitive joystick-style drone control (hold position).
 * 
 * Supported IMUs:
 * - MPU6050: Basic 6-axis IMU with manual angle calculation (yaw from gyro integration)
 * - BNO085: Advanced 9-axis IMU with built-in sensor fusion (more accurate!)
 */

// IMU Type Selection
enum IMUType {
  IMU_MPU6050 = 0,  // Basic 6-axis IMU
  IMU_BNO085 = 1    // Advanced 9-axis IMU with sensor fusion
};

class IMUHandler {
public:
  /**
   * Initialize the IMU handler
   * @param sdaPin I2C SDA pin
   * @param sclPin I2C SCL pin
   * @param type IMU type to use (IMU_MPU6050 or IMU_BNO085)
   * @return true if successful, false otherwise
   */
  bool begin(uint8_t sdaPin, uint8_t sclPin, IMUType type = IMU_MPU6050);
  
  /**
   * Update IMU readings and calculate roll/pitch values
   * Call this regularly (e.g., at your RC send frequency)
   */
  void update();
  
  /**
   * Get normalized roll value (-1.0 to 1.0)
   * Based on actual tilt angle left/right (like a joystick position)
   */
  float getRoll();
  
  /**
   * Get normalized pitch value (-1.0 to 1.0)
   * Based on actual tilt angle forward/back (like a joystick position)
   */
  float getPitch();
  
  /**
   * Get normalized yaw value (-1.0 to 1.0)
   * Based on rotation angle around Z axis (like a joystick position)
   */
  float getYaw();
  
  /**
   * Get raw gyro readings in degrees/sec (for debugging)
   */
  void getRawGyro(float &gyroX, float &gyroY, float &gyroZ);
  
  /**
   * Get roll, pitch, and yaw angles in degrees (for debugging)
   */
  void getAngles(float &rollAngle, float &pitchAngle, float &yawAngle);
  
  /**
   * Get current tilt angle from accelerometer (for debugging)
   */
  float getTiltAngle();
  
  /**
   * Set sensitivity for roll control (degrees of tilt for full deflection)
   * Default: 45.0 degrees (tilting 45° = full stick)
   */
  void setRollSensitivity(float sensitivity) { rollSensitivity = sensitivity; }
  
  /**
   * Set sensitivity for pitch control (degrees of tilt for full deflection)
   * Default: 45.0 degrees (tilting 45° = full stick)
   */
  void setPitchSensitivity(float sensitivity) { pitchSensitivity = sensitivity; }
  
  /**
   * Set sensitivity for yaw control (degrees of rotation for full deflection)
   * Default: 45.0 degrees (rotating 45° = full stick)
   */
  void setYawSensitivity(float sensitivity) { yawSensitivity = sensitivity; }

private:
  // IMU instances
  Adafruit_MPU6050 mpu;
  Adafruit_BNO08x bno08x;
  sh2_SensorValue_t bnoSensorValue;
  
  // Active IMU type
  IMUType imuType = IMU_MPU6050;
  
  // Calibration offsets (for MPU6050)
  float gyroXOffset = 0.0;
  float gyroYOffset = 0.0;
  float gyroZOffset = 0.0;
  float accelXOffset = 0.0;
  float accelYOffset = 0.0;
  float accelZOffset = 0.0;
  
  // Current readings (calibrated)
  float gyroXDeg = 0.0;  // degrees/sec
  float gyroYDeg = 0.0;
  float gyroZDeg = 0.0;
  float accelX = 0.0;    // m/s²
  float accelY = 0.0;
  float accelZ = 0.0;
  
  // Normalized output values (-1.0 to 1.0)
  float rollNormalized = 0.0;
  float pitchNormalized = 0.0;
  float yawNormalized = 0.0;
  
  // Current angle values (for debugging)
  float rollAngleDeg = 0.0;
  float pitchAngleDeg = 0.0;
  float yawAngleDeg = 0.0;
  
  // Yaw zero offset (calibrated at startup)
  float yawZeroOffset = 0.0;
  bool yawZeroCalibrated = false;
  
  // Sensitivity settings
  float rollSensitivity = 45.0;   // degrees of tilt for full stick deflection
  float pitchSensitivity = 45.0;  // degrees of tilt for full stick deflection
  float yawSensitivity = 45.0;    // degrees of rotation for full stick deflection
  
  /**
   * Perform IMU calibration
   * @return true if successful
   */
  bool calibrate();
};

#endif // BUILD_TX

#endif // IMU_HANDLER_H

