#include "imu_handler.h"

#ifdef BUILD_TX  // Only compile for transmitter

bool IMUHandler::begin(uint8_t sdaPin, uint8_t sclPin, IMUType type) {
  imuType = type;
  
  // Initialize I2C with custom pins
  Wire.begin(sdaPin, sclPin);
  
  if (imuType == IMU_BNO085) {
    // Initialize BNO085
    Serial.println("\n[IMU] Initializing BNO085...");
    
    if (!bno08x.begin_I2C()) {
      Serial.println("[IMU] Failed to find BNO085 chip!");
      Serial.println("[IMU] Check wiring:");
      Serial.printf("     SDA -> GPIO%d\n", sdaPin);
      Serial.printf("     SCL -> GPIO%d\n", sclPin);
      Serial.println("     VCC -> 3.3V");
      Serial.println("     GND -> GND");
      return false;
    }
    
    Serial.println("[IMU] BNO085 found!");
    
    // Enable game rotation vector (sensor fusion with no magnetometer)
    // This gives us accurate roll/pitch without magnetic interference
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000)) {  // 10ms = 100Hz
      Serial.println("[IMU] Failed to enable game rotation vector!");
      return false;
    }
    
    Serial.println("[IMU] BNO085 configured:");
    Serial.println("     Sensor Fusion: Game Rotation Vector (no mag)");
    Serial.println("     Update Rate: 100 Hz");
    Serial.println("     Mode: Built-in sensor fusion (superior accuracy!)");
    
    // Calibrate yaw zero - read initial orientation
    Serial.println("[IMU] Calibrating yaw zero...");
    delay(500);  // Wait for sensor to stabilize
    for (int i = 0; i < 10; i++) {
      if (bno08x.getSensorEvent(&bnoSensorValue)) {
        if (bnoSensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
          float qw = bnoSensorValue.un.gameRotationVector.real;
          float qx = bnoSensorValue.un.gameRotationVector.i;
          float qy = bnoSensorValue.un.gameRotationVector.j;
          float qz = bnoSensorValue.un.gameRotationVector.k;
          
          float siny_cosp = 2.0 * (qw * qz + qx * qy);
          float cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
          yawZeroOffset = atan2(siny_cosp, cosy_cosp) * 57.2958;
          yawZeroCalibrated = true;
          break;
        }
      }
      delay(50);
    }
    if (yawZeroCalibrated) {
      Serial.printf("[IMU] Yaw zero calibrated: %.1f°\n", yawZeroOffset);
    } else {
      Serial.println("[IMU] Warning: Yaw zero calibration failed, using 0°");
      yawZeroOffset = 0.0;
      yawZeroCalibrated = true;
    }
    
  } else {
    // Initialize MPU6050
    Serial.println("\n[IMU] Initializing MPU6050...");
    
    if (!mpu.begin()) {
      Serial.println("[IMU] Failed to find MPU6050 chip!");
      Serial.println("[IMU] Check wiring:");
      Serial.printf("     SDA -> GPIO%d\n", sdaPin);
      Serial.printf("     SCL -> GPIO%d\n", sclPin);
      Serial.println("     VCC -> 3.3V");
      Serial.println("     GND -> GND");
      return false;
    }
    
    Serial.println("[IMU] MPU6050 found!");
    
    // Configure MPU6050 settings for smooth control
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    Serial.println("[IMU] MPU6050 configured:");
    Serial.println("     Accelerometer: ±8G");
    Serial.println("     Gyroscope: ±500°/s");
    Serial.println("     Filter: 21 Hz");
    
    // Perform calibration (only for MPU6050)
    if (!calibrate()) {
      Serial.println("[IMU] Calibration failed!");
      return false;
    }
    
    // For MPU6050, yaw starts at 0 (gyro integration starts from zero)
    // We'll set the zero offset to 0 since yaw is relative
    yawZeroOffset = 0.0;
    yawZeroCalibrated = true;
    Serial.println("[IMU] Yaw zero set to 0° (MPU6050 uses relative yaw)");
  }
  
  Serial.println("[IMU] Control Mapping:");
  Serial.println("     Roll:  Tilt angle left/right (like joystick position)");
  Serial.println("     Pitch: Tilt angle forward/back (like joystick position)");
  Serial.println("     Yaw:   Rotation angle around Z axis (like joystick position)");
  Serial.println("     Mode: ANGLE-BASED (holds position when tilted/rotated)");
  Serial.println();
  
  return true;
}

bool IMUHandler::calibrate() {
  Serial.println("\n[IMU] Calibrating...");
  Serial.println("     Keep the sensor still!");
  delay(1000);
  
  float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
  float accelXSum = 0, accelYSum = 0, accelZSum = 0;
  const int numSamples = 100;
  
  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    gyroXSum += g.gyro.x;
    gyroYSum += g.gyro.y;
    gyroZSum += g.gyro.z;
    accelXSum += a.acceleration.x;
    accelYSum += a.acceleration.y;
    accelZSum += a.acceleration.z;
    
    delay(10);
  }
  
  gyroXOffset = gyroXSum / numSamples;
  gyroYOffset = gyroYSum / numSamples;
  gyroZOffset = gyroZSum / numSamples;
  accelXOffset = accelXSum / numSamples;
  accelYOffset = accelYSum / numSamples;
  accelZOffset = (accelZSum / numSamples) - 9.81;  // Remove gravity from Z axis
  
  Serial.println("[IMU] Calibration complete!");
  Serial.printf("     Gyro offsets: X=%.3f Y=%.3f Z=%.3f rad/s\n", 
                gyroXOffset, gyroYOffset, gyroZOffset);
  Serial.printf("     Accel offsets: X=%.2f Y=%.2f Z=%.2f m/s²\n", 
                accelXOffset, accelYOffset, accelZOffset);
  
  return true;
}

void IMUHandler::update() {
  if (imuType == IMU_BNO085) {
    // BNO085: Use built-in sensor fusion
    if (bno08x.wasReset()) {
      Serial.println("[IMU] BNO085 was reset!");
      // Re-enable reports
      bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    }
    
    if (bno08x.getSensorEvent(&bnoSensorValue)) {
      // Check if we got game rotation vector data
      if (bnoSensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        // Get quaternion values (w, x, y, z)
        float qw = bnoSensorValue.un.gameRotationVector.real;
        float qx = bnoSensorValue.un.gameRotationVector.i;
        float qy = bnoSensorValue.un.gameRotationVector.j;
        float qz = bnoSensorValue.un.gameRotationVector.k;
        
        // Convert quaternion to Euler angles (roll, pitch, yaw)
        // Roll (rotation around X axis)
        float sinr_cosp = 2.0 * (qw * qx + qy * qz);
        float cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        rollAngleDeg = atan2(sinr_cosp, cosr_cosp) * 57.2958;
        
        // Pitch (rotation around Y axis)
        float sinp = 2.0 * (qw * qy - qz * qx);
        if (fabs(sinp) >= 1.0) {
          pitchAngleDeg = copysign(90.0, sinp);  // Use 90 degrees if out of range
        } else {
          pitchAngleDeg = asin(sinp) * 57.2958;
        }
        
        // Yaw (rotation around Z axis)
        float siny_cosp = 2.0 * (qw * qz + qx * qy);
        float cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        float rawYawDeg = atan2(siny_cosp, cosy_cosp) * 57.2958;
        
        // Apply yaw zero offset (calibrated at startup)
        if (yawZeroCalibrated) {
          yawAngleDeg = rawYawDeg - yawZeroOffset;
          // Normalize to -180 to 180 range
          while (yawAngleDeg > 180.0) yawAngleDeg -= 360.0;
          while (yawAngleDeg < -180.0) yawAngleDeg += 360.0;
        } else {
          yawAngleDeg = rawYawDeg;
        }
        
        // Map angles to normalized values (-1.0 to 1.0)
        rollNormalized = rollAngleDeg / rollSensitivity;
        rollNormalized = constrain(rollNormalized, -1.0, 1.0);
        
        pitchNormalized = pitchAngleDeg / pitchSensitivity;
        pitchNormalized = constrain(pitchNormalized, -1.0, 1.0);
        
        yawNormalized = yawAngleDeg / yawSensitivity;
        yawNormalized = constrain(yawNormalized, -1.0, 1.0);
      }
    }
    
  } else {
    // MPU6050: Manual angle calculation
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    
    // Apply calibration offsets
    float gyroXRaw = gyro.gyro.x - gyroXOffset;  // rad/s
    float gyroYRaw = gyro.gyro.y - gyroYOffset;
    float gyroZRaw = gyro.gyro.z - gyroZOffset;
    
    accelX = accel.acceleration.x - accelXOffset;  // m/s²
    accelY = accel.acceleration.y - accelYOffset;
    accelZ = accel.acceleration.z - accelZOffset;
    
    // Convert gyro from rad/s to degrees/s
    gyroXDeg = gyroXRaw * 57.2958;  // 180/PI
    gyroYDeg = gyroYRaw * 57.2958;
    gyroZDeg = gyroZRaw * 57.2958;
    
    // Calculate actual tilt angles from accelerometer (like a joystick position!)
    // Roll: Tilt angle left/right (positive = right tilt)
    rollAngleDeg = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 57.2958;
    
    // Pitch: Tilt angle forward/back (positive = forward tilt)
    pitchAngleDeg = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 57.2958;
    
    // Yaw: For MPU6050, use gyro integration (no magnetometer, so yaw drifts)
    // Integrate gyro Z (yaw rate) to get yaw angle
    static unsigned long lastUpdateMs = 0;
    unsigned long now = millis();
    if (lastUpdateMs > 0) {
      float dt = (now - lastUpdateMs) / 1000.0;  // Time delta in seconds
      yawAngleDeg += gyroZDeg * dt;  // Integrate yaw rate
      // Keep yaw in reasonable range (-180 to 180)
      if (yawAngleDeg > 180.0) yawAngleDeg -= 360.0;
      if (yawAngleDeg < -180.0) yawAngleDeg += 360.0;
    }
    lastUpdateMs = now;
    
    // Map tilt angles to normalized values (-1.0 to 1.0)
    rollNormalized = rollAngleDeg / rollSensitivity;
    rollNormalized = constrain(rollNormalized, -1.0, 1.0);
    
    pitchNormalized = pitchAngleDeg / pitchSensitivity;
    pitchNormalized = constrain(pitchNormalized, -1.0, 1.0);
    
    yawNormalized = yawAngleDeg / yawSensitivity;
    yawNormalized = constrain(yawNormalized, -1.0, 1.0);
  }
}

float IMUHandler::getRoll() {
  return rollNormalized;
}

float IMUHandler::getPitch() {
  return pitchNormalized;
}

float IMUHandler::getYaw() {
  return yawNormalized;
}

void IMUHandler::getRawGyro(float &gyroX, float &gyroY, float &gyroZ) {
  gyroX = gyroXDeg;
  gyroY = gyroYDeg;
  gyroZ = gyroZDeg;
}

void IMUHandler::getAngles(float &rollAngle, float &pitchAngle, float &yawAngle) {
  rollAngle = rollAngleDeg;
  pitchAngle = pitchAngleDeg;
  yawAngle = yawAngleDeg;
}

float IMUHandler::getTiltAngle() {
  // Calculate pitch angle in degrees (positive = nose up)
  return atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 57.2958;
}

#endif // BUILD_TX

