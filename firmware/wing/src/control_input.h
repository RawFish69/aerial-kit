#pragma once

struct ControlInput {
  float roll = 0.0f;     // -1..1, right roll positive
  float pitch = 0.0f;    // -1..1, nose-up positive
  float yaw = 0.0f;      // -1..1, right yaw positive
  float throttle = 0.0f; // 0..1

  ControlInput() = default;
  ControlInput(float roll_, float pitch_, float yaw_, float throttle_)
      : roll(roll_), pitch(pitch_), yaw(yaw_), throttle(throttle_) {}
};

void readControlInput(ControlInput& out);
