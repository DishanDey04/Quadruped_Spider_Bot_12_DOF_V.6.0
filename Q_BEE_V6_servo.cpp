// ...existing code...
#include "Q_BEE_V6_servo.h"
#include "Q_BEE_V6_state.h"
#include "Q_BEE_V6_defs.h"
#include <Arduino.h>
#include <math.h>

// Define the servo hardware instance (previously in the .ino)
Servo servo[4][3];

// Map a floating `angle` in degrees to a safe microsecond pulse for writeMicroseconds().
// Adjust SERVO_MIN_US / SERVO_MAX_US in the .ino if your servos require different ranges.
static inline int angle_to_pulse(float ang){
  ang = clampf(ang, 0.0f, 180.0f);
  const int MIN_US = 1000;
  const int MAX_US = 2000;
  return (int)(MIN_US + (ang / 180.0f) * (MAX_US - MIN_US) + 0.5f);
}

void servo_attach(void){
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++) {
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
}

void servo_detach(void){
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++) {
      servo[i][j].detach();
      delay(100);
    }
}

// Keep this callback short: increment angles and write servos only
void servo_service(void){
  for (int i = 0; i < 4; i++){
    // update cartesian position (keeps existing behaviour so site_now follows site_expect)
    for (int j = 0; j < 3; j++){
      float diff = site_now[i][j] - site_expect[i][j];
      float ts = temp_speed[i][j];
      if (fabsf(diff) >= fabsf(ts)) site_now[i][j] += ts; else site_now[i][j] = site_expect[i][j];
    }

    // Smooth quintic (min-jerk) interpolation per-leg to eliminate jerk/jitter.
    unsigned int steps = servo_ticks_total[i];
    unsigned int idx = servo_tick_idx[i];
    if (steps > 1 && idx < steps){
      float s = (float)idx / (float)steps;
      float s2 = s * s;
      float s3 = s2 * s;
      float s4 = s2 * s2;
      float s5 = s3 * s2;
      float h = 10.0f * s3 - 15.0f * s4 + 6.0f * s5; // quintic smoothing curve
      for (int j = 0; j < 3; j++){
        float start = servo_start[i][j];
        float target = servo_target[i][j];
        float ang = start + (target - start) * h;
        servo_now[i][j] = ang;
        int pulse = angle_to_pulse(ang);
        if (pulse != last_servo_pulse[i][j]){ // avoid redundant writes (reduces jitter)
          servo[i][j].writeMicroseconds(pulse);
          last_servo_pulse[i][j] = pulse;
        }
      }
      servo_tick_idx[i]++;
      if (servo_tick_idx[i] >= steps){
        // ensure exact final values when interpolation completes
        for (int j = 0; j < 3; j++){
          servo_now[i][j] = servo_target[i][j];
          int pulse = angle_to_pulse(servo_now[i][j]);
          if (pulse != last_servo_pulse[i][j]){
            servo[i][j].writeMicroseconds(pulse);
            last_servo_pulse[i][j] = pulse;
          }
        }
        servo_ticks_total[i] = 0;
        servo_tick_idx[i] = 0;
      }
    } else {
      // no interpolation scheduled -> hold target exactly
      for (int j = 0; j < 3; j++){
        float ang = clampf(servo_target[i][j], 0.0f, 180.0f);
        servo_now[i][j] = ang;
        int pulse = angle_to_pulse(ang);
        if (pulse != last_servo_pulse[i][j]){
          servo[i][j].writeMicroseconds(pulse);
          last_servo_pulse[i][j] = pulse;
        }
      }
    }
  }
  rest_counter++;
}
