// ...existing code...
#ifndef Q_BEE_V6_KINEMATICS_H
#define Q_BEE_V6_KINEMATICS_H

#include "Q_BEE_V6_defs.h"

// Convert cartesian coords to leg polar angles (alpha, beta, gamma)
static inline void cartesian_to_polar(float &alpha, float &beta, float &gamma, float x, float y, float z){
  float v, w;
  w = (x >= 0 ? 1.0f : -1.0f) * (sqrt(x * x + y * y));
  v = w - length_c;
  float denom = 2.0f * length_a * sqrt(v * v + z * z);
  float aarg;
  if (denom < EPSILON) aarg = 1.0f; else aarg = (length_a * length_a - length_b * length_b + v * v + z * z) / denom;
  aarg = clampf(aarg, -1.0f, 1.0f);
  alpha = atan2(z, v) + acos(aarg);
  float barg = (length_a * length_a + length_b * length_b - v * v - z * z) / (2.0f * length_a * length_b);
  barg = clampf(barg, -1.0f, 1.0f);
  beta = acos(barg);
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  // convert to degrees
  alpha = alpha / pi * 180.0f;
  beta = beta / pi * 180.0f;
  gamma = gamma / pi * 180.0f;
}

// Produces servo-friendly angles (clamped to [0,180]) based on leg orientation
static inline void polar_to_servo_angles(int leg, float alpha, float beta, float gamma, float &a_out, float &b_out, float &c_out){
  float a = alpha, b = beta, c = gamma;
  if (leg == 0){
    a = 90.0f - a; b = b; c += 90.0f;
  }
  else if (leg == 1){
    a += 90.0f; b = 180.0f - b; c = 90.0f - c;
  }
  else if (leg == 2){
    a += 90.0f; b = 180.0f - b; c = 90.0f - c;
  }
  else if (leg == 3){
    a = 90.0f - a; b = b; c += 90.0f;
  }
  a_out = clampf(a, 0.0f, 180.0f);
  b_out = clampf(b, 0.0f, 180.0f);
  c_out = clampf(c, 0.0f, 180.0f);
}

#endif // Q_BEE_V6_KINEMATICS_H
