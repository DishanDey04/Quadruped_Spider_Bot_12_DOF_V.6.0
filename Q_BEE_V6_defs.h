// ...existing code...
#ifndef Q_BEE_V6_DEFS_H
#define Q_BEE_V6_DEFS_H

// Basic robot geometry and small utility helpers
#include <math.h>

static const float length_a = 55.0f;
static const float length_b = 77.5f;
static const float length_c = 27.5f;
static const float length_side = 71.0f;
static const float z_absolute = -28.0f;

// Movement constants that are compile-time constants
static const float x_default = 62.0f;
static const float x_offset = 0.0f;
static const float y_start = 0.0f;
static const float y_step = 40.0f;
static const float y_default = x_default;

// Small utilities and guards
static const float KEEP = 255.0f; // sentinel: "keep this coordinate"
static const float pi = 3.14159265358979323846f;
static const float EPSILON = 0.0001f;

static inline float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }

// Turn geometry (precomputed constants used by motion primitives)
static const float temp_a = sqrt((2.0f * x_default + length_side) * (2.0f * x_default + length_side) + y_step * y_step);
static const float temp_b = 2.0f * (y_start + y_step) + length_side;
static const float temp_c = sqrt((2.0f * x_default + length_side) * (2.0f * x_default + length_side) + (2.0f * y_start + y_step + length_side) * (2.0f * y_start + y_step + length_side));
static const float temp_alpha = acos(clampf(((temp_a * temp_a + temp_b * temp_b - temp_c * temp_c) / (2.0f * temp_a * temp_b)), -1.0f, 1.0f));
static const float turn_x1 = (temp_a - length_side) / 2.0f;
static const float turn_y1 = y_start + y_step / 2.0f;
static const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
static const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

#endif // Q_BEE_V6_DEFS_H
