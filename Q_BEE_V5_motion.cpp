#include "Q_BEE_V5_motion.h"
#include "Q_BEE_V5_kinematics.h"
#include <Arduino.h>
#include <math.h>

// Modified set_site(): compute cartesian temp_speed (as before) AND compute servo angle targets and steps
void set_site(int leg, float x, float y, float z){
  float length_x = 0.0f, length_y = 0.0f, length_z = 0.0f;
  if (x != KEEP)    length_x = x - site_now[leg][0];
  if (y != KEEP)    length_y = y - site_now[leg][1];
  if (z != KEEP)    length_z = z - site_now[leg][2];
  float length = sqrt(length_x * length_x + length_y * length_y + length_z * length_z);

  // compute cartesian per-tick speed (keeps prior behaviour)
  if (length < EPSILON){
    temp_speed[leg][0] = 0.0f;
    temp_speed[leg][1] = 0.0f;
    temp_speed[leg][2] = 0.0f;
  } else {
    temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
    temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
    temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;
  }

  // update expected cartesian locations
  if (x != KEEP)    site_expect[leg][0] = x;
  if (y != KEEP)    site_expect[leg][1] = y;
  if (z != KEEP)    site_expect[leg][2] = z;

  // Determine final target cartesian coords (consider KEEP sentinel -> final expected)
  float tx = (x == KEEP) ? site_expect[leg][0] : x;
  float ty = (y == KEEP) ? site_expect[leg][1] : y;
  float tz = (z == KEEP) ? site_expect[leg][2] : z;

  // Compute number of ticks to complete move based on linear distance and (move_speed * speed_multiple)
  int steps = 1;
  if (length < EPSILON) steps = 1; else {
    float perTick = move_speed * speed_multiple; // units per timer tick
    if (perTick < EPSILON) steps = 1; else steps = (int)ceil(length / perTick);
  }

  // Compute target servo angles (deg)
  float alpha_t, beta_t, gamma_t;
  cartesian_to_polar(alpha_t, beta_t, gamma_t, tx, ty, tz);
  float a_t, b_t, c_t;
  polar_to_servo_angles(leg, alpha_t, beta_t, gamma_t, a_t, b_t, c_t);

  // Compute current servo angles from site_now (to have a clean baseline)
  float alpha_c, beta_c, gamma_c;
  cartesian_to_polar(alpha_c, beta_c, gamma_c, site_now[leg][0], site_now[leg][1], site_now[leg][2]);
  float a_c, b_c, c_c;
  polar_to_servo_angles(leg, alpha_c, beta_c, gamma_c, a_c, b_c, c_c);

  // Nominal per-tick delta (kept for compatibility) and -- importantly --
  // store interpolation metadata so the servo service can run a smooth
  // quintic (min-jerk) trajectory rather than a constant-velocity step.
  float sa = (a_t - a_c) / (float)steps;
  float sb = (b_t - b_c) / (float)steps;
  float sc = (c_t - c_c) / (float)steps;

  // Atomically update servo interpolation metadata & targets
  noInterrupts();
  servo_start[leg][0] = a_c; servo_start[leg][1] = b_c; servo_start[leg][2] = c_c;
  servo_target[leg][0] = a_t; servo_target[leg][1] = b_t; servo_target[leg][2] = c_t;
  servo_ticks_total[leg] = (unsigned int)steps;
  servo_tick_idx[leg] = 0;
  // keep nominal per-tick deltas for backward compatibility/useful diagnostics
  servo_step[leg][0] = sa; servo_step[leg][1] = sb; servo_step[leg][2] = sc;
  // update software servo_now baseline to current calculated (keeps interpolation consistent)
  servo_now[leg][0] = a_c; servo_now[leg][1] = b_c; servo_now[leg][2] = c_c;
  interrupts();
}

void wait_reach(int leg){
  while (!(fabsf(site_now[leg][0] - site_expect[leg][0]) <= 0.1f && fabsf(site_now[leg][1] - site_expect[leg][1]) <= 0.1f && fabsf(site_now[leg][2] - site_expect[leg][2]) <= 0.1f)){
    delay(1); // yield to other tasks and avoid tight busy-wait
  }
}

void wait_all_reach(void){
  for (int i = 0; i < 4; i++)    wait_reach(i);
}
