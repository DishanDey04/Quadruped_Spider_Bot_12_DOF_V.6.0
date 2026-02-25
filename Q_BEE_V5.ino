#include "Q_BEE_V5_defs.h"
#include "Q_BEE_V5_state.h"
#include "Q_BEE_V5_servo.h"
#include "Q_BEE_V5_kinematics.h"
#include "Q_BEE_V5_motion.h"
#include "Q_BEE_V5_moves.h"
#include <FlexiTimer2.h>

/* variables for movement ----------------------------------------------------*/
float z_default = -50.0f, z_up = -30.0f, z_boot = z_absolute;
volatile float site_now[4][3]; volatile float site_expect[4][3];
volatile float temp_speed[4][3];
float move_speed;
float speed_multiple = 1.0f;
const float spot_turn_speed = 4.0f;
const float leg_move_speed = 8.0f;
const float body_move_speed = 3.0f;
const float stand_seat_speed = 1.0f;
volatile int rest_counter;     

// new arrays for angle interpolation (degrees)
volatile float servo_target[4][3]; // desired final servo angles for each joint
volatile float servo_now[4][3];    // current servo angles tracked in software
volatile float servo_step[4][3];   // nominal per-tick angle increment (kept for compatibility)
volatile float servo_start[4][3];  // interpolation start angles
volatile unsigned int servo_ticks_total[4]; // total ticks for ongoing interpolation
volatile unsigned int servo_tick_idx[4];   // current tick index for interpolation
volatile int last_servo_pulse[4][3]; // last pulse written (microseconds) to avoid redundant writes

// Configuration: interpolation tick period (ms) and microsecond mapping for `writeMicroseconds`
const unsigned int SERVO_TICK_MS = 10; // 10 ms -> 100 Hz interpolation (smooth & responsive)
const int SERVO_MIN_US = 1000;         // microsecond mapping for 0 degrees (adjust if needed)
const int SERVO_MAX_US = 2000;         // microsecond mapping for 180 degrees (adjust if needed)

#include "Q_BEE_V5_moves.h"

void setup(){
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < 4; i++) for (int j = 0; j < 3; j++) site_now[i][j] = site_expect[i][j];
  for (int leg = 0; leg < 4; leg++){
    float a, b, c;
    cartesian_to_polar(a,b,c, site_now[leg][0], site_now[leg][1], site_now[leg][2]);
    float aTemp, bTemp, cTemp;
    polar_to_servo_angles(leg, a, b, c, aTemp, bTemp, cTemp);
    noInterrupts();
    servo_now[leg][0] = aTemp; servo_now[leg][1] = bTemp; servo_now[leg][2] = cTemp;
    servo_target[leg][0] = servo_now[leg][0]; servo_target[leg][1] = servo_now[leg][1]; servo_target[leg][2] = servo_now[leg][2];
    servo_step[leg][0] = servo_step[leg][1] = servo_step[leg][2] = 0.0f;
    // initialize new interpolation metadata
    servo_start[leg][0] = servo_now[leg][0]; servo_start[leg][1] = servo_now[leg][1]; servo_start[leg][2] = servo_now[leg][2];
    servo_ticks_total[leg] = 0; servo_tick_idx[leg] = 0;
    last_servo_pulse[leg][0] = last_servo_pulse[leg][1] = last_servo_pulse[leg][2] = -1;
    interrupts();
  }
  FlexiTimer2::set(SERVO_TICK_MS, servo_service); // interpolation tick (configurable)
  FlexiTimer2::start();
  servo_attach();
}

void loop()
{
  // Demo loop: exercise every movement primitive with short pauses so you can observe behavior

  // ensure robot is standing before demo
  stand();
  delay(500);

  // locomotion
  step_forward(1);
  delay(300);
  step_back(1);
  delay(300);

  // turning
  turn_left(1);
  delay(300);
  turn_right(1);
  delay(300);

  // body shifts
  body_left(15);
  delay(300);
  body_right(15);
  delay(300);

  // hand / head gestures
  hand_wave(3);
  delay(300);
  hand_shake(3);
  delay(300);
  head_up(15);
  delay(300);
  head_down(15);
  delay(300);

  // dance + sit/stand
  body_dance(3);
  delay(500);
  sit();
  delay(800);
  stand();
  delay(800);

  // brief idle before repeating demo
  delay(1000);
}
