// ...existing code...
#ifndef Q_BEE_V6_STATE_H
#define Q_BEE_V6_STATE_H

#include "Q_BEE_V6_defs.h"

// Shared runtime state (defined in the main .ino file)
extern volatile float site_now[4][3];
extern volatile float site_expect[4][3];
extern volatile float temp_speed[4][3]; // cartesian per-tick speed

// Servo interpolation state
extern volatile float servo_target[4][3];
extern volatile float servo_now[4][3];
extern volatile float servo_step[4][3];           // retained for compatibility (nominal per-tick delta)
extern volatile float servo_start[4][3];          // start angles for active interpolation
extern volatile unsigned int servo_ticks_total[4]; // total ticks for current interpolation
extern volatile unsigned int servo_tick_idx[4];    // current interpolation tick index
extern volatile int last_servo_pulse[4][3];       // last written microsecond pulse (hysteresis)

// Motion control
extern float move_speed;
extern float speed_multiple;
extern volatile int rest_counter;

// Global movement configuration defined in main .ino
extern float z_default;
extern float z_up;
extern float z_boot;
extern const float spot_turn_speed;
extern const float leg_move_speed;
extern const float body_move_speed;
extern const float stand_seat_speed;

#endif // Q_BEE_V6_STATE_H
