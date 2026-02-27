// ...existing code...
#include "Q_BEE_V6_defs.h"
#include "Q_BEE_V6_bluetooth.h"
#include "Q_BEE_V6_state.h"
#include "Q_BEE_V6_servo.h"
#include "Q_BEE_V6_kinematics.h"
#include "Q_BEE_V6_motion.h"
#include "Q_BEE_V6_moves.h"
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

// ...existing code...
#include "Q_BEE_V6_moves.h"

// Initialize robot and Bluetooth
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
    servo_start[leg][0] = servo_now[leg][0]; servo_start[leg][1] = servo_now[leg][1]; servo_start[leg][2] = servo_now[leg][2];
    servo_ticks_total[leg] = 0; servo_tick_idx[leg] = 0;
    last_servo_pulse[leg][0] = last_servo_pulse[leg][1] = last_servo_pulse[leg][2] = -1;
    interrupts();
  }
  FlexiTimer2::set(SERVO_TICK_MS, servo_service);
  FlexiTimer2::start();
  servo_attach();
  bluetooth_init(); // Initialize Bluetooth module
}

// Main loop: process Bluetooth commands and execute robot movements
void loop() {
  static bool is_moving = false;
  bluetooth_process();

  // If a movement is in progress, ignore Bluetooth commands
  if (is_moving) {
    // Check if all legs have reached their expected positions
    bool all_reached = true;
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 3; j++) {
        if (fabsf(site_now[i][j] - site_expect[i][j]) > 0.1f) {
          all_reached = false;
          break;
        }
      }
      if (!all_reached) break;
    }
    if (all_reached) is_moving = false;
    return;
  }

  // If a Bluetooth command is available and not moving, process it
  if (bluetooth_command_available()) {
    String cmd = bluetooth_get_command();
    // Example: simple command parser (expand as needed)
    if (cmd == "FORWARD") {
      is_moving = true;
      step_forward(1);
    } else if (cmd == "BACK") {
      is_moving = true;
      step_back(1);
    } else if (cmd == "LEFT") {
      is_moving = true;
      turn_left(1);
    } else if (cmd == "RIGHT") {
      is_moving = true;
      turn_right(1);
    } else if (cmd == "SIT") {
      is_moving = true;
      sit();
    } else if (cmd == "STAND") {
      is_moving = true;
      stand();
    } else if (cmd == "DANCE") {
      is_moving = true;
      body_dance(3);
    }
    bluetooth_clear_command();
    return;
  }

  // Idle: can add more idle behavior here if needed
}
