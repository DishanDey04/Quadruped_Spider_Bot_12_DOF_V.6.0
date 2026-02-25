#include "Q_BEE_V5_moves.h"
#include "Q_BEE_V5_state.h"
#include "Q_BEE_V5_defs.h"
#include "Q_BEE_V5_motion.h"
#include <Arduino.h>

void sit(void){
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) set_site(leg, KEEP, KEEP, z_boot);
  wait_all_reach();
}

void stand(void){
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) set_site(leg, KEEP, KEEP, z_default);
  wait_all_reach();
}

void turn_left(unsigned int step){
  move_speed = spot_turn_speed;
  while (step-- > 0){
    if (site_now[3][1] == y_start){
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, turn_x1 - x_offset, turn_y1, z_default);set_site(1, turn_x0 - x_offset, turn_y0, z_default);set_site(2, turn_x1 + x_offset, turn_y1, z_default);set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(0, turn_x1 + x_offset, turn_y1, z_default);set_site(1, turn_x0 + x_offset, turn_y0, z_default);set_site(2, turn_x1 - x_offset, turn_y1, z_default);set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);set_site(1, x_default + x_offset, y_start, z_up);set_site(2, x_default - x_offset, y_start + y_step, z_default);set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else{
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, turn_x0 + x_offset, turn_y0, z_up);set_site(1, turn_x1 + x_offset, turn_y1, z_default);set_site(2, turn_x0 - x_offset, turn_y0, z_default);set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();
      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(0, turn_x0 - x_offset, turn_y0, z_default);set_site(1, turn_x1 - x_offset, turn_y1, z_default);set_site(2, turn_x0 + x_offset, turn_y0, z_default);set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();
      set_site(0, x_default - x_offset, y_start + y_step, z_default);set_site(1, x_default - x_offset, y_start + y_step, z_default);set_site(2, x_default + x_offset, y_start, z_up);set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

void turn_right(unsigned int step){
  move_speed = spot_turn_speed;
  while (step-- > 0){
    if (site_now[2][1] == y_start){
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, turn_x0 - x_offset, turn_y0, z_default);set_site(1, turn_x1 - x_offset, turn_y1, z_default);set_site(2, turn_x0 + x_offset, turn_y0, z_up);set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(0, turn_x0 + x_offset, turn_y0, z_default);set_site(1, turn_x1 + x_offset, turn_y1, z_default);set_site(2, turn_x0 - x_offset, turn_y0, z_default);set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();
      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);set_site(1, x_default + x_offset, y_start, z_default);set_site(2, x_default - x_offset, y_start + y_step, z_default);set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else{
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, turn_x1 + x_offset, turn_y1, z_default);set_site(1, turn_x0 + x_offset, turn_y0, z_up);set_site(2, turn_x1 - x_offset, turn_y1, z_default);set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(0, turn_x1 - x_offset, turn_y1, z_default);set_site(1, turn_x0 - x_offset, turn_y0, z_default);set_site(2, turn_x1 + x_offset, turn_y1, z_default);set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();
      set_site(0, x_default - x_offset, y_start + y_step, z_default);set_site(1, x_default - x_offset, y_start + y_step, z_default);set_site(2, x_default + x_offset, y_start, z_default);set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

void step_forward(unsigned int step){
  move_speed = leg_move_speed;
  while (step-- > 0)  {
    if (site_now[2][1] == y_start){
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2.0f * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2.0f * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);set_site(1, x_default + x_offset, y_start + 2.0f * y_step, z_default);set_site(2, x_default - x_offset, y_start + y_step, z_default);set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2.0f * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else{
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2.0f * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2.0f * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);set_site(1, x_default - x_offset, y_start + y_step, z_default);set_site(2, x_default + x_offset, y_start, z_default);set_site(3, x_default + x_offset, y_start + 2.0f * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2.0f * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

void step_back(unsigned int step){
  move_speed = leg_move_speed;
  while (step-- > 0){
    if (site_now[3][1] == y_start){
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2.0f * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2.0f * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2.0f * y_step, z_default);set_site(1, x_default + x_offset, y_start, z_default);set_site(2, x_default - x_offset, y_start + y_step, z_default);set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2.0f * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else{
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2.0f * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2.0f * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);set_site(1, x_default - x_offset, y_start + y_step, z_default);set_site(2, x_default + x_offset, y_start + 2.0f * y_step, z_default);set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2.0f * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

void body_left(int i){
  set_site(0, site_now[0][0] + i, KEEP, KEEP);set_site(1, site_now[1][0] + i, KEEP, KEEP);set_site(2, site_now[2][0] - i, KEEP, KEEP);set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void body_right(int i){
  set_site(0, site_now[0][0] - i, KEEP, KEEP);set_site(1, site_now[1][0] - i, KEEP, KEEP);set_site(2, site_now[2][0] + i, KEEP, KEEP);set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void hand_wave(int i){
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1.0f;
  if (site_now[3][1] == y_start){
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++){
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1.0f;
    body_left(15);
  }
  else{
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++){
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1.0f;
    body_right(15);
  }
}

void hand_shake(int i){
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1.0f;
  if (site_now[3][1] == y_start){
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++){
      set_site(2, x_default - 30, y_start + 2.0f * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2.0f * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1.0f;
    body_left(15);
  }
  else{
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++){
      set_site(0, x_default - 30, y_start + 2.0f * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2.0f * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1.0f;
    body_right(15);
  }
}

void head_up(int i){
  set_site(0, KEEP, KEEP, site_now[0][2] - i);set_site(1, KEEP, KEEP, site_now[1][2] + i);set_site(2, KEEP, KEEP, site_now[2][2] - i);set_site(3, KEEP, KEEP, site_now[3][2] + i);wait_all_reach();
}

void head_down(int i){
  set_site(0, KEEP, KEEP, site_now[0][2] + i);set_site(1, KEEP, KEEP, site_now[1][2] - i);set_site(2, KEEP, KEEP, site_now[2][2] + i);set_site(3, KEEP, KEEP, site_now[3][2] - i);wait_all_reach();
}

void body_dance(int i){
  float z_abs = z_default;
  z_default = -50;
  float x_tmp;
  float y_tmp;
  float z_tmp;
  float body_dance_speed = 2.0f;
  sit();
  move_speed = 1.0f;
  set_site(0, x_default, y_default, KEEP);set_site(1, x_default, y_default, KEEP);set_site(2, x_default, y_default, KEEP);set_site(3, x_default, y_default, KEEP);
  wait_all_reach();
  //stand();
  set_site(0, x_default, y_default, z_default - 20);set_site(1, x_default, y_default, z_default - 20);set_site(2, x_default, y_default, z_default - 20);set_site(3, x_default, y_default, z_default - 20);
  wait_all_reach();
  move_speed = body_dance_speed;
  head_up(30);
  for (int j = 0; j < i; j++){
    if (j > i / 4)      move_speed = body_dance_speed * 2.0f;
    if (j > i / 2)      move_speed = body_dance_speed * 3.0f;
    set_site(0, KEEP, y_default - 20, KEEP);set_site(1, KEEP, y_default + 20, KEEP);set_site(2, KEEP, y_default - 20, KEEP);set_site(3, KEEP, y_default + 20, KEEP);
    wait_all_reach();
    set_site(0, KEEP, y_default + 20, KEEP);set_site(1, KEEP, y_default - 20, KEEP);set_site(2, KEEP, y_default + 20, KEEP);set_site(3, KEEP, y_default - 20, KEEP);
    wait_all_reach();
  }
  move_speed = body_dance_speed;
  head_down(30);
  z_default = z_abs;
  stand();
}