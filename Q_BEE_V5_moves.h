#ifndef Q_BEE_V5_MOVES_H
#define Q_BEE_V5_MOVES_H

#include "Q_BEE_V5_state.h"

void sit(void);
void stand(void);
void turn_left(unsigned int step);
void turn_right(unsigned int step);
void step_forward(unsigned int step);
void step_back(unsigned int step);
void body_left(int i);
void body_right(int i);
void hand_wave(int i);
void hand_shake(int i);
void head_up(int i);
void head_down(int i);
void body_dance(int i);

#endif // Q_BEE_V5_MOVES_H
