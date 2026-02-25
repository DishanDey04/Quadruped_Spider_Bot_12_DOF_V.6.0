#ifndef Q_BEE_V5_MOTION_H
#define Q_BEE_V5_MOTION_H

#include "Q_BEE_V5_state.h"

void set_site(int leg, float x, float y, float z);
void wait_reach(int leg);
void wait_all_reach(void);

#endif // Q_BEE_V5_MOTION_H
