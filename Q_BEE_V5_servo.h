#ifndef Q_BEE_V5_SERVO_H
#define Q_BEE_V5_SERVO_H

#include <Servo.h>
#include "Q_BEE_V5_defs.h"

// Pin mapping for servos (internal linkage is fine for a header constant)
static const int servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13} };

// Servo hardware and service forward declarations
extern Servo servo[4][3];

void servo_attach(void);
void servo_detach(void);
void servo_service(void);

#endif // Q_BEE_V5_SERVO_H
