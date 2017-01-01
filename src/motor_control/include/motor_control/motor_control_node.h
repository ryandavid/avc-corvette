#ifndef MOTOR_CONTROL_NODE_H
#define MOTOR_CONTROL_NODE_H

// Min/max ranges for the joystick.  2^16 - 1
const int kJoystickAxisMax = 32767;
const int kJoystickAxisMin = -32767;

// The amount of counts from the joystick to assume its neutral.
const int kJoystickDeadband = 75;

#endif