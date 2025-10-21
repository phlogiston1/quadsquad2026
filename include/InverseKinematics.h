#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "Quadcopter.h"

/**
 * @brief Implements inverse kinematics for the quadcopter
 *
 * Step 1: Given a desired acceleration vector and yaw rate, calculate the ideal orientation and thrust for the quadcopter, 
 * ignoring frame and motor acceleration limits.
 * Step 2: Given the ideal orientation and thrust, calculate the motor velocities that would best achieve this
 * We will have two different kinematics modes, one will adjust the thrust to best match the *direction* of the acceleration vector,
 * and the other will adjust the thrust to best match the *magnitude* of the acceleration vector.
 * I'm not sure which will work better, so I'll implement both and see.
 */
struct InverseKinematicResult {
    MotorVelocities motorVelocities;
    QCAcceleration achievedAccel;
    double errorMagnitude;
};

struct TargetQCState {
    Rotation3d targetAngle;
    double targetThrust;
    double targetYawRate;
};

TargetQCState calculateTargetState(QCState currentState, Vector3d targetAccel, double targetYaw);

InverseKinematicResult optimizeMotorVelocitiesForDirection(QCState currentState, TargetQCState targetState, double timestep);

InverseKinematicResult calculateMotorVelocitiesForAccel(QCState currentState, TargetQCState targetState);

#endif