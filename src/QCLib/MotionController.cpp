#include "MotionController.h"

/**
 * @file MotionController.cpp
 * @author Sean Benham
 * @brief Implements various motion controllers for the quadcopter
 * @version 0.1
 * @date 2025-10-12
 * 
 */

 //ManualController simply sets the motor velocities to a fixed value
ManualController::ManualController(MotorVelocities initialVels): motorVels(initialVels){
}

MotorVelocities ManualController::calculateMotorVelocities(const QCState& currentState, const Vector3d& targetVelocity, double targetYaw) {
    return motorVels;
}

void ManualController::setMotorVelocities(MotorVelocities newVels) {
    motorVels = newVels;
}