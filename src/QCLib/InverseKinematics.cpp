/**
 * @file InverseKinematics.cpp
 * @author Sean Benham
 * @brief Takes in a desired quadcopter acceleration and converts it to optimal motor velocities
 * @version 0.1
 * @date 2025-10-12
 * 
 * @copyright Copyright (c) 2025
 *
 */

#include "InverseKinematics.h"
#include "Quadcopter.h"
#include "Util.h"
#include "Configuration.h"
#include <cmath>
#include <iostream>

double velocityToThrust(double velocity) {
    return THRUST_COEFF * velocity * velocity;
}

double thrustToVelocity(double thrust) {
    if(thrust < 0) return 0;
    return std::sqrt(thrust / THRUST_COEFF);
}

TargetQCState calculateTargetState(QCState currentState, Vector3d targetAccel, double targetYawRate) {
    // Frame convention: +X forward, +Y right, +Z down (NED)

    // 1. Get current yaw (assumed to be around Z axis)
    double fixed_yaw = currentState.getPose().getRotation().getYaw();

    // 2. Normalize targetAccel to get direction (z_body axis)
    Vector3d z_body = targetAccel.normalized(); // body Z axis in world frame (aligned with thrust direction)

    // 3. Compute desired x_c from yaw (projected on horizontal plane)
    double cy = cos(fixed_yaw);
    double sy = sin(fixed_yaw);
    Vector3d x_c(cy, sy, 0); // forward direction in world frame

    // 4. Compute orthogonal body axes using Gram-Schmidt process
    Vector3d y_body = z_body.cross(x_c);

    // Handle degenerate case if targetAccel is vertical (to avoid zero vector)
    if (y_body.getMagnitude() < 1e-6) {
        y_body = Vector3d(0, 1, 0); // pick arbitrary orthogonal vector
    }

    y_body = y_body.normalized();
    Vector3d x_body = y_body.cross(z_body).normalized();

    // 5. Build rotation from body axes
    Rotation3d targetAngle = Rotation3d::fromRotationMatrix(x_body, y_body, z_body);

    // 6. Compute thrust (F = ma)
    double targetThrust = targetAccel.getMagnitude() * QUADCOPTER_MASS;

    return TargetQCState{targetAngle, targetThrust, targetYawRate};
}

InverseKinematicResult optimizeMotorVelocitiesForDirection(QCState currentState, TargetQCState targetState, double timestep) {
    //Step 1: Find the theretical forces at the points (width/2,0) and (0,width/2) that would produce the desired angular acceleration for pitch and roll.
    double desiredAngularAccelPitch = (targetState.targetAngle.getPitch() - currentState.getPose().getRotation().getPitch()) / timestep;
    double desiredAngularAccelRoll = (targetState.targetAngle.getRoll() - currentState.getPose().getRotation().getRoll()) / timestep;

    double torque_x = (desiredAngularAccelPitch * QUADCOPTER_MOI) / (QUADCOPTER_ROTOR_DISTANCE/2);
    double torque_y = (desiredAngularAccelRoll * QUADCOPTER_MOI) / (QUADCOPTER_ROTOR_DISTANCE/2);

    //Step 2: Find the motor velocity needed to produce the desired yaw rate, from rotor drag torque=v^2 * DRAG_COEFF
    

}