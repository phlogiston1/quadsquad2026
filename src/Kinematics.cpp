/**
 * @file Kinematics.cpp
 * @author Sean Benham
 * @brief Converts known motor thrusts to overall quadcopter acceleration
 * @version 0.1
 * @date 2025-10-12
 *
 * @copyright Sean Benham (c) 2025
 *
 */

#include "Kinematics.h"
#include "Constants.h"
#include "Util.h"
#include <cmath>
#include <array>
#include <iostream>

double rotor_pos = QUADCOPTER_ROTOR_DISTANCE/2;

double velocityToThrust(double velocity) {
    return THRUST_COEFF * velocity * velocity;
}

QCAcceleration velocitiesToAccel (MotorVelocities velocities, Rotation3d measuredAngle) {
    double thrust_fl = velocityToThrust(velocities.getFrontLeft());
    double thrust_rl = velocityToThrust(velocities.getRearLeft());
    double thrust_fr = velocityToThrust(velocities.getFrontRight());
    double thrust_rr = velocityToThrust(velocities.getRearRight());

    //note: torque_x means torque about the x axis, etc.
    //since the rotors are opposing each other's torques, we can simply consider the difference between opposite rotors.
    double torque_x = ((thrust_rl - thrust_fr) - (thrust_rr - thrust_fl)) * rotor_pos;
    double torque_y = ((thrust_rl - thrust_fr) + (thrust_rr - thrust_fl)) * rotor_pos;

    //calculate torque about the z axis due to drag from rotors:
    double drag_torque_fl = (velocities.getFrontLeft() * velocities.getFrontLeft()) * ROTOR_DRAG_COEFF;
    double drag_torque_rl = (velocities.getRearLeft() * velocities.getRearLeft()) * ROTOR_DRAG_COEFF;
    double drag_torque_fr = (velocities.getFrontRight() * velocities.getFrontRight()) * ROTOR_DRAG_COEFF;
    double drag_torque_rr = (velocities.getRearRight() * velocities.getRearRight()) * ROTOR_DRAG_COEFF;

    double drag_torque_a = drag_torque_fl + drag_torque_rr;
    double drag_torque_b = drag_torque_fr + drag_torque_rl;

    double torque_z;
    if(FRONT_LEFT_SPINS_CCW) torque_z = drag_torque_b - drag_torque_a;
    else torque_z = drag_torque_a - drag_torque_b;

    //calculate angular acceleration from all of the torques
    Rotation3d angularAccel = Rotation3d(torque_z / QUADCOPTER_MOI, torque_y / QUADCOPTER_MOI, torque_x / QUADCOPTER_MOI);

    //finally, calculate overall thrust and then rotate it to align with the measured quadcopter angle. Using spherical coordinates.
    double thrust = thrust_fl + thrust_rl + thrust_fr + thrust_rr;
    std::array thrustDirection = measuredAngle.thrustDirection();

    return QCAcceleration(
        angularAccel,
        (thrust * thrustDirection[0]) / QUADCOPTER_MASS,
        (thrust * thrustDirection[1]) / QUADCOPTER_MASS,
        (thrust * thrustDirection[2]) / QUADCOPTER_MASS + 9.8 //account for gravity of course
    );

}