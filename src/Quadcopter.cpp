#include "Quadcopter.h"
#include "Kinematics.h"
#include "Util.h"
#include "Constants.h"
#include <array>
#include <cmath>


MotorVelocities::MotorVelocities(double fl, double fr, double rl, double rr) : frontLeft(fl), frontRight(fr), rearLeft(rl), rearRight(rr) {

}

double MotorVelocities::getFrontLeft() const {
    return frontLeft;
}

double MotorVelocities::getFrontRight() const {
    return frontRight;
}

double MotorVelocities::getRearRight() const {
    return rearRight;
}

double MotorVelocities::getRearLeft() const {
    return rearLeft;
}


QCAcceleration::QCAcceleration(Rotation3d angular_accel, double accel_x, double accel_y, double accel_z):
    angular_accel(angular_accel),
    accel_x(accel_x),
    accel_y(accel_y),
    accel_z(accel_z){

}

Rotation3d QCAcceleration::getAngular() {
    return angular_accel;
}

double QCAcceleration::getX() {
    return accel_x;
}

double QCAcceleration::getY() {
    return accel_y;
}

double QCAcceleration::getZ() {
    return accel_z;
}

QCState::QCState(Pose3d pose, Pose3d velocity, MotorVelocities motorVelocities, double time): pose(pose), velocity(velocity), motorVelocities(motorVelocities), time(time){

}

Pose3d QCState::getPose() {
    return pose;
}

Pose3d QCState::getVelocity() {
    return velocity;
}

MotorVelocities QCState::getMotorVelocities() {
    return motorVelocities;
}

double QCState::getTime(){
    return time;
}

QCState QCState::predict(double timestep) {
    QCAcceleration accel = velocitiesToAccel(motorVelocities, pose.getRotation());
    double drag_x = -velocity.getX() * LINEAR_DRAG_COEFF_XY;
    double drag_y = -velocity.getY() * LINEAR_DRAG_COEFF_XY;
    double drag_z = -velocity.getZ() * LINEAR_DRAG_COEFF_Z;

    double ang_drag_x = -velocity.getRotation().getRoll() * ANGULAR_DRAG_COEFF_XY;
    double ang_drag_y = -velocity.getRotation().getPitch() * ANGULAR_DRAG_COEFF_XY;
    double ang_drag_z = -velocity.getRotation().getYaw() * ANGULAR_DRAG_COEFF_Z;

    double newVX = (velocity.getX() + ((accel.getX() + drag_x) * timestep));
    double newVY = (velocity.getY() + ((accel.getY() + drag_x) * timestep));
    double newVZ = (velocity.getZ() + ((accel.getZ() + drag_x) * timestep));

    double newAZ = velocity.getRotation().getYaw() + ((accel.getAngular().getYaw() + ang_drag_z) * timestep);
    double newAY = velocity.getRotation().getPitch() + ((accel.getAngular().getPitch() + ang_drag_y) * timestep);
    double newAX = velocity.getRotation().getRoll() + ((accel.getAngular().getRoll() + ang_drag_x) * timestep);

    double newPX = pose.getX() + (newVX * timestep);
    double newPY = pose.getY() + (newVY * timestep);
    double newPZ = pose.getZ() + (newVZ * timestep);
    double newPAZ = pose.getRotation().getYaw() + (newAZ * timestep);
    double newPAY = pose.getRotation().getPitch() + (newAY * timestep);
    double newPAX = pose.getRotation().getRoll() + (newAX * timestep);

    return QCState(
        Pose3d(Translation3d(newPX,newPY,newPZ), Rotation3d(newPAZ,newPAY,newPAX)),
        Pose3d(Translation3d(newVX,newVY,newVZ), Rotation3d(newAZ, newAY, newAX)),
        motorVelocities,
        time+timestep
    );
}