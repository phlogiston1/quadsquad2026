#include "Quadcopter.h"
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

Rotation3d::Rotation3d(double yaw, double pitch, double roll) : yaw(yaw), pitch(pitch), roll(roll){

}

double Rotation3d::getYaw() {
    return yaw;
}

double Rotation3d::getPitch() {
    return pitch;
}

double Rotation3d::getRoll() {
    return roll;
}

// Returns a normalized direction vector (x, y, z)
// representing the body -Z axis in world coordinates
// (assuming thrust acts along body -Z)
std::array<double, 3> Rotation3d::thrustDirection() const {
    double cy = std::cos(this->yaw);
    double sy = std::sin(this->yaw);
    double cp = std::cos(this->pitch);
    double sp = std::sin(this->pitch);
    double cr = std::cos(this->roll);
    double sr = std::sin(this->roll);

    // Combined rotation matrix (ZYX: yaw → pitch → roll)
    // We only need the 3rd column (direction of body -Z axis)
    // Because the thrust points opposite body Z.
    double x = -(cy * sp * cr + sy * sr);
    double y = -(sy * sp * cr - cy * sr);
    double z = -(cp * cr);

    // Normalize to ensure it’s a unit vector
    double mag = std::sqrt(x*x + y*y + z*z);
    return { x / mag, y / mag, z / mag };
}

// Returns the full thrust vector = unit direction × thrust magnitude
std::array<double, 3> Rotation3d::thrustVector(double thrustMagnitude) const {
    auto dir = thrustDirection();
    return { dir[0] * thrustMagnitude, dir[1] * thrustMagnitude, dir[2] * thrustMagnitude };
}