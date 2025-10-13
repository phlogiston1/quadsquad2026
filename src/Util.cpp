#include "Util.h"
#include <cmath>
#include <array>
#include <vector>
#include <iostream>
#include <stdexcept> // For std::runtime_error

// Function to perform linear interpolation on arrays
double interpolateLinear(const std::vector<double>& x_data,
                         const std::vector<double>& y_data,
                         double x_interp) {
    // Basic error checking
    if (x_data.size() != y_data.size() || x_data.empty()) {
        throw std::runtime_error("Input arrays must have the same non-zero size.");
    }

    // Handle edge cases where x_interp is outside the data range
    if (x_interp <= x_data.front()) {
        return y_data.front();
    }
    if (x_interp >= x_data.back()) {
        return y_data.back();
    }

    // Find the interval where x_interp lies
    for (size_t i = 0; i < x_data.size() - 1; ++i) {
        if (x_interp >= x_data[i] && x_interp <= x_data[i+1]) {
            // Linear interpolation formula: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
            double x1 = x_data[i];
            double y1 = y_data[i];
            double x2 = x_data[i+1];
            double y2 = y_data[i+1];

            return y1 + (x_interp - x1) * (y2 - y1) / (x2 - x1);
        }
    }

    // This part should ideally not be reached if edge cases and loop logic are correct
    throw std::runtime_error("Could not find interpolation interval.");
}

Translation3d::Translation3d(double x, double y, double z) : x(x), y(y), z(z){

}

Translation3d::Translation3d(): x(0), y(0), z(0) {

}

double Translation3d::getX() {
    return x;
}

double Translation3d::getY() {
    return y;
}

double Translation3d::getZ() {
    return z;
}

void Translation3d::print() {
    std::cout << "Translation3d - X: " << x << " Y: " << y << " Z: " << z << "\n";
}


Rotation3d::Rotation3d(double yaw, double pitch, double roll) : yaw(yaw), pitch(pitch), roll(roll){

}

Rotation3d::Rotation3d(): yaw(0), pitch(0), roll(0) {

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

void Rotation3d::print() {
    std::cout << "Rotation3d - Yaw: " << yaw << " Pitch: " << pitch << " Roll: " << roll;
}

Pose3d::Pose3d(Translation3d translation, Rotation3d rotation): translation(translation), rotation(rotation) {

}

Pose3d::Pose3d(double x, double y, double z, Rotation3d rotation): translation(Translation3d(x,y,z)), rotation(rotation) {

}

Pose3d::Pose3d(double x, double y, double z): translation(Translation3d(x,y,z)), rotation(Rotation3d()){

}

Pose3d::Pose3d(): translation(Translation3d()), rotation(Rotation3d()) {

}

Translation3d Pose3d::getTranslation() {
    return translation;
}

Rotation3d Pose3d::getRotation() {
    return rotation;
}

double Pose3d::getX() {
    return translation.getX();
}

double Pose3d::getY() {
    return translation.getY();
}

double Pose3d::getZ() {
    return translation.getZ();
}

void Pose3d::print() {
    std::cout << "Pose3d:\n\t";
    translation.print();
    std::cout << "\t";
    rotation.print();
    std::cout << "\n";
}