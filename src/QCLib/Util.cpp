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

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

double radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

Vector3d::Vector3d(double x, double y, double z) : x(x), y(y), z(z){

}

Vector3d::Vector3d(): x(0), y(0), z(0) {

}

double Vector3d::getX() const {
    return x;
}

double Vector3d::getY() const {
    return y;
}

double Vector3d::getZ() const {
    return z;
}

double Vector3d::getMagnitude() const {
    return std::sqrt(x*x + y*y + z*z);
}

Vector3d Vector3d::cross(const Vector3d& other) const {
    return Vector3d(
        getY() * other.getZ() - getZ() * other.getY(),
        getZ() * other.getX() - getX() * other.getZ(),
        getX() * other.getY() - getY() * other.getX()
    );
}

Rotation3d Vector3d::getDirection() const {
    double mag = getMagnitude();
    if(mag == 0) return Rotation3d(0,0,0);
    double pitch = std::asin(y/mag); //rotation about Y axis
    double yaw = std::atan2(x, -z); //rotation about Z axis
    return Rotation3d(yaw, pitch, 0); //roll is always 0 for a direction vector
}

Vector3d Vector3d::operator+(const Vector3d& other) const {
    return Vector3d(x + other.x, y + other.y, z + other.z);
}

Vector3d Vector3d::operator-(const Vector3d& other) const {
    return Vector3d(x - other.x, y - other.y, z - other.z);
}

Vector3d Vector3d::operator*(double scalar) const {
    return Vector3d(x * scalar, y * scalar, z * scalar);
}

Vector3d Vector3d::operator/(double scalar) const {
    if(scalar == 0) throw std::runtime_error("Division by zero in Translation3d");
    return Vector3d(x / scalar, y / scalar, z / scalar);
}

Vector3d Vector3d::operator-() const {
    return Vector3d(-x, -y, -z);
}
Vector3d Vector3d::componentWiseMultiply(const Vector3d& other) const {
    return Vector3d(x * other.x, y * other.y, z * other.z);
}
bool Vector3d::operator==(const Vector3d& other) const {
    return (x == other.x) && (y == other.y) && (z == other.z);
}

Vector3d Vector3d::normalized() const {
    double mag = getMagnitude();
    // if(mag == 0) throw std::runtime_error("Cannot normalize zero vector");
    return Vector3d(x/mag, y/mag, z/mag);
}

double Vector3d::dot(const Vector3d& other) const {
    return x * other.x + y * other.y + z * other.z;
}

void Vector3d::print() {
    std::cout << "Translation3d - X: " << x << " Y: " << y << " Z: " << z << "\n";
}


Rotation3d::Rotation3d(double yaw, double pitch, double roll) : yaw(yaw), pitch(pitch), roll(roll){
    hasYPRValues = true;
    hasQuatValues = false;
}

Rotation3d::Rotation3d(double w, double x, double y, double z) : w(w), x(x), y(y), z(z){
    hasYPRValues = false;
    hasQuatValues = true;
}

Rotation3d::Rotation3d(): yaw(0), pitch(0), roll(0) {
    hasYPRValues = true;
    hasQuatValues = false;
}

Rotation3d Rotation3d::fromRotationMatrix(const Vector3d& x_axis, const Vector3d& y_axis, const Vector3d& z_axis) {
    double m00 = x_axis.getX();
    double m01 = y_axis.getX();
    double m02 = z_axis.getX();
    double m10 = x_axis.getY();
    double m11 = y_axis.getY();
    double m12 = z_axis.getY();
    double m20 = x_axis.getZ();
    double m21 = y_axis.getZ();
    double m22 = z_axis.getZ();

    double trace = m00 + m11 + m22;
    double qw, qx, qy, qz;

    if (trace > 0.0) {
        double s = 0.5 / std::sqrt(trace + 1.0);
        qw = 0.25 / s;
        qx = (m21 - m12) * s;
        qy = (m02 - m20) * s;
        qz = (m10 - m01) * s;
    } else {
        if (m00 > m11 && m00 > m22) {
            double s = 2.0 * std::sqrt(1.0 + m00 - m11 - m22);
            qw = (m21 - m12) / s;
            qx = 0.25 * s;
            qy = (m01 + m10) / s;
            qz = (m02 + m20) / s;
        } else if (m11 > m22) {
            double s = 2.0 * std::sqrt(1.0 + m11 - m00 - m22);
            qw = (m02 - m20) / s;
            qx = (m01 + m10) / s;
            qy = 0.25 * s;
            qz = (m12 + m21) / s;
        } else {
            double s = 2.0 * std::sqrt(1.0 + m22 - m00 - m11);
            qw = (m10 - m01) / s;
            qx = (m02 + m20) / s;
            qy = (m12 + m21) / s;
            qz = 0.25 * s;
        }
    }

    return Rotation3d(qw, qx, qy, qz);
}

void Rotation3d::calculateQuatFromYPR() {
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;

    hasQuatValues = true;
}

void Rotation3d::calculateYPRFromQuat() {
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    hasYPRValues = true;
}

double Rotation3d::getYaw() {
    if (!hasYPRValues) calculateYPRFromQuat();
    return yaw;
}

double Rotation3d::getPitch() {
    if (!hasYPRValues) calculateYPRFromQuat();
    return pitch;
}

double Rotation3d::getRoll() {
    if (!hasYPRValues) calculateYPRFromQuat();
    return roll;
}

double Rotation3d::getW() {
    if (!hasQuatValues) calculateQuatFromYPR();
    return w;
}

double Rotation3d::getX() {
    if (!hasQuatValues) calculateQuatFromYPR();
    return x;
}

double Rotation3d::getY() {
    if (!hasQuatValues) calculateQuatFromYPR();
    return y;
}

double Rotation3d::getZ() {
    if (!hasQuatValues) calculateQuatFromYPR();
    return z;
}

Vector3d Rotation3d::getZAxis() const {
    // Body Z axis in world coordinates
    double cy = std::cos(this->yaw);
    double sy = std::sin(this->yaw);
    double cp = std::cos(this->pitch);
    double sp = std::sin(this->pitch);
    double cr = std::cos(this->roll);
    double sr = std::sin(this->roll);

    double x = sy * sr - cy * sp * cr;
    double y = -cy * sr - sy * sp * cr;
    double z = cp * cr;

    return Vector3d(x, y, z).normalized();
}

Vector3d Rotation3d::getXAxis() const {
    // Body X axis in world coordinates
    double cy = std::cos(this->yaw);
    double sy = std::sin(this->yaw);
    double cp = std::cos(this->pitch);
    double sp = std::sin(this->pitch);
    double cr = std::cos(this->roll);
    double sr = std::sin(this->roll);

    double x = cy * cp;
    double y = sy * cp;
    double z = -sp;

    return Vector3d(x, y, z).normalized();
}

Vector3d Rotation3d::getYAxis() const {
    // Body Y axis in world coordinates
    double cy = std::cos(this->yaw);
    double sy = std::sin(this->yaw);
    double cp = std::cos(this->pitch);
    double sp = std::sin(this->pitch);
    double cr = std::cos(this->roll);
    double sr = std::sin(this->roll);

    double x = cy * sp * sr - sy * cr;
    double y = sy * sp * sr + cy * cr;
    double z = cp * sr;

    return Vector3d(x, y, z).normalized();
}

Rotation3d Rotation3d::normalized() {
    if (!hasQuatValues) calculateQuatFromYPR();
    double mag = std::sqrt(w*w + x*x + y*y + z*z);
    if(mag == 0) throw std::runtime_error("Cannot normalize zero quaternion");
    return Rotation3d(w/mag, x/mag, y/mag, z/mag);
}

void Rotation3d::rotateBy(Rotation3d other) {
    if (!hasQuatValues) calculateQuatFromYPR();
    if (!other.hasQuatValues) other.calculateQuatFromYPR();
    double nw = w * other.w - x * other.x - y * other.y - z * other.z;
    double nx = w * other.x + x * other.w + y * other.z - z * other.y;
    double ny = w * other.y - x * other.z + y * other.w + z * other.x;
    double nz = w * other.z + x * other.y - y * other.x + z * other.w;
    w = nw;
    x = nx;
    y = ny;
    z = nz;
    hasYPRValues = false;
    hasQuatValues = true;
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
    double y = getYaw();
    double p = getPitch();
    double r = getRoll();
    std::cout << "Rotation3d - Yaw: " << y << " Pitch: " << p << " Roll: " << r << "\n";
}

void Rotation3d::printDegrees() {
    double y = radiansToDegrees(getYaw());
    double p = radiansToDegrees(getPitch());
    double r = radiansToDegrees(getRoll());
    std::cout << "Rotation3d - Yaw: " << y << " Pitch: " << p << " Roll: " << r << "\n";
}

Rotation3d Rotation3d::fromDegrees(double yaw, double pitch, double roll) {
    return Rotation3d(degreesToRadians(yaw), degreesToRadians(pitch), degreesToRadians(roll));
}

Pose3d::Pose3d(Vector3d translation, Rotation3d rotation): translation(translation), rotation(rotation) {

}

Pose3d::Pose3d(double x, double y, double z, Rotation3d rotation): translation(Vector3d(x,y,z)), rotation(rotation) {

}

Pose3d::Pose3d(double x, double y, double z): translation(Vector3d(x,y,z)), rotation(Rotation3d()){

}

Pose3d::Pose3d(): translation(Vector3d()), rotation(Rotation3d()) {

}

Vector3d Pose3d::getTranslation() {
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