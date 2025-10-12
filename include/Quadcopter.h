#include <array>

#ifndef QUADCOPTER_H
#define QUADCOPTER_H
//ALL POSITIVE VALUES, IGNORING DIRECTION
class MotorVelocities {
    private:
        double frontLeft;
        double frontRight;
        double rearLeft;
        double rearRight;

    public:
        MotorVelocities(double frontLeft, double frontRight, double rearLeft, double rearRight);

        double getFrontLeft() const;
        double getFrontRight() const;
        double getRearLeft() const;
        double getRearRight() const;
};

class Rotation3d {
    private:
        double yaw;
        double pitch;
        double roll;

    public:
        Rotation3d(double yaw, double pitch, double roll);

        double getYaw();
        double getPitch();
        double getRoll();
        std::array<double, 3> thrustDirection() const;
        std::array<double, 3> thrustVector(double thrustMagnitude) const;
};

class QCAcceleration {
    private:
        Rotation3d angular_accel;
        double accel_x;
        double accel_y;
        double accel_z;

    public:
        QCAcceleration(Rotation3d angular_accel, double accel_x, double accel_y, double accel_z);

        Rotation3d getAngular();
        double getX();
        double getY();
        double getZ();
};

class QCState {
    private:
        MotorVelocities velocities;
        QCAcceleration acceleration;

    public:
        QCState(MotorVelocities velocities, QCAcceleration acceleration);

        MotorVelocities getVelocities();
        QCAcceleration getAcceleration();
};

#endif