#include <array>

#ifndef UTIL_H
#define UTIL_H

class Translation3d {
    private:
        double x;
        double y;
        double z;

    public:
        Translation3d(double x, double y, double z);
        Translation3d();

        double getX();
        double getY();
        double getZ();
        void print();
};

class Rotation3d {
    private:
        double yaw;
        double pitch;
        double roll;

    public:
        Rotation3d(double yaw, double pitch, double roll);
        Rotation3d();

        double getYaw();
        double getPitch();
        double getRoll();
        std::array<double, 3> thrustDirection() const;
        std::array<double, 3> thrustVector(double thrustMagnitude) const;
        void print();
};

class Pose3d {
    private:
        Translation3d translation;
        Rotation3d rotation;

    public:
        Pose3d (Translation3d translation, Rotation3d rotation);
        Pose3d(double x, double y, double z, Rotation3d rotation);
        Pose3d(double x, double y, double z);
        Pose3d();
        Translation3d getTranslation();
        Rotation3d getRotation();
        double getX();
        double getY();
        double getZ();
        void print();
};

#endif