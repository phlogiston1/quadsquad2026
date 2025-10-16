#include <array>

#ifndef UTIL_H
#define UTIL_H

class Rotation3d;
class Vector3d {
    private:
        double x;
        double y;
        double z;

    public:
        Vector3d(double x, double y, double z);
        Vector3d();

        double getX() const;
        double getY() const;
        double getZ() const;
        double getMagnitude() const;
        Vector3d cross(const Vector3d& other) const;
        Rotation3d getDirection() const;
        Vector3d operator+(const Vector3d& other) const;
        Vector3d operator-(const Vector3d& other) const;
        Vector3d operator*(double scalar) const;
        Vector3d operator/(double scalar) const;
        Vector3d operator-() const;
        Vector3d normalized() const;
        bool operator==(const Vector3d& other) const;
        void print();
};

class Rotation3d {
    private:
        double w, x, y, z;
        double yaw, pitch, roll;
        bool hasYPRValues;
        bool hasQuatValues;

    public:
        Rotation3d(double yaw, double pitch, double roll);
        Rotation3d(double w, double x, double y, double z);
        Rotation3d();

        double getYaw();
        double getPitch();
        double getRoll();
        double getW();
        double getX();
        double getY();
        double getZ();
        Rotation3d normalized();
        void rotateBy(Rotation3d other);
        std::array<double, 3> thrustDirection() const;
        std::array<double, 3> thrustVector(double thrustMagnitude) const;
        void print();
        void printDegrees();
        void calculateYPRFromQuat();
        void calculateQuatFromYPR();
        static Rotation3d fromDegrees(double yaw, double pitch, double roll);
        static Rotation3d fromRotationMatrix(const Vector3d& x_axis, const Vector3d& y_axis, const Vector3d& z_axis);
};

class Pose3d {
    private:
        Vector3d translation;
        Rotation3d rotation;

    public:
        Pose3d (Vector3d translation, Rotation3d rotation);
        Pose3d(double x, double y, double z, Rotation3d rotation);
        Pose3d(double x, double y, double z);
        Pose3d();
        Vector3d getTranslation();
        Rotation3d getRotation();
        double getX();
        double getY();
        double getZ();
        void print();
};

#endif