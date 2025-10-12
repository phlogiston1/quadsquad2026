#include <iostream>
#include "Quadcopter.h"
#include "Kinematics.h"

int main(int, char**){
    MotorVelocities testV = MotorVelocities(1,1,1,1);
    QCAcceleration test = velocitiesToAccel(testV, Rotation3d(0,0,0));
    std::cout << "X: " << test.getX() << " Y: " << test.getY() << " Z: " << test.getZ();
    std::cout << "\nYaw: " << test.getAngular().getYaw() << " Pitch: " << test.getAngular().getPitch() << " Roll: " << test.getAngular().getRoll();
}
