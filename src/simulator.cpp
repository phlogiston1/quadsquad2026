#include <iostream>
#include "Quadcopter.h"
#include "Util.h"
#include "Kinematics.h"

int main(int, char**){
    MotorVelocities testV = MotorVelocities(3,3,2,2);
    QCAcceleration test = velocitiesToAccel(testV, Rotation3d(0,0,0));
    QCState testSt = QCState(
        Pose3d(Translation3d(0,0,0), Rotation3d(0,0,0)),
        Pose3d(Translation3d(0,0,0), Rotation3d(0,0,0)), testV, 0);

    QCState next = testSt;
    for(int i = 0; i < 1000; i++) {
        next = next.predict(0.01);
        next.getPose().print();
    }
}
