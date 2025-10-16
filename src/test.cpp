#include "Quadcopter.h"
#include "InverseKinematics.h"

#include <iostream>

int main() {
    QCState currentState = QCState(
        Pose3d(Vector3d(0,0,0), Rotation3d::fromDegrees(0,0,0)),
        Pose3d(Vector3d(0,0,0), Rotation3d::fromDegrees(0,0,0)),
        MotorVelocities(2000,2000,2000,2000),
        0
    );

    Vector3d targetAccel(1,0,0);
    double targetYawRate = 0;

    TargetQCState targetState = calculateTargetState(currentState, targetAccel, targetYawRate);

    targetState.targetAngle.print();
}