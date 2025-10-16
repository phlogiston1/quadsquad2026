#include "Quadcopter.h"
class MotionController {
    public:
        virtual ~MotionController() = default;
        virtual MotorVelocities calculateMotorVelocities(const QCState& currentState, const Translation3d& targetVelocity, double targetYaw) = 0;
};

class ManualController : public MotionController {
    public:
        ManualController(MotorVelocities initialVels);
        MotorVelocities calculateMotorVelocities(const QCState& currentState, const Translation3d& targetVelocity, double targetYaw ) override;
        void setMotorVelocities(MotorVelocities newVels);
    private:
        MotorVelocities motorVels;
};

class PIDController : public MotionController {
    public:
        PIDController(Pose3d kp, Pose3d ki, Pose3d kd, MotorVelocities hoverVels);
        MotorVelocities calculateMotorVelocities(const QCState& currentState, const Translation3d& targetVelocity, double targetYaw) override;
    private:
        Pose3d kp;
        Pose3d ki;
        Pose3d kd;
        MotorVelocities hoverVels;

        Pose3d integralError;
        Pose3d lastError;
};

class QuadSquadController : public MotionController {
    public:
        QuadSquadController(MotorVelocities hoverVels);
        MotorVelocities calculateMotorVelocities(const QCState& currentState, const Translation3d& targetVelocity, double targetYaw) override;
};