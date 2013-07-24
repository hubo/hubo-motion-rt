#ifndef DRCHUBOKIN_H
#define DRCHUBOKIN_H


#include <RobotKin/Robot.h>
#include <Hubo_Control.h>

class DrcHuboKin : public RobotKin::Robot
{
public:

    friend class Hubo_Control;

    DrcHuboKin();
    DrcHuboKin(std::string filename);

    RobotKin::TRANSFORM armFK(int side);
    RobotKin::TRANSFORM legFK(int side);

    RobotKin::rk_result_t legIK(int side, LegVector &q, const Eigen::Isometry3d target, const LegVector &qPrev);
    RobotKin::rk_result_t legIK(int side, LegVector &q, const Eigen::Isometry3d target);

    RobotKin::rk_result_t armIK(int side, ArmVector &q, const RobotKin::TRANSFORM target, const ArmVector &qPrev);
    RobotKin::rk_result_t armIK(int side, ArmVector &q, const RobotKin::TRANSFORM target);

    RobotKin::rk_result_t armTorques(int side, ArmVector &jointTorque, const Vector6d &eeWrench=Vector6d::Zero());
    RobotKin::rk_result_t armTorques(int side, ArmVector &jointTorque, const Vector6d &eeWrench, const ArmVector &jointAngles);

    void updateArmJoints(int side, const ArmVector& jointValues);
    void updateLegJoints(int side, const LegVector& jointValues);

    void updateHubo(Hubo_Control& hubo);

    ArmVector armRestValues[2];
    LegVector legRestValues[2];

};



// Foot translation: 0, 0, -0.14
// Hand translation: 0, 0, -0.11952






#endif // DRCHUBOKIN_H
