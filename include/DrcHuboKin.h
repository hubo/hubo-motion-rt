#ifndef DRCHUBOKIN_H
#define DRCHUBOKIN_H


#include <RobotKin/Robot.h>
#include <Hubo_Control.h>

class DrcHuboKin : public RobotKin::Robot
{
public:

    DrcHuboKin();
    DrcHuboKin(std::string filename);

    RobotKin::TRANSFORM handFK(int side);
    RobotKin::TRANSFORM footFK(int side);

    RobotKin::rk_result_t legIK(int side, LegVector &q, const Eigen::Isometry3d B, const LegVector &qPrev);

    RobotKin::rk_result_t armIK(int side, ArmVector &q, const Eigen::Isometry3d B, const ArmVector &qPrev);
    RobotKin::rk_result_t armIK(int side, ArmVector &q, const Eigen::Isometry3d B);

    RobotKin::rk_result_t armTorques(int side, ArmVector &jointTorque, const Vector6d &eeWrench=Vector6d::Zero());
    RobotKin::rk_result_t armTorques(int side, ArmVector &jointTorque, const Vector6d &eeWrench, const ArmVector &jointAngles);

};



// Foot translation: 0, 0, -0.14
// Hand translation: 0, 0, -0.11952






#endif // DRCHUBOKIN_H
