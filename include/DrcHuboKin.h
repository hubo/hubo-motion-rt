#ifndef DRCHUBOKIN_H
#define DRCHUBOKIN_H


#include <Robot.h>
#include <Hubo_Control.h>


class DrcHuboKin : public RobotKin::Robot
{
public:

    DrcHuboKin();
    DrcHuboKin(string filename);

    RobotKin::rk_result_t legIK(int side, LegVector &q, const Eigen::Isometry3d B, const LegVector &qPrev);

    RobotKin::rk_result_t armIK(int side, ArmVector &q, const Eigen::Isometry3d B, const ArmVector &qPrev);
    RobotKin::rk_result_t armIK(int side, ArmVector &q, const Eigen::Isometry3d B);

    RobotKin::rk_result_t armTorques(int side, ArmVector &jointTorque, const Vector6d &eeTorque);
    RobotKin::rk_result_t armTorques(int side, ArmVector &jointTorque, const Vector6d &eeTorque, const ArmVector &jointAngles);

};










#endif // DRCHUBOKIN_H
