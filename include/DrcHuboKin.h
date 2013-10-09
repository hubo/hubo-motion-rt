#ifndef DRCHUBOKIN_H
#define DRCHUBOKIN_H


#include <RobotKin/Robot.h>
#include <Hubo_Control.h>

typedef Eigen::Matrix< double, 6, 7 > ArmJacobian;



class DrcConstraints : public RobotKin::Constraints
{
public:
    void iterativeJacobianSeed(RobotKin::Robot &robot, size_t attemptNumber, const std::vector<size_t> &indices, Eigen::VectorXd &values);
};

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

    RobotKin::rk_result_t armIK(int side, ArmVector &q, const RobotKin::TRANSFORM target, const ArmVector &qNull);
    RobotKin::rk_result_t armIK(int side, ArmVector &q, const RobotKin::TRANSFORM target);

    ArmJacobian armJacobian(int side);
    ArmJacobian armJacobian(int side, ArmVector &q);

    RobotKin::rk_result_t armTorques(int side, ArmVector &jointTorque, const Vector6d &eeWrench=Vector6d::Zero());
    RobotKin::rk_result_t armTorques(int side, ArmVector &jointTorque, const Vector6d &eeWrench, const ArmVector &jointAngles);

    void updateArmJoints(int side, const ArmVector& jointValues);
    void updateLegJoints(int side, const LegVector& jointValues);

    void updateHubo(Hubo_Control& hubo, bool state=true);

    Eigen::VectorXd armRestValues;


    DrcConstraints armConstraints;
//    RobotKin::Constraints armConstraints;

    RobotKin::TRANSFORM toolTf[2];
    RobotKin::TRANSFORM toolTfR;
    RobotKin::TRANSFORM toolTfL;

    void resetTool(int side);
    void setTool(int side, const RobotKin::TRANSFORM offset);
    RobotKin::TRANSFORM getTool(int side);

    
protected:
    
    Eigen::VectorXd jointVals;
    Eigen::VectorXd restVals;
};




// Foot translation: 0, 0, -0.14
// Hand translation: 0, 0, -0.11952






#endif // DRCHUBOKIN_H
