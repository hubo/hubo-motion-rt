#ifndef DRCHUBOKIN_H
#define DRCHUBOKIN_H


#include <Robot.h>
#include <Hubo_Control.h>


class DrcHuboKin : public RobotKin::Robot
{
public:

    DrcHuboKin();
    DrcHuboKin(string filename);

    RobotKin::rk_result_t legIK(int side, LegVector &q, const Eigen::Isometry3d B, LegVector qPrev);

protected:
    double legLinks[6];

};










#endif // DRCHUBOKIN_H
