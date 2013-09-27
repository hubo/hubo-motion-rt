#ifndef SLERPER_H
#define SLERPER_H

//#define HAVE_REFLEX // Uncomment this to get autocompletion
#ifdef HAVE_REFLEX
#include <amino.h>
#include <reflex.h>
#endif //HAVE_REFLEX


#include "DrcHuboKin.h"
#include "manip.h"



class Slerper
{
public:
    Slerper();

    void resetSlerper(int side, Hubo_Control &hubo);
    void commenceSlerping(int side, hubo_manip_cmd_t &cmd, Hubo_Control &hubo, double dt);


protected:

    DrcHuboKin kin;
    
    RobotKin::TRANSLATION dr[2];
    RobotKin::TRANSLATION V[2];
    RobotKin::TRANSLATION dV[2];

    RobotKin::TRANSLATION com;
    
    RobotKin::TRANSFORM start;
    RobotKin::TRANSFORM goal;
    RobotKin::TRANSFORM next[2];
    RobotKin::TRANSFORM altStart;
    RobotKin::TRANSFORM altNext;
    RobotKin::TRANSFORM eepose;
    RobotKin::TRANSFORM alteepose;

    ArmVector qdotC;
    Eigen::Matrix< double, 7, 1 > qdot;
    Eigen::Matrix< double, 6, 1 > mscrew;
    ArmJacobian J;
    
    double nomSpeed;
    double nomAcc;
    double stopSpeed;
    double maxVel;
    RobotKin::TRANSLATION accel;
    
    Eigen::AngleAxisd angax;
    double angle[2];
    double W[2];
    double dW[2];
    
    double nomRotSpeed;
    double nomRotAcc;
    double stopRotSpeed;
    double maxRotVel;
    double rotAccel;
    double ada;

    double worstOffense;
    int worstOffender;
    
    std::string limb[2];
    
    ArmVector armAngles[2];
    ArmVector lastAngles[2];

    FILE * dump;

#ifdef HAVE_REFLEX
    aa_mem_region_t reg;

#endif //HAVE_REFLEX

};





















#endif // SLERPER_H
