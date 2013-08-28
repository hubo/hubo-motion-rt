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

    void commenceSlerping(int side, hubo_manip_cmd_t &cmd, Hubo_Control &hubo, double dt);


protected:

    DrcHuboKin kin;
    
    RobotKin::TRANSLATION dr[2];
    RobotKin::TRANSLATION V[2];
    RobotKin::TRANSLATION dV[2];
    
    RobotKin::TRANSFORM start;
    RobotKin::TRANSFORM goal;
    RobotKin::TRANSFORM next;
    RobotKin::TRANSFORM altStart;
    RobotKin::TRANSFORM altNext;
    
    double nomSpeed;
    double nomAcc;
    double adr;
    
    Eigen::AngleAxisd angax;
    double angle[2];
    double W[2];
    double dW[2];
    
    double nomRotSpeed;
    double nomRotAcc;
    double ada;
    
    std::string limb[2];
    
    ArmVector armAngles[2];
    ArmVector lastAngles[2];

#ifdef HAVE_REFLEX
    aa_mem_region_t reg;

#endif //HAVE_REFLEX

};





















#endif // SLERPER_H
