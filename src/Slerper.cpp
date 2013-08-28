

#include "Slerper.h"

using namespace RobotKin;

double sign(double x)
{
    return (x < 0) ? -1 : (x > 0);
}

Slerper::Slerper() :
    kin()
{
#ifdef HAVE_REFLEX
    aa_mem_region_init(&reg, 1024*32);
#endif //HAVE_REFLEX

    
    nomSpeed = 0.15;
    nomAcc   = 0.5;
    
    nomRotSpeed = 0.5;
    nomRotAcc   = 0.5;
    
    for(int i=0; i<2; i++)
    {
        dr[i] = TRANSLATION::Zero();
        V[i] = TRANSLATION::Zero();
        dV[i] = TRANSLATION::Zero();
        
        angle[i] = 0;
        W[i] = 0;
        dW[i] = 0;
    }
    
    
    limb[RIGHT] = "RightArm";
    limb[LEFT]  = "LeftArm";

}

#ifdef HAVE_REFLEX
void Slerper::commenceSlerping(int side, hubo_manip_cmd_t &cmd)
{
    std::cerr << "We do not currently have reflex/amino support" << std::endl;
    
    
    hubo.update();
    kin.updateHubo(hubo);

    aa_mem_region_release(&reg);

    rfx_trajx_parablend_t T;
    rfx_trajx_splend_init(&T, &reg, 1);

    rfx_trajx_t *pT = (rfx_trajx_t*)&T;

    // NOTE: Amino quaternions are x, y, z, w

    double X[3];

    double R[4];


}

#else //HAVE_REFLEX
void Slerper::commenceSlerping(int side, hubo_manip_cmd_t &cmd, Hubo_Control &hubo, double dt)
{
    kin.updateHubo(hubo); // Probably not necessary
    
    bool dual;
    if(cmd.m_mode[side] == MC_DUAL_TELEOP)
        dual = true;
    else
        dual = false;
    
    
    int alt;
    if(side==LEFT)
        alt = RIGHT;
    else
        alt = LEFT;
    
    
    start = kin.linkage(limb[side]).tool().withRespectTo(kin.joint("RAP"));
    next = start;
    
    if(dual)
    {
        altStart = kin.linkage(limb[alt]).tool().withRespectTo(kin.joint("RAP"));
        altNext = altStart;
    }
    
    goal = TRANSFORM::Identity();
    goal.translate(TRANSLATION(cmd.pose[side].x, 
                               cmd.pose[side].y,
                               cmd.pose[side].z));
    goal.rotate(Eigen::Quaterniond(cmd.pose[side].w,
                            cmd.pose[side].i,
                            cmd.pose[side].j,
                            cmd.pose[side].k));
    
    dr[side] = goal.translation() - start.translation();
    
    dV[side] = dr[side].normalized()*fabs(nomSpeed) - V[side];
    
    adr = sqrt(fabs(2.0*nomAcc*dr[side].norm()));
    if( V[side].norm() >= adr )
        dV[side] = dr[side].normalized()*adr - V[side];
    else
        clampMag(dV[side], fabs(nomAcc*dt));
    
    V[side] += dV[side];
    
    
    if( dr[side].norm() > V[side].norm()*dt || dr[side].dot(V[side]) < 0 )
        dr[side] = V[side]*dt;
    
    V[side] = dr[side]/dt;
    
    next.translate(dr[side]);
    if(dual)
        altNext.translate(dr[side]);
    
    
    angax = goal.rotation()*start.rotation().transpose();
    
    angle[side] = angax.angle();
    
    if( angle[side] > M_PI )
        angle[side] = angle[side]-2*M_PI;
    
    dW[side] = sign(angle[side])*nomRotSpeed - W[side];
    
    ada = sqrt(fabs(2.0*nomRotAcc*angle[side]));
    if( fabs(W[side]) >= ada )
        dW[side] = sign(angle[side])*ada - W[side];
    else if( dW[side] > fabs(nomRotAcc*dt) )
        dW[side] = fabs(nomRotAcc*dt);
    else if( dW[side] < -fabs(nomRotAcc*dt) )
        dW[side] = -fabs(nomRotAcc*dt);
    
    W[side] += dW[side];
    
    if( fabs(angle[side]) > fabs(W[side]*dt) || angle[side]*W[side] < 0 )
        angle[side] = W[side]*dt;
    
    W[side] = angle[side]/dt;
    
    next.rotate(Eigen::AngleAxisd(angle[side], angax.axis()));
    
    if(dual)
        altNext.rotate(Eigen::AngleAxisd(angle[side], angax.axis()));
    
    next = kin.joint("RAP").respectToRobot()*next;
    if(dual)
        altNext = kin.joint("RAP").respectToRobot()*altNext;
    
    hubo.getArmAngles(side, armAngles[side]);
    if(dual)
        hubo.getArmAngles(alt, armAngles[alt]);
    
    lastAngles[side] = armAngles[side];
    if(dual)
        lastAngles[alt] = armAngles[alt];
    
    kin.armIK(side, armAngles[side], next);
    if(dual)
        kin.armIK(alt, armAngles[alt], altNext);
    
    hubo.setArmTraj(side, armAngles[side], (armAngles[side]-lastAngles[side])/dt);
    if(dual)
        hubo.setArmTraj(alt, armAngles[alt], (armAngles[alt]-lastAngles[alt])/dt);
    
    
}
#endif //HAVE_REFLEX




