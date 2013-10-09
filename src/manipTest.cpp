
#include "DrcHuboKin.h"
#include "manip.h"

using namespace RobotKin;
using namespace std;




int main(int argc, char **argv)
{
//    int side = RIGHT;
//    std::string limb;
//    double dy = 0.1;
//    if(side == LEFT)
//        dy = fabs(dy);
//    else if(side == RIGHT)
//        dy = -fabs(dy);
    

//    if(side==RIGHT)
//        limb = "RightArm";
//    else
//        limb = "LeftArm";

//    ach_channel_t chan_manip_cmd;
//    ach_open(&chan_manip_cmd, CHAN_HUBO_MANIP_CMD, NULL);

//    Hubo_Control hubo;
//    hubo.update();
//    DrcHuboKin kin;
//    kin.updateHubo(hubo);

//    hubo_manip_cmd_t manip_cmd;

//    TRANSFORM pose = kin.linkage(limb).tool().withRespectTo(kin.joint("RAP"));
//    TRANSFORM goal = TRANSFORM::Identity();

//    std::cout << pose.matrix() << std::endl;
    
//    goal.translate( TRANSLATION(0.1,dy,0.25) );
//    goal.translate( pose.translation() );
//    goal.rotate(Eigen::AngleAxisd(-M_PI/2, AXIS(0, 1, 0)));
//    goal.rotate( pose.rotation() );

    

//    manip_cmd.pose[side].x = goal.translation().x();
//    manip_cmd.pose[side].y = goal.translation().y();
//    manip_cmd.pose[side].z = goal.translation().z();
//    Eigen::Quaterniond quat(goal.rotation());
//    manip_cmd.pose[side].w = quat.w();
//    manip_cmd.pose[side].i = quat.x();
//    manip_cmd.pose[side].j = quat.y();
//    manip_cmd.pose[side].k = quat.z();

//    manip_cmd.m_mode[side] = MC_TELEOP;
//    manip_cmd.interrupt[side] = true;

//    ach_put(&chan_manip_cmd, &manip_cmd, sizeof(manip_cmd));

    DrcHuboKin kin;
    
//    kin.linkage("RightLeg").printInfo();
    LegVector q, targetVals; q.setZero(); targetVals.setZero();

    kin.updateLegJoints(LEFT, q);

    TRANSFORM B = kin.legFK(LEFT);

//    cout << "Before: " << q.transpose() << endl;
//    kin.legIK(LEFT, q, B);
//    cout << "After:  " << q.transpose() << endl;
//    q.setZero();

//    q(KN) = 0.4;
//    kin.updateLegJoints(LEFT, q);
//    B = kin.legFK(LEFT);
//    cout << "Before: " << q.transpose() << endl;
//    kin.legIK(LEFT, q, B);
//    cout << "After:  " << q.transpose() << endl;

    string limb = "LeftLeg";

    cout << kin.joint("RHR").min() << "\t" << kin.joint("RHR").max();
    srand(time(NULL));

    int tests = 1000;
    int resolution = 1000;
    int good = 0;

    for(int k=0; k<tests; k++)
    {
        for(int i=0; i<6; i++)
        {
            int randomVal = rand();
            targetVals(i) = ((double)(randomVal%resolution))/((double)resolution-1)
                    *(kin.linkage(limb).joint(i).max() - kin.linkage(limb).joint(i).min())
                    + kin.linkage(limb).joint(i).min();

            randomVal = rand();

            q(i) = ((double)(randomVal%resolution))/((double)resolution-1)
                                *(kin.linkage(limb).joint(i).max() - kin.linkage(limb).joint(i).min())
                                + kin.linkage(limb).joint(i).min();
        }

        kin.updateLegJoints(LEFT, targetVals);
        B = kin.legFK(LEFT);

        kin.updateLegJoints(LEFT, q);
        kin.legIK(LEFT, q, B);
        kin.updateLegJoints(LEFT, q);

        if( (targetVals-q).norm() > 1e-12 )
        {
            cout << "Target: " << B.matrix() << endl << endl;
            cout << "Actual: " << kin.legFK(LEFT).matrix() << endl << endl;
        }
        else
            good++;
    }

    cout << "Good: " << good << endl;



//    cout << "Trunk Left:" << endl << kin.linkage("LeftLeg").joint(1).respectToRobot().matrix() << endl << endl;

//    cout << "Trunk Right:" << endl << kin.linkage("RightLeg").joint(1).respectToRobot().matrix() << endl;
}
