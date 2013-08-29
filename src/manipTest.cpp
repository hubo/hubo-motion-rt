
#include "DrcHuboKin.h"
#include "manip.h"

using namespace RobotKin;





int main(int argc, char **argv)
{
    int side = LEFT;
    std::string limb;

    if(side==RIGHT)
        limb = "RightArm";
    else
        limb = "LeftArm";

    ach_channel_t chan_manip_cmd;
    ach_open(&chan_manip_cmd, CHAN_HUBO_MANIP_CMD, NULL);

    Hubo_Control hubo;
    hubo.update();
    DrcHuboKin kin;
    kin.updateHubo(hubo);

    hubo_manip_cmd_t manip_cmd;

    TRANSFORM pose = kin.linkage(limb).tool().withRespectTo(kin.joint("RAP"));
    TRANSFORM goal = TRANSFORM::Identity();

    std::cout << pose.matrix() << std::endl;
    
    goal.translate( TRANSLATION(0.1,0.1,0.25) );
    goal.translate( pose.translation() );
    goal.rotate(Eigen::AngleAxisd(-M_PI/2, AXIS(0, 1, 0)));
    goal.rotate( pose.rotation() );

    

    manip_cmd.pose[side].x = goal.translation().x();
    manip_cmd.pose[side].y = goal.translation().y();
    manip_cmd.pose[side].z = goal.translation().z();
    Eigen::Quaterniond quat(goal.rotation());
    manip_cmd.pose[side].w = quat.w();
    manip_cmd.pose[side].i = quat.x();
    manip_cmd.pose[side].j = quat.y();
    manip_cmd.pose[side].k = quat.z();

    manip_cmd.m_mode[side] = MC_TELEOP;
    manip_cmd.interrupt[side] = true;

    ach_put(&chan_manip_cmd, &manip_cmd, sizeof(manip_cmd));

}
