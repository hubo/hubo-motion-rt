
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
    
    std::cout << pose.matrix() << std::endl;
    pose.pretranslate(TRANSLATION(0.0,0.0,0.2));
//    pose.rotate(Eigen::AngleAxisd(-M_PI/2, AXIS(1, 0, 0)));

    

    manip_cmd.pose[side].x = pose.translation().x();
    manip_cmd.pose[side].y = pose.translation().y();
    manip_cmd.pose[side].z = pose.translation().z();
    Eigen::Quaterniond quat(pose.rotation());
    manip_cmd.pose[side].w = quat.w();
    manip_cmd.pose[side].i = quat.x();
    manip_cmd.pose[side].j = quat.y();
    manip_cmd.pose[side].k = quat.z();

    manip_cmd.m_mode[side] = MC_TELEOP;
    manip_cmd.interrupt[side] = true;

    ach_put(&chan_manip_cmd, &manip_cmd, sizeof(manip_cmd));

}
