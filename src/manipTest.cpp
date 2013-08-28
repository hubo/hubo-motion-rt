
#include "DrcHuboKin.h"
#include "manip.h"

using namespace RobotKin;





int main(int argc, char **argv)
{
    ach_channel_t chan_manip_cmd;
    ach_open(&chan_manip_cmd, CHAN_HUBO_MANIP_CMD, NULL);

    Hubo_Control hubo;
    DrcHuboKin kin;
    kin.updateHubo(hubo);

    hubo_manip_cmd_t manip_cmd;

    TRANSFORM pose = kin.linkage("RightArm").tool().withRespectTo(kin.joint("RAP"));
    pose.translate(TRANSLATION(0,0,0.1));

    manip_cmd.pose[RIGHT].x = pose.translation().x();
    manip_cmd.pose[RIGHT].y = pose.translation().y();
    manip_cmd.pose[RIGHT].z = pose.translation().z();
    Eigen::Quaterniond quat(pose.rotation());
    manip_cmd.pose[RIGHT].w = quat.w();
    manip_cmd.pose[RIGHT].i = quat.x();
    manip_cmd.pose[RIGHT].j = quat.y();
    manip_cmd.pose[RIGHT].k = quat.z();

    manip_cmd.m_mode[RIGHT] = MC_TELEOP;

    ach_put(&chan_manip_cmd, &manip_cmd, sizeof(manip_cmd));

}
