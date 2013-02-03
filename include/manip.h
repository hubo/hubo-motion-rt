#include <Hubo_Tech.h>


#define CHAN_HUBO_MANIP "manip-cmd"


struct hubo_manip_cmd {
    double translation[2][3];   // Use translation[RIGHT][0] to specify x for the right-side end effector
                                // translation[LEFT][0] -> left arm's x
                                // translation[LEFT][1] -> left arm's y
                                // translation[LEFT][2] -> left arm's z
    double eulerAngles[2][3];   // Use eulerAngles[LEFT][0] to specify x-axis rotation for left-side end effector
                                // eulerAngles[RIGHT][0] -> right arm's roll
                                // eulerAngles[RIGHT][1] -> right arm's pitch
                                // eulerAngles[RIGHT][2] -> right arm's yaw

    // Euler Angles are applied in the following order: X1, Y2, Z3
};

