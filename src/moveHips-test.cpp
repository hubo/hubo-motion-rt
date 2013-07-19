
/**
 * \brief Test program to test the leg FK and IK as well
 * as hipVelocityIK
*/

#include "Hubo_Control.h"
#include <Eigen/StdVector>
#include <iostream>

int main(int argc, char **argv)
{
    Hubo_Control hubo;

    std::vector<LegVector, Eigen::aligned_allocator<LegVector> > legJointVels(2);
    std::vector<LegVector, Eigen::aligned_allocator<LegVector> > legJointAngs(2);
    std::vector<LegVector, Eigen::aligned_allocator<LegVector> > legJointAngsNext(2);
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > footTF(2);


    for(int side=0; side<2; side++)
    {
        hubo.getLegAngles(side, legJointAngs[side]);
        hubo.huboLegFK(footTF[side], legJointAngs[side], side);
        if(side = LEFT)
            std::cout << "LEFT:";
        else
            std::cout << "RIGHT:";
        std::cout << "\n\tAngles = " << q.transpose()
        std::cout << "\n\tFoot TF = \n" << footTF[side].matrix() << "\n";
        footTF[side](2,3) += 0.2;
        hubo.huboLegIK(legJointAngsNext[side], footTF[side], legJointAngs[side], side);
        hubo.setLegAngles(side, legJointAngsNext[side]);
    }

    std::cout << "Left Angles = " << q.transpose()
              << "\nFoot TF = \n" << TF.matrix() << "\n";
//    hubo.sendControls();

    char c;
    std::cout << "Press any key to move the hips\n";
    c = getchar();

    Eigen::Vector3d hipVelocity;

    hipVelocity << 0.1, 0.1, 0.0;

    //---------------------------
    //   LEG JOINT VELOCITIES
    //---------------------------
    for(int side=0; side<2; side++)
    {
        hubo.hipVelocityIK(legJointVels[side], hipVelocity, side);
        hubo.setLegVels(side, legJointVels[side]);
    }

//    hubo.sendControls();

    std::cout << "Press any key to stop the hips\n";
    c = getchar();

    for(int side=0; side<2; side++)
    {
        legJointVels[side].setZero();
        hubo.setLegVels(side, legJointVels[side]);
    }

//    hubo.sendControls();

    return 0;
}
