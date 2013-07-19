
/**
 * \brief Test program to test the leg FK and IK as well
 * as hipVelocityIK
*/

//#include "DrcHuboKin.h"
#include "Hubo_Control.h"
#include <iostream>

int main(int argc, char **argv)
{
    Hubo_Control hubo(false);
    //DrcHuboKin kin;
//    hubo.update(false);

    std::vector<LegVector, Eigen::aligned_allocator<LegVector> > legJointVels(2);
    std::vector<LegVector, Eigen::aligned_allocator<LegVector> > legJointAngs(2);
    std::vector<LegVector, Eigen::aligned_allocator<LegVector> > legJointAngsNext(2);
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > footTF(2);

    for(int side=0; side<2; side++)
    {
        hubo.getLegAngles(side, legJointAngs[side]);
        legJointAngs[side] << 0,0,0,0,0, 0,0,0,0,0;
        //footTF[side] = kin.linkage("LeftLeg").tool().respectToWorld();
        hubo.huboLegFK(footTF[side], legJointAngs[side], side);
        if(LEFT == side)
            std::cout << "LEFT:";
        else if(RIGHT == side)
            std::cout << "RIGHT:";
        std::cout << "\nPrevious Angles = " << legJointAngs[side].transpose();
        std::cout << "\nFoot TF = \n" << footTF[side].matrix() << "\n\n";
        footTF[side](2,3) += 0.2;
        hubo.huboLegIK(legJointAngsNext[side], footTF[side], legJointAngs[side], side);
        //hubo.setLegAngles(side, legJointAngsNext[side]);
    }

    for(int side=0; side<2; side++)
    {
        if(LEFT == side)
            std::cout << "LEFT:";
        else if(RIGHT == side)
            std::cout << "RIGHT:";
        std::cout << "\nModified Angles = " << legJointAngsNext[side].transpose();
        std::cout << "\nFoot TF = \n" << footTF[side].matrix() << "\n\n";
    }

//    hubo.sendControls();

    char c;
    std::cout << "Press any key to MOVE the hips\n";
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

    std::cout << "Press any key to STOP the hips\n";
    c = getchar();

    for(int side=0; side<2; side++)
    {
        legJointVels[side].setZero();
        hubo.setLegVels(side, legJointVels[side]);
    }

//    hubo.sendControls();

    return 0;
}
