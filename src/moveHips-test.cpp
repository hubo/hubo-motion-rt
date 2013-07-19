
/**
 * \brief Test program to test the leg FK and IK as well
 * as hipVelocityIK
*/

//#include "DrcHuboKin.h"
#include "Hubo_Control.h"
#include <iostream>

int main(int argc, char **argv)
{
    Hubo_Control hubo;
    //DrcHuboKin kin;
    hubo.update();

    std::vector<LegVector, Eigen::aligned_allocator<LegVector> > legJointVels(2);
    std::vector<LegVector, Eigen::aligned_allocator<LegVector> > legJointAngs(2);
    std::vector<LegVector, Eigen::aligned_allocator<LegVector> > legJointAngsNext(2);
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > footTF(2);

    //------------------------
    //     RAISE FEET
    //------------------------
    for(int side=0; side<2; side++)
    {
        legJointAngs[side] << 0,0,0,0,0, 0,0,0,0,0;
        //footTF[side] = kin.linkage("LeftLeg").tool().respectToWorld();
        hubo.huboLegFK(footTF[side], legJointAngs[side], side);
        if(LEFT == side)
            std::cout << "LEFT:";
        else if(RIGHT == side)
            std::cout << "RIGHT:";
        std::cout << "\nPrevious Angles = " << legJointAngs[side].transpose();
        std::cout << "\nFoot TF = \n" << footTF[side].matrix() << "\n\n";
        footTF[side](2,3) += 0.02;
        hubo.huboLegIK(legJointAngsNext[side], footTF[side], legJointAngs[side], side);
        hubo.setLegAngles(side, legJointAngsNext[side]);
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

    hubo.sendControls();
    hubo.update();

    char c;
    std::cout << "Press any key to MOVE the hips\n";
    c = getchar();
    hubo.update();

    //-------------------------
    //        MOVE HIPS
    //-------------------------
    Eigen::Vector3d hipVelocity;

    double startTime = hubo.getTime();
    double ptime = hubo.getTime();
    double relativeTime = 0;
    double dt = 0;
    int counter = 0; int counterMax = 40;
    double timeout = 4.0;

    while(!daemon_sig_quit)
    {
        hubo.update();
        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();
        relativeTime = ptime - startTime;
        if(relativeTime > timeout)
            break;

        counter++;

        hipVelocity << 0.02, 0.02, 0.0;

        //---------------------------
        //   LEG JOINT VELOCITIES
        //---------------------------
        if(dt > 0)
        {
            for(int side=0; side<2; side++)
            {
                hubo.hipVelocityIK(legJointVels[side], hipVelocity, side);
                hubo.setLegVels(side, legJointVels[side]);
            }

/*            if(counter > counterMax)
            {
                for(int side=0; side<2; side++)
                {
                    hubo.getLegVels(side, legJointVels[side]);
                    if(LEFT == side)
                        std::cout << "LEFT";
                    else if(RIGHT == side)
                        std::cout << "RIGHT";
                    std::cout << "\nlegVels = " << legJointVels[side].transpose() << "\n\n";
                }
            }
*/            hubo.sendControls();
        }
    }

    for(int side=0; side<2; side++)
    {
        legJointVels[side].setZero();
        hubo.setLegVels(side, legJointVels[side]);
    }

    hubo.sendControls();
    hubo.update();

    std::cout << "Press any key to PRINT feet poses\n";
    c = getchar();

    for(int side=0; side<2; side++)
    {
        hubo.getLegAngleStates(side, legJointAngsNext[side]);
        hubo.huboLegFK(footTF[side], legJointAngsNext[side], side);
        if(LEFT == side)
            std::cout << "LEFT:";
        else if(RIGHT == side)
            std::cout << "RIGHT:";
        std::cout << "\nModified Angles = " << legJointAngsNext[side].transpose();
        std::cout << "\nFoot TF = \n" << footTF[side].matrix() << "\n\n";
    }

    return 0;
}
