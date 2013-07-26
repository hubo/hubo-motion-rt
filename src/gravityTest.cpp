/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: July 23, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "DrcHuboKin.h"



int main(int argc, char **argv)
{
    Hubo_Control hubo(false);
    DrcHuboKin kin;
    kin.updateHubo(hubo);


//    kin.linkage("LeftLeg").printInfo();

    ArmVector torques, armAngles;

    armAngles << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    std::cout << "mass: " << kin.linkage("RightArm").mass("RSP") << std::endl;
    std::cout << "com:  " << ( kin.linkage("RightArm").centerOfMass("RSP")
                               - kin.joint("RSR").respectToRobot().translation() ).transpose() << std::endl;


    std::cout << std::endl << std::endl;
    std::cout << "torq: " << kin.joint("RSP").gravityTorque() << std::endl;

//    for(int i=0; i<kin.linkage("RightArm").nChildren(); i++)
//        kin.linkage("RightArm").childLinkage(i).printInfo();


    std::cout << "PARENT" << std::endl;
    kin.joint("RF11").parentJoint().printInfo();

    RobotKin::Joint& parentJoint = kin.joint("RF1").parentJoint();


//    std::cout << "com:  " << kin.linkage("LeftArm").centerOfMass() << std::endl;
//    std::cout << "mass: " << 9.8*kin.linkage("LeftArm").mass() << std::endl;




    while(false)
//    while(true)
    {
        hubo.update(true);
        kin.updateHubo(hubo);

        armAngles(SP) = -90*M_PI/180;
//        armAngles(SR) = -90*M_PI/180;

        kin.updateArmJoints(RIGHT, armAngles);

        kin.armTorques(RIGHT, torques);

        hubo.setArmTorques(RIGHT, torques);

        std::cout << torques.transpose() << std::endl;

        hubo.sendControls();

    }

}
