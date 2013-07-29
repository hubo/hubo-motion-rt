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

    int iter=0, maxi=75;

    hubo.setJointAntiFriction(LSP, true);
    hubo.setJointAntiFriction(LSR, true);
    hubo.setJointAntiFriction(LSY, true);
    hubo.setJointAntiFriction(LEB, true);
    hubo.setJointAntiFriction(LWY, true);
    hubo.setJointAntiFriction(LWP, true);


//    hubo.setJointAntiFriction(RSP, true);
//    hubo.setJointAntiFriction(RSR, true);
//    hubo.setJointAntiFriction(RSY, true);
//    hubo.setJointAntiFriction(REB, true);
//    hubo.setJointAntiFriction(RWY, true);
//    hubo.setJointAntiFriction(RWP, true);

/*
    hubo.setJointTorque(LSP, 0);
    hubo.setJointTorque(LSR, 0);
    hubo.setJointTorque(LSY, 0);
    hubo.setJointTorque(LEB, 0);
    hubo.setJointTorque(LWY, 0);
    hubo.setJointTorque(LWP, 0);

    hubo.setJointTorque(RSP, 0);
    hubo.setJointTorque(RSR, 0);
    hubo.setJointTorque(RSY, 0);
    hubo.setJointTorque(REB, 0);
    hubo.setJointTorque(RWY, 0);
    hubo.setJointTorque(RWP, 0);
*/
//    hubo.sendControls();

    while(true)
    {
        iter++;
        if(iter>maxi) iter=0;

        hubo.update(true);
        kin.updateHubo(hubo);

        kin.armTorques(LEFT, torques);

        hubo.setJointTorque(LSP, torques(SP));
        hubo.setJointTorque(LSR, torques(SR));
        hubo.setJointTorque(LSY, torques(SY));
        hubo.setJointTorque(LEB, torques(EB));
        hubo.setJointTorque(LWY, torques(WY));
        hubo.setJointTorque(LWP, torques(WP));
//        hubo.setArmTorques(LEFT, torques);

//        kin.armTorques(RIGHT, torques);
        
//        hubo.setJointTorque(RSP, torques(SP));
//        hubo.setJointTorque(RSR, torques(SR));
//        hubo.setJointTorque(RSY, torques(SY));
//        hubo.setJointTorque(REB, torques(EB));

        if(iter==maxi)
            std::cout << torques(WP) << std::endl;
//            std::cout << torques.transpose() << std::endl;

        hubo.sendControls();

    }

}
