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


using namespace RobotKin;


int main(int argc, char **argv)
{
    bool constrained = false;

    double kp = 0;
    double kd = 0;

    if(constrained)
    {
        kp = atof(argv[1]);
        kd = atof(argv[2]);
    }

    if( !(0 <= kp && kp < 1000) )
        return -1;

    if( !(0 <= kd && kd <= kp ) )
        return -1;

    std::cout << "kp:" << kp << "\tkd:" << kd << std::endl;

    Hubo_Control hubo;
    DrcHuboKin kin;
    kin.updateHubo(hubo);
//    kin.linkage("LeftLeg").printInfo();

    ArmVector torques, armAngles, gravity;

    TRANSFORM startL = kin.linkage("LeftArm").tool().respectToRobot();
    TRANSFORM currentL = kin.linkage("LeftArm").tool().respectToRobot();

    TRANSFORM startR = kin.linkage("RightArm").tool().respectToRobot();
    TRANSFORM currentR = kin.linkage("RightArm").tool().respectToRobot();

    TRANSLATION err, rot; rot.setZero();
    SCREW vel;
    SCREW wrench;

    ArmJacobian J;
    ArmVector qdotC;
    Eigen::Matrix< double, 7, 1 > qdot;

    int iter=0, maxi=20;

    hubo.setJointAntiFriction(LSP, true);
    hubo.setJointAntiFriction(LSR, true);
    hubo.setJointAntiFriction(LSY, true);
    hubo.setJointAntiFriction(LEB, true);
    hubo.setJointAntiFriction(LWY, true);
    hubo.setJointAntiFriction(LWP, true);


    hubo.setJointAntiFriction(RSP, true);
    hubo.setJointAntiFriction(RSR, true);
    hubo.setJointAntiFriction(RSY, true);
    hubo.setJointAntiFriction(REB, true);
    hubo.setJointAntiFriction(RWY, true);
    hubo.setJointAntiFriction(RWP, true);

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
/*
        currentL = kin.linkage("LeftArm").tool().respectToRobot();

        err = startL.translation() - currentL.translation();
        err(0) *= 0;
        err(1) *= kp; // 200 was okay
        err(2) *= 0;

        J = kin.armJacobian(LEFT);

        hubo.getArmVels(LEFT, qdotC);
        for(int i=0; i<7; i++)
            qdot(i) = qdotC(i);

        vel = J*qdot;
        vel(0) *= 0;
        vel(1) *= -kd;
        vel(2) *= 0;
        vel(3) *= 0;
        vel(4) *= 0;
        vel(5) *= 0;

        wrench << err, rot;
        wrench += vel;
*/

//        kin.armTorques(LEFT, torques, wrench);

//        hubo.setJointTorque(LSP, torques(SP));
//        hubo.setJointTorque(LSR, torques(SR));
//        hubo.setJointTorque(LSY, torques(SY));
//        hubo.setJointTorque(LEB, torques(EB));
//        hubo.setJointTorque(LWY, torques(WY));
//        hubo.setJointTorque(LWP, torques(WP));
////        hubo.setArmTorques(LEFT, torques);

//        if(iter==maxi)
//            std::cout << torques(EB) << "\t";
/*
        currentR = kin.linkage("RightArm").tool().respectToRobot();

        err = startR.translation() - currentR.translation();
        err(0) *= 0;
        err(1) *= kp; // 200 was okay
        err(2) *= 0;

        J = kin.armJacobian(RIGHT);

        hubo.getArmVels(RIGHT, qdotC);
        for(int i=0; i<7; i++)
            qdot(i) = qdotC(i);

        vel = J*qdot;
        vel(0) *= 0;
        vel(1) *= -kd;
        vel(2) *= 0;
        vel(3) *= 0;
        vel(4) *= 0;
        vel(5) *= 0;

        wrench << err, rot;
        wrench += vel;
*/

//        kin.armTorques(RIGHT, torques, wrench);
//        kin.armTorques(RIGHT, gravity);
        kin.armTorques(RIGHT, torques);
        
        hubo.setJointTorque(RSP, torques(SP));
        hubo.setJointTorque(RSR, torques(SR));
        hubo.setJointTorque(RSY, torques(SY));
        hubo.setJointTorque(REB, torques(EB));
        hubo.setJointTorque(RWY, torques(WY));
        hubo.setJointTorque(RWP, torques(WP));

        if(iter==maxi)
        {
/*
            std::cout << "Total: " << torques.transpose() << std::endl;
            std::cout << "Grav:  " << gravity.transpose() << std::endl;
            std::cout << "Diff:  " << (torques-gravity).transpose() << std::endl;
*/
            std::cout << "Torque: " << torques(WY) << "\tVel:" << qdot(WY) << std::endl;
        }

        hubo.sendControls();

    }

}
