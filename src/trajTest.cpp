/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: August 9, 2013
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
    Hubo_Control hubo;

    double stime, time, dt=0, angle=0, pangle=0, vel;

    double T = 10;

    stime = hubo.getTime();
    time = hubo.getTime();

//    hubo.setJointCompliance(LEB, true, 40, 20);
//    hubo.setJointCompliance(LEB, true);

    double frequency = 10;
    hubo.setAllTrajFrequency(frequency);

    hubo.update(true);

    while(true)
    {
        dt = hubo.getTime() - time;
        time = hubo.getTime();
        

        angle = M_PI/4*cos(2*M_PI*(time-stime)/T) - M_PI/4;

        vel = (angle - pangle)/dt;
        std::cout << "Angle: " << angle << "\t\tVel: " << vel << std::endl;

//        hubo.setJointAngle(LEB, angle);
        hubo.setJointTraj(LEB, angle, vel);
//        hubo.passJointAngle(LEB, angle);
        hubo.sendControls();
        

        pangle = angle;

        usleep(1.0/frequency*1e6);
        hubo.update(false);

    }


}
