/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Feb 03, 2013
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


#ifndef CONTROLDAEMON_H
#define CONTROLDAEMON_H

// For Hubo
#include "hubo.h"
#include "hubo-jointparams.h"

// For control
#include <math.h>
#include "daemonizer.h"

#define ARM_JOINT_COUNT 6
#define LEG_JOINT_COUNT 6
#define FIN_JOINT_COUNT 5
#define AUX_JOINT_COUNT 4

#define     HUBO_CHAN_RL_CTRL_NAME      "hubo-RL-control" // Right Leg control channel
#define     HUBO_CHAN_LL_CTRL_NAME      "hubo-LL-control" // Left Leg control channel
#define     HUBO_CHAN_RA_CTRL_NAME      "hubo-RA-control" // Right Arm control channel
#define     HUBO_CHAN_LA_CTRL_NAME      "hubo-LA-control" // Left Arm control channel
#define     HUBO_CHAN_RF_CTRL_NAME      "hubo-RF-control" // Right Finger control channel
#define     HUBO_CHAN_LF_CTRL_NAME      "hubo-LF-control" // Left Finger control channel
#define     HUBO_CHAN_AUX_CTRL_NAME     "hubo-AUX-control"// Neck and Waist control channel
#define     CTRL_CHAN_STATE             "ctrl-d-state"    // Control daemon state channel


// TODO: Save these as parameters defined in a table instead:
const int leftarmjoints[ARM_JOINT_COUNT]  = { LSP, LSR, LSY, LEB, LWY, LWP };
const int rightarmjoints[ARM_JOINT_COUNT] = { RSP, RSR, RSY, REB, RWY, RWP };
const int leftlegjoints[LEG_JOINT_COUNT]  = { LHY, LHR, LHP, LKN, LAP, LAR };
const int rightlegjoints[LEG_JOINT_COUNT] = { RHY, RHR, RHP, RKN, RAP, RAR };
const int leftfinjoints[FIN_JOINT_COUNT]  = { LF1, LF2, LF3, LF4, LF5 };
const int rightfinjoints[FIN_JOINT_COUNT]  = { RF1, RF2, RF3, RF4, RF5 };
const int auxjoints[AUX_JOINT_COUNT] = { WST, NKY, NK1, NK2 }; 

typedef enum {
    CTRL_OFF    = 0,
    CTRL_POS,
    CTRL_VEL,
    CTRL_HOME,
    CTRL_RESET,
    CTRL_PASS
} hubo_ctrl_mode_t;


struct hubo_joint_control {
    double position;
    double velocity;
    double acceleration;

    double speed_limit;

    double pos_min;
    double pos_max;

    double timeOut;

    hubo_ctrl_mode_t mode;
};

struct hubo_control {
    struct hubo_joint_control joint[HUBO_JOINT_COUNT];
    int active;
};

struct hubo_arm_control {
    struct hubo_joint_control joint[ARM_JOINT_COUNT];
    int active;
};

struct hubo_leg_control {
    struct hubo_joint_control joint[LEG_JOINT_COUNT];
    int active;
};

struct hubo_fin_control {
    struct hubo_joint_control joint[FIN_JOINT_COUNT];
    int active;
};

struct hubo_aux_control {
    struct hubo_joint_control joint[AUX_JOINT_COUNT];
    int active;
};

struct hubo_ctrl_state {
    int status[HUBO_JOINT_COUNT];
    int paused;
};


#endif // CONTROLDAEMON_H
