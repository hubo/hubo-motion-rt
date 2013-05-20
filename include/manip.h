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


#include <Hubo_Control.h>


#define CHAN_HUBO_MANIP_CMD "manip-cmd"
#define CHAN_HUBO_MANIP_TRAJ "manip-traj"
#define CHAN_HUBO_MANIP_PARAM "manip-param"

typedef enum {
    
    MC_READY = 0,
    MC_TRANS_EULER,
    MC_TRANS_QUAT,
    MC_TRAJ
    
} manip_cmd_t;

typedef enum {
    
    MC_GRASP_LIMP = 0,
    MC_GRASP_STATIC,
    MC_GRASP_NOW,
    MC_GRASP_AT_END,
    MC_RELEASE_NOW,
    MC_RELEASE_AT_END
    
} manip_grasp_t;

typedef enum {
    
    MC_NONE,
    MC_FORCE,
    MC_CURRENT
    
} manip_ctrl_t;

typedef enum {
    
    MC_SOLE,
    MC_START,
    MC_MID,
    MC_END
    
} manip_traj_chain_t;

typedef enum {
    
    MC_NO_ERROR = 0,
    MC_INVALID_POSE,
    MC_INVALID_TRANSITION,
    MC_BROKEN_CHAIN
    
} manip_error_t;


typedef struct hubo_manip_state {
    
    manip_cmd_t cmd_state[2];      // Current state of the operational command
    manip_grasp_t grasp_state[2];  // Current state of the grasp command
    
    manip_error_t error[2];
    
} hubo_manip_state_t;


typedef struct hubo_manip_cmd {
    
    manip_cmd_t m_cmd[side];
    manip_ctrl_t m_ctrl[side];
    manip_side_t m_side;
    manip_grasp_t m_grasp[2];
    bool interrupt[2];
    
    double translation[2][3];   // Use translation[RIGHT][0] to specify x for the right-side end effector
                                // translation[LEFT][0] -> left arm's x
                                // translation[LEFT][1] -> left arm's y
                                // translation[LEFT][2] -> left arm's z
    
    double quaternion[2][4];    // w, x, y, z
    
    double eulerAngles[2][3];   // Use eulerAngles[LEFT][0] to specify x-axis rotation for left-side end effector
                                // eulerAngles[RIGHT][0] -> right arm's roll
                                // eulerAngles[RIGHT][1] -> right arm's pitch
                                // eulerAngles[RIGHT][2] -> right arm's yaw
    // Euler Angles are applied in the following order: X1, Y2, Z3
    
    double convergeNorm;
    
} hubo_manip_cmd_t;


typedef struct hubo_manip_param {
    
    double mx_P[2][6];
    double mx_D[2][6];
    double mx_I[2][6];
    
    double my_P[2][6];
    double my_D[2][6];
    double my_I[2][6];
    
    double fz_P[2][6];
    double fz_D[2][6];
    double fz_I[2][6];
    
    double current_P[2][ARM_JOINT_COUNT];
    double current_D[2][ARM_JOINT_COUNT];
    double current_I[2][ARM_JOINT_COUNT];
    
} hubo_manip_param_t;

typedef struct hubo_manip_traj {
    
    manip_side_t m_side;
    unsigned int id;
    manip_traj_chain_t chain;
    unsigned int parent_id;
    bool interrupt;
    
    double arm_angles[2][ARM_JOINT_COUNT][MAX_TRAJ_SIZE];
    double arm_speeds[2][ARM_JOINT_COUNT][MAX_TRAJ_SIZE];
    double arm_accels[2][ARM_JOINT_COUNT][MAX_TRAJ_SIZE];
    
    unsigned int count;
    
} hubo_manip_traj_t;
