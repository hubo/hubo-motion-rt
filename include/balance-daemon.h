
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: May 30, 2013
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


#ifndef BALANCE_DAEMON_H
#define BALANCE_DAEMON_H

#define BALANCE_CMD_CHAN "balance-cmd"
#define BALANCE_STATE_CHAN "balance-state"
#define BALANCE_PARAM_CHAN "balance-param"
#define CRPC_PARAM_CHAN "crpc-param"
#define CRPC_STATE_CHAN "crpc-state"
#include "hubo-zmp.h"

typedef enum {

    BAL_READY=0,
    BAL_LEGS_ONLY,
    BAL_ZMP_WALKING,
    BAL_CRPC

} balance_mode_t;

typedef enum {

    WALK_INACTIVE=0,
    WALK_WAITING,
    WALK_INITIALIZING,
    WALK_IN_PROGRESS

} walk_mode_t;

typedef enum {

    NO_WALK_ERROR=0,
    WALK_INIT_FAILED,
    WALK_FAILED_SWAP

} walk_error_t;

typedef enum {

    CRPC_READY=0,
    CRPC_PHASE_1=1,
    CRPC_PHASE_2=2,
    CRPC_PHASE_3=3,
    CRPC_DONE

} crpc_phase_t;

typedef struct crpc_params {

    double kp_upper_body;
    double kp_mass_distrib;
    double kp_zmp_diff;
    double kp_zmp_com;
    double zmp_ref_x;
    double zmp_ref_y;

    int negate_moments;
    
    int from_current_ref;

    double hip_crouch;

}__attribute__((packed)) crpc_params_t;

typedef struct crpc_state {

    double body_angle[2];
    double body_com[2];
    double leg_length[2];
    double foot_angle_x[2];
    double foot_angle_y[2];

    crpc_phase_t phase;

}__attribute__((packed)) crpc_state_t;


typedef struct balance_gains {

    double flattening_gain;
    double decay_gain;
    double force_min_threshold;
    double force_max_threshold;

    double straightening_pitch_gain;
    double straightening_roll_gain;
    
    double spring_gain;
    double damping_gain;
    double fz_response;

    double single_support_hip_nudge_kp;
    double single_support_hip_nudge_kd;
    double double_support_hip_nudge_kp;
    double double_support_hip_nudge_kd;

}__attribute__((packed)) balance_gains_t;

typedef struct walking_gains {

    double flattening_gain;
    double decay_gain;
    double force_min_threshold;
    double force_max_threshold;

    double straightening_pitch_gain;
    double straightening_roll_gain;

    double spring_gain;
    double damping_gain;
    double fz_response;

}__attribute__((packed)) walking_gains_t;

typedef struct balance_params {

    balance_gains_t balance_gains;
    walking_gains_t walking_gains;

}__attribute__((packed)) balance_params_t;

typedef struct balance_cmd {

    balance_mode_t cmd_request;
    
    double height;
    double com_x_offset;

}__attribute__((packed)) balance_cmd_t;

typedef struct balance_state {

    balance_mode_t m_balance_mode;

    walk_mode_t m_walk_mode;
    walk_error_t m_walk_error;
    bipedStance_t biped_stance;

}__attribute__((packed)) balance_state_t;


typedef enum {
    
    T_INVALID,
    T_INCOMPLETE,
    T_COMPLETE

} transition_result_t;


#endif // BALANCE_DAEMON_H

