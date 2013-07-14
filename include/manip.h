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

#ifndef MANIP_H
#define MANIP_H

#include <Hubo_Control.h>

// Trajectory upper bounds
#ifndef MAX_TRAJ_SIZE     // placeholder until install is updated
#define TRAJ_FREQ_HZ 200
#define MAX_TRAJ_TIME 10
#define MAX_TRAJ_SIZE MAX_TRAJ_TIME*TRAJ_FREQ_HZ
#endif

#define CHAN_HUBO_MANIP_CMD "manip-cmd"
#define CHAN_HUBO_MANIP_TRAJ "manip-traj"
#define CHAN_HUBO_MANIP_PARAM "manip-param"
#define CHAN_HUBO_MANIP_STATE "manip-state"
#define CHAN_HUBO_MANIP_OVERRIDE "manip-override"

#define NUM_ARMS 2

typedef enum {
    
    MC_READY = 0,
    MC_HALT,
    MC_TRANS_EULER,
    MC_TRANS_QUAT,
    MC_ANGLES,
    MC_TRAJ
    
}__attribute__((packed)) manip_mode_t;

typedef enum {

    OVR_SOVEREIGN = 0,
    OVR_ACQUIESCENT

}__attribute__((packed)) override_t;

typedef enum {
    
    MC_GRASP_LIMP = 0,
    MC_GRASP_STATIC,
    MC_GRASP_NOW,
    MC_GRASP_AT_END,
    MC_RELEASE_NOW,
    MC_RELEASE_AT_END
    
}__attribute__((packed)) manip_grasp_t;

typedef enum {
    
    MC_NONE,
    MC_FORCE,
    MC_CURRENT
    
}__attribute__((packed)) manip_ctrl_t;

typedef enum {
    
    MC_SOLE,
    MC_START,
    MC_MID,
    MC_END
    
}__attribute__((packed)) manip_traj_chain_t;

typedef enum {
    
    MC_NO_ERROR = 0,
    MC_INVALID_POSE,
    MC_INVALID_TRANSITION,
    MC_BROKEN_CHAIN
    
}__attribute__((packed)) manip_error_t;

typedef struct manip_override {

    override_t m_override;

}__attribute__((packed)) manip_override_t;

/**
 * \union manip_pose_t
 * \brief Contains all pose parameters to pass to manipulation daemon for ik-control
 *
 * This structure is defined as a union, so one can access the stored values like
 * myPose.data[1] or myPose.y, and they will refer to the same chunk of memory.
 * Names are provided for both quaternion and Euler angles.
 */
typedef union 
{
	double data[7];  ///< Array of raw data containing pose info.
	struct
	{
		double x,y,z;
		union
		{
			struct
			{
				double w,i,j,k;
			};

			struct
			{
				double alpha,beta,gamma,empty;
			};
		};
	};
}__attribute__((packed)) hubo_manip_pose_t;


typedef struct hubo_manip_state {

    uint32_t goalID[NUM_ARMS];
    manip_mode_t mode_state[NUM_ARMS];    ///< Current state of the operational mode. Changes to manip_mode_t::MC_READY when path finished.
    manip_grasp_t grasp_state[NUM_ARMS];  ///< Current state of the grasp command
    manip_error_t error[NUM_ARMS];        ///< Current error state of the daemon
    hubo_manip_pose_t pose[NUM_ARMS];
    override_t override;
    
}__attribute__((packed)) hubo_manip_state_t;


typedef struct hubo_manip_cmd {
    
    uint32_t goalID[NUM_ARMS];
    manip_mode_t m_mode[NUM_ARMS];        ///< Defines what type of manipulation to execute: trajectory or pose
    manip_ctrl_t m_ctrl[NUM_ARMS];        ///< Defines the type of compliance to use
    manip_grasp_t m_grasp[NUM_ARMS];      ///< Defines at what point to perform a grasp
    double waistAngle;
    bool interrupt[NUM_ARMS];             ///< Interrupts the specified arm's execution
    
    hubo_manip_pose_t pose[NUM_ARMS];     ///< Defines a pose target for the arm. Ignored if m_mode == manip_mode_t::MC_TRAJ
    double arm_angles[NUM_ARMS][ARM_JOINT_COUNT];   ///< Defines the joint angles for each arm

    double stopNorm;
    double convergeNorm;
    
}__attribute__((packed)) hubo_manip_cmd_t;


typedef struct hubo_manip_param {
    
    double mx_P[NUM_ARMS][6];
    double mx_D[NUM_ARMS][6];
    double mx_I[NUM_ARMS][6];
    
    double my_P[NUM_ARMS][6];
    double my_D[NUM_ARMS][6];
    double my_I[NUM_ARMS][6];
    
    double fz_P[NUM_ARMS][6];
    double fz_D[NUM_ARMS][6];
    double fz_I[NUM_ARMS][6];
    
    double current_P[NUM_ARMS][ARM_JOINT_COUNT];
    double current_D[NUM_ARMS][ARM_JOINT_COUNT];
    double current_I[NUM_ARMS][ARM_JOINT_COUNT];
    
}__attribute__((packed)) hubo_manip_param_t;

typedef struct hubo_manip_traj {
    
    unsigned int id;
    manip_traj_chain_t chain;
    unsigned int parent_id;
    bool interrupt;
    
    // TODO: would this be more efficient as an array of structs rather than several separate arrays?
    // I assume the data at each timestep is accessed with the greatest locality...
    double arm_angles[NUM_ARMS][ARM_JOINT_COUNT][MAX_TRAJ_SIZE];
    double arm_speeds[NUM_ARMS][ARM_JOINT_COUNT][MAX_TRAJ_SIZE];
    double arm_accels[NUM_ARMS][ARM_JOINT_COUNT][MAX_TRAJ_SIZE];
    
    unsigned int count;
    
}__attribute__((packed)) hubo_manip_traj_t;

#ifdef __cplusplus
static std::ostream& operator<<(std::ostream& stream, const hubo_manip_pose_t& pose)
{
	stream << "[ ";
	for (int i = 0; i < 7; i++)
	{
		stream << pose.data[i];
		if (2 == i)
		{
			stream << "; ";
		}
		else if (6 == i)
		{
			// Do nothing
		}
		else
		{
			stream << ", ";
		}
	}
	stream << " ]";

	return stream;
}

static std::ostream& operator<<(std::ostream& stream, const hubo_manip_cmd_t& cmd)
{
	for (int i = 0; i < NUM_ARMS; i++)
	{
		stream << "Arm " << i << ":" << std::endl;
		stream << "\tGoal ID: " << cmd.goalID[i] << std::endl;
		stream << "\tMode: " << cmd.m_mode[i] << std::endl;
		stream << "\tControl: " << cmd.m_ctrl[i] << std::endl;
		stream << "\tGrasp: " << cmd.m_grasp[i] << std::endl;
		stream << "\tInterrupt: " << cmd.interrupt[i] << std::endl;
		stream << "\tPose: " << cmd.pose[i] << std::endl;

		stream << "\tJoints: [ ";
		for (int j = 0; j < ARM_JOINT_COUNT; j++)
		{
			stream << cmd.arm_angles[i][j];
			if (j < ARM_JOINT_COUNT - 1)
			{
				stream << ", ";
			}
		}
		stream << " ]" << std::endl;
	}
	
	stream << "waistAngle: " << cmd.waistAngle << std::endl;
	stream << "stopNorm: " << cmd.stopNorm << std::endl;
	stream << "convergeNorm: " << cmd.convergeNorm << std::endl;

	return stream;
}

static std::ostream& operator<<(std::ostream& stream, const hubo_manip_state_t& state)
{
	stream << "Override: " << state.override << std::endl;
	for (int i = 0; i < NUM_ARMS; i++)
	{
		stream << "Arm " << i << ":" << std::endl;
		stream << "\tGoal ID: " << state.goalID[i] << std::endl;
		stream << "\tMode: " << state.mode_state[i] << std::endl;
		stream << "\tGrasp: " << state.grasp_state[i] << std::endl;
		stream << "\tPose: " << state.pose[i] << std::endl;

		stream << "\tError: " << state.error[i] << std::endl;

	}
	
	return stream;
}
#endif // __cplusplus

#endif // MANIP_H
