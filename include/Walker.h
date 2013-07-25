
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


#ifndef WALKER_H
#define WALKER_H

#include "hubo-zmp.h"
#include "ImpedanceController.h"

typedef struct nudge_state {

    Eigen::Vector3d vprev;
    Eigen::Vector3d verr;

    Eigen::Vector3d nudge;
    Eigen::Vector3d spin;
    Eigen::Vector3d integratedFeetOffset;
    Eigen::Vector3d prevIntegratedFeetOffset;
    Eigen::Matrix<double,6,1> dFeetOffset;
    Eigen::Matrix<double,6,1> prevdFeetOffset;

    double ankle_roll_compliance[2];
    double ankle_pitch_compliance[2];
    double ankle_roll_resistance[2];
    double ankle_pitch_resistance[2];

    double knee_offset[2];
    double knee_velocity_offset[2];
    Eigen::Vector3d prevTorqueErr[2];
    Eigen::Vector3d imu_offset;

    double V0[HUBO_JOINT_COUNT];
    
} nudge_state_t;


class Walker
{

public:

    /**
     * \brief Constructor for Walker class.
     * \param maxInitTime Seconds to allow for getting into walk ready position.
     * \param jointSpaceTolerance Tolerance in radians for when a desired body position is reached
     * \param jointVelContTol Tolerance in radians/s for when a desired joint velocity is reached
    */
    Walker(double maxInitTime=15, double jointSpaceTolerance=0.0075, double jointVelContTol=6.0);

    /**
     * \brief Destructor for Walker class
    */
    ~Walker();

    ach_channel_t zmp_chan;         //!< Ach channel for receiving ZMP trajectories
    ach_channel_t param_chan;       //!< Ach channel for receiving gain parameters
    ach_channel_t bal_cmd_chan;     //!< Ach channel for receiving balance commands
    ach_channel_t bal_state_chan;   //!< Ach channel for sending balancing state

    /**
     * \brief Start walking by getting a new ZMP trajector over ach and getting into
     * the initial position, and then executing the trajectory.
     * \param parent_state balance state
     * \param state Nudge state which stores the integrated states
     * \param gains Gains for balancing
    */
    void commenceWalking(balance_state_t &parent_state, nudge_state_t &state, balance_gains_t &gains);

    double m_jointSpaceTolerance;   //!< Joint space tolerance in radians.
    double m_jointVelContTol;       //!< Joint velocity tolerance in radians/s.
    double m_maxInitTime;           //!< Max wait time for initializing to walk position.

    balance_cmd_t cmd;              //!< Balance command struct
    balance_state_t bal_state;      //!< Balance state struct

    bool keepWalking;   //!< Whether or not to keep walking.
    int counter;        //!< Print counter

protected:


    /// Hubo_Control object used to get state info and send commands. 
    Hubo_Control hubo;

    /// DrcHuboKin object
    DrcHuboKin kin;

    /// ImpedanceController object
    ImpedanceController impCtrl;

    /**
     * \brief Executes trajectory for current timestep using controllers
     * with gains specified by the user.
     * \param hubo Hubo_Control object to get state info from
     * \param prevElem Previous ZMP trajectory element
     * \param currentElem ZMP trajectory element we are currently executing
     * \param nextElem Next ZMP trajectory element we will execute
     * \param state Nudge state which stores the integrated states
     * \param gains Gains for balancing
     * \param dt Time change between last update and current update
     * \return void
    */
    void executeTimeStep( Hubo_Control &hubo, zmp_traj_element &prevElem,
            zmp_traj_element_t &currentElem, zmp_traj_element &nextElem,
            nudge_state_t &state, balance_gains_t &gains, double dt );

    /**
     * \brief Checks for a new trajectory from the ach channel.
     * \param newTrajectory Trajectory variable to be set by reference.
     * \param haveNewTrajAlready Whether or not we have a new traj already.
     * \return Whether or not we got a new trajectory. 
    */
    bool checkForNewTrajectory(zmp_traj_t &newTrajectory, bool haveNewTrajAlready);

    /**
     * \brief Validates the trajectory at the swap-in point by checking the velocity
     * of all the joint.
     * \param current Current zmp_traj_element, which holds the joint angles.
     * \param next Next zmp_traj_element, which hold the joint angles.
     * \param dt Time change between last update and current update
     * \return bool Whether or not the swap-in point is continuous.
    */
    bool validateNextTrajectory( zmp_traj_element_t &current, zmp_traj_element_t &next, double dt );

    /**
     * \brief Complies the ankles so that the feet are flat on
     * the ground by integrating the ankle torques, which are
     * saved in the nudge state and then sent to the robot. It
     * uses compliance.
     * \param hubo Hubo_Control object to get state info from
     * \param elem ZMP trajectory element we are currently executing
     * \param state Nudge state which stores the integrated ankle
     * roll and pitch terms.
     * \param gains Ankle gain values for balancing
     * \param dt Time change between last update and current update
     * \return void
    */
    void flattenFoot( Hubo_Control &hubo, zmp_traj_element_t &elem,
			nudge_state_t &state, balance_gains_t &gains, double dt );

    /**
     * \brief Straightens the back by adjusting the ankle pitch and
     * roll based on feedback from the IMU sensor. The joint angles
     * in the zmp_traj_element get modified based on the integrated
     * nudge state and sent to the robot. It uses resistance.
     * \param hubo Hubo_Control object to get state info from
     * \param elem ZMP trajectory element we are currently executing
     * \param state Nudge state which stores the integrated ankle
     * roll and pitch terms.
     * \param gains Ankle gain values for balancing
     * \param dt Time change between last update and current update
     * \return void
    */
    void straightenBack( Hubo_Control &hubo, zmp_traj_element_t &elem,
            nudge_state_t &state, balance_gains_t &gains, double dt );

    /**
     * \brief Complies the knee joints (as well as the ankle and hip
     * pitch joints) using a spring and damper controller.
     * \param hubo Hubo_Control object to get state info from
     * \param elem ZMP trajectory element we are currently executing
     * \param state Nudge state which stores the integrated knee offset
       state.
     * \param gains Knee gain values for balancing
     * \param dt Time change between last update and current update
     * \return void
    */
    void complyKnee( Hubo_Control &hubo, zmp_traj_element_t &elem,
        nudge_state_t &state, balance_gains_t &gains, double dt );


    /**
     * \brief Send balance state, which include balance mode, walk
     * mode and walk errors.
     * \return void
`   */
    void sendState();

    /**
     * \brief Check balance commands, which include the requested
     * balance mode and the desired height of the neck.
    */
    void checkCommands();

};



#endif // WALKER_H

