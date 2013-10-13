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

#include "DrcHuboKin.h"
#include "manip.h"
#include "Slerper.h"

extern "C" {
// For process management
#include "daemonizer.h"
}

// FIXME: Comment out if reflex and amino don't exist
manip_error_t handle_teleop(Hubo_Control &hubo, hubo_manip_state_t &state,
                hubo_manip_cmd_t &cmd, ArmVector &arm, int side);

manip_error_t handle_trans_euler(Hubo_Control &hubo, hubo_manip_state_t &state,
                hubo_manip_cmd_t &cmd, ArmVector &arm, int side);

manip_error_t handle_trans_quat(Hubo_Control &hubo, hubo_manip_state_t &state,
                hubo_manip_cmd_t &cmd, ArmVector &arm, int side);

manip_error_t handle_traj(Hubo_Control &hubo, hubo_manip_state_t &state,
                hubo_manip_cmd_t &cmd, ArmVector &arm, int side);

manip_error_t handle_halt(Hubo_Control &hubo, hubo_manip_state_t &state,
                hubo_manip_cmd_t &cmd, ArmVector &arm, int side);

manip_error_t handle_angles(Hubo_Control &hubo, hubo_manip_state_t &state,
                hubo_manip_cmd_t &cmd, ArmVector &arm, int side);

void grasp_close( Hubo_Control &hubo, int side );
void grasp_open( Hubo_Control &hubo, int side );
void grasp_limp( Hubo_Control &hubo, int side );

void trigger_close( Hubo_Control &hubo );
void trigger_open( Hubo_Control &hubo );
void trigger_limp( Hubo_Control &hubo );

void initializeHubo(Hubo_Control &hubo);


int main( int argc, char **argv )
{
//    Hubo_Control hubo("manip-daemon");
    Hubo_Control hubo;
    initializeHubo(hubo);

    DrcHuboKin kin;
    kin.updateHubo(hubo);
    Slerper slerp;
    slerp.resetSlerper(LEFT, hubo);
    slerp.resetSlerper(RIGHT, hubo);
    
    ach_channel_t chan_manip_cmd;
    ach_channel_t chan_manip_traj;
    ach_channel_t chan_manip_param;
    ach_channel_t chan_manip_state;
    ach_channel_t chan_manip_override;

    ach_status_t r = ach_open( &chan_manip_cmd, CHAN_HUBO_MANIP_CMD, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );
    
    r = ach_open( &chan_manip_traj, CHAN_HUBO_MANIP_TRAJ, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );

    r = ach_open( &chan_manip_param, CHAN_HUBO_MANIP_PARAM, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );
    
    r = ach_open( &chan_manip_state, CHAN_HUBO_MANIP_STATE, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );

    r = ach_open( &chan_manip_override, CHAN_HUBO_MANIP_OVERRIDE, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );
    
    hubo_manip_cmd_t manip_cmd[2];
    hubo_manip_cmd_t manip_req;
    hubo_manip_traj_t manip_traj;
    hubo_manip_param_t manip_param;
    hubo_manip_state manip_state;
    manip_override_t override_cmd;

    memset( &(manip_cmd[RIGHT]), 0, sizeof(hubo_manip_cmd_t) );
    memset( &(manip_cmd[LEFT]),  0, sizeof(hubo_manip_cmd_t) );
    memset( &manip_req, 0, sizeof(manip_req) );
    memset( &manip_traj, 0, sizeof(manip_traj) );
    memset( &manip_param, 0, sizeof(manip_param) );
    memset( &manip_state, 0, sizeof(manip_state) );
    memset( &override_cmd, 0, sizeof(override_cmd) );

    ArmVector arms[2];
    ArmVector torques[2];
    Vector6d eeWrench[2];
    
    ArmVector highGainsP, highGainsD; highGainsP.setOnes(); highGainsD.setZero(); 
    highGainsP *= 80;
    highGainsP(SR) = 100;
    highGainsP(SY) = 150;
    highGainsP(WY) = 100;
    highGainsP(WP) = 120;
    
    double time=hubo.getTime(), dt=0;
    hubo.update(true);
    dt = hubo.getTime() - time;
    time = hubo.getTime();

    ArmVector defaultNomSpeed, defaultNomAcc; defaultNomSpeed.setOnes(); defaultNomAcc.setOnes();
    defaultNomSpeed *= 1.0;
    defaultNomAcc *= 0.8;

    hubo.setLeftArmNomSpeeds(defaultNomSpeed);
    hubo.setRightArmNomSpeeds(defaultNomSpeed);
    hubo.setLeftArmNomAcc(defaultNomAcc);
    hubo.setRightArmNomAcc(defaultNomAcc);
    hubo.setArmAntiFriction(LEFT, false);
    hubo.setArmAntiFriction(RIGHT, false);

    hubo.storeArmDefaults(LEFT);
    hubo.storeArmDefaults(RIGHT);

    bool newRequest[2] = {false, false};
    
    size_t fs;
    while( !daemon_sig_quit )
    {
        // Update hubo, and get latest manipulation and override commands from ach
        hubo.update(true);
        dt = hubo.getTime() - time;
        time = hubo.getTime();
        kin.updateHubo(hubo);
        
        ach_status_t r = ach_get( &chan_manip_cmd, &manip_req, sizeof(manip_req), &fs, NULL, ACH_O_LAST );
        if( ACH_OK == r || ACH_MISSED_FRAME == r )
        {
            for(int nr=0; nr<2; nr++)
                newRequest[nr] = true;
        }
        ach_get( &chan_manip_override, &override_cmd, sizeof(override_cmd), &fs, NULL, ACH_O_LAST );

        if(manip_req.trigger == MC_GRASP_NOW ||
                ( manip_cmd[RIGHT].m_grasp[RIGHT]==MC_GRASP_AT_END
                 && manip_state.mode_state[RIGHT]==MC_READY ) )
            trigger_close(hubo);
        else if(manip_req.trigger == MC_RELEASE_NOW ||
                ( manip_cmd[RIGHT].m_grasp[RIGHT]==MC_RELEASE_AT_END
                && manip_state.mode_state[RIGHT]==MC_READY ) )
            trigger_open(hubo);
        else if( manip_cmd[RIGHT].m_grasp[RIGHT]==MC_GRASP_LIMP )
            trigger_limp(hubo);


        manip_state.override = override_cmd.m_override;

        // Handle commands for left and right arms
        for(int side=0; side<2; side++)
        {
            // If manip should interrupt current arm motion or it's ready,
            // then copy manip_req into manip_cmd
            if( newRequest[side] && (manip_req.interrupt[side] || manip_state.mode_state[side] == MC_READY) )
            {
                memcpy( &(manip_cmd[side]), &manip_req, sizeof(manip_req) );
                newRequest[side] = false;
            }

            // Update manip state mode and goal ID            
            manip_state.mode_state[side] = manip_cmd[side].m_mode[side];
            manip_state.goalID[side] = manip_cmd[side].goalID[side];

            // Handle arm motions
            switch( manip_cmd[side].m_mode[side] )
            {
                case MC_TELEOP:
                case MC_DUAL_TELEOP:
                    slerp.commenceSlerping(side, manip_cmd[side], hubo, dt); break;
                case MC_TRANS_EULER:
                    hubo.resetArmDefaults(side);
                    slerp.resetSlerper(side, hubo);
                    handle_trans_euler(hubo, manip_state, manip_cmd[side], arms[side], side); break;
                case MC_TRANS_QUAT:
                    hubo.resetArmDefaults(side);
                    slerp.resetSlerper(side, hubo);
                    handle_trans_quat(hubo, manip_state, manip_cmd[side], arms[side], side); break;
                case MC_TRAJ:
                    hubo.resetArmDefaults(side);
                    slerp.resetSlerper(side, hubo);
                    handle_traj(hubo, manip_state, manip_cmd[side], arms[side], side); break;
                case MC_HALT:
                    hubo.resetArmDefaults(side);
                    slerp.resetSlerper(side, hubo);
                    handle_halt(hubo, manip_state, manip_cmd[side], arms[side], side); break;
                case MC_ANGLES:
                    hubo.resetArmDefaults(side);
                    slerp.resetSlerper(side, hubo);
                    handle_angles(hubo, manip_state, manip_cmd[side], arms[side], side); break;
                case MC_READY:
                    slerp.resetSlerper(side, hubo);
            }
            
            // Handle grasping
            if( manip_cmd[side].m_grasp[side]==MC_GRASP_NOW ||
                   ( manip_cmd[side].m_grasp[side]==MC_GRASP_AT_END
                    && manip_state.mode_state[side]==MC_READY ) )
                grasp_close(hubo, side);
            else if( manip_cmd[side].m_grasp[side]==MC_RELEASE_NOW ||
                    ( manip_cmd[side].m_grasp[side]==MC_RELEASE_AT_END
                    && manip_state.mode_state[side]==MC_READY ) )
                grasp_open(hubo, side);
            else if( manip_cmd[side].m_grasp[side]==MC_GRASP_LIMP )
                grasp_limp(hubo, side);

            Eigen::Isometry3d Breal; ArmVector qreal;
            hubo.getArmAngleStates( side, qreal );
            hubo.huboArmFK( Breal, qreal, side );
            Eigen::Quaterniond quatreal( Breal.rotation() );
            Eigen::Vector3d transreal( Breal.translation() );
            
            manip_state.pose[side].w = quatreal.w();
            manip_state.pose[side].i = quatreal.x();
            manip_state.pose[side].j = quatreal.y();
            manip_state.pose[side].k = quatreal.z();

            for(int i=0; i<3; i++)
                manip_state.pose[side].data[i] = transreal(i);


            // Handle Control Mode
            if(manip_cmd[side].m_ctrl[side]!=MC_RIGID  && side==RIGHT)
                kin.linkage("RightArm").tool().massProperties.setMass(
                            manip_cmd[side].m_tool[side].mass,
                            Vector3d(manip_cmd[side].m_tool[side].com_x,
                                     manip_cmd[side].m_tool[side].com_y,
                                     manip_cmd[side].m_tool[side].com_z));
            else if(manip_cmd[side].m_ctrl[side]!=MC_RIGID  && side==LEFT)
                kin.linkage("LeftArm").tool().massProperties.setMass(
                            manip_cmd[side].m_tool[side].mass,
                            Vector3d(manip_cmd[side].m_tool[side].com_x,
                                     manip_cmd[side].m_tool[side].com_y,
                                     manip_cmd[side].m_tool[side].com_z));


            if( manip_cmd[side].m_ctrl[side] == MC_RIGID )
            {
                hubo.setArmCompliance(side, false);
                hubo.releaseArmTorques(side); // Just in case
            }
            else if( manip_cmd[side].m_ctrl[side] == MC_COMPLIANT )
            {
                hubo.setArmCompliance(side, true, highGainsP, highGainsD);
                kin.armTorques(side, torques[side]);
                hubo.setArmTorques(side, torques[side]);
            }
            else if( manip_cmd[side].m_ctrl[side] == MC_FORCE )
            {
                for(int i=0; i<6; i++)
                    eeWrench[side][i] = manip_cmd[side].m_wrench[side].data[i];

                hubo.setArmCompliance(side, false);
                kin.armTorques(side, torques[side], eeWrench[side]);
                hubo.setArmTorques(side, torques[side]);
            }
            else if( manip_cmd[side].m_ctrl[side] == MC_HYBRID )
            {
                for(int i=0; i<6; i++)
                    eeWrench[side][i] = manip_cmd[side].m_wrench[side].data[i];

                hubo.setArmCompliance(side, true, highGainsP, highGainsD);
                kin.armTorques(side, torques[side], eeWrench[side]);
                hubo.setArmTorques(side, torques[side]);
            }

        }

        hubo.setJointAngle( WST, manip_req.waistAngle );
        
        // NOTE WELL: THIS MUST BE THE ONLY PLACE THAT hubo.sendControls() IS USED!!!
        if( OVR_SOVEREIGN == manip_state.override )
            hubo.sendControls();
        else
        {
            for(int i=0; i<2; i++)
            {
                slerp.resetSlerper(i, hubo);
                for(int j=0; j<2; j++)
                    manip_cmd[j].m_mode[i] = MC_HALT;
            }
        }

        hubo.releaseArm(LEFT);
        hubo.releaseArm(RIGHT);

        ach_put( &chan_manip_state, &manip_state, sizeof(manip_state) );
    }
    
    ach_close( &chan_manip_cmd );
    ach_close( &chan_manip_traj );
    ach_close( &chan_manip_param );
    ach_close( &chan_manip_state );

    return 0;
}

void initializeHubo( Hubo_Control &hubo )
{
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        hubo.setJointMaxPWM(i, 8);

    hubo.setJointMaxPWM(LSR, 15);
    hubo.setJointMaxPWM(LSY, 15);
    hubo.setJointMaxPWM(LWY, 15);
    hubo.setJointMaxPWM(LWP, 15);

    hubo.setJointMaxPWM(RSR, 15);
    hubo.setJointMaxPWM(RSY, 15);
    hubo.setJointMaxPWM(RWY, 15);
    hubo.setJointMaxPWM(RWP, 15);
}


void grasp_close( Hubo_Control &hubo, int side )
{
    if( side == RIGHT )
    {
        hubo.passJointAngle(RF1, 1);
    }
    else
    {
        hubo.passJointAngle(LF1, 1);
    }
}

void grasp_open( Hubo_Control &hubo, int side )
{
    if( side == RIGHT )
    {
        hubo.passJointAngle(RF1, -1);
    }
    else
    {
        hubo.passJointAngle(LF1, -1);
    }
}

void grasp_limp( Hubo_Control &hubo, int side )
{
    if( side == RIGHT )
    {
        hubo.passJointAngle(RF1, 0);
    }
    else
    {
        hubo.passJointAngle(LF1, 0);
    }
}

void trigger_close( Hubo_Control &hubo )
{
    hubo.passJointAngle(RF2, 1);
}

void trigger_open( Hubo_Control &hubo )
{
    hubo.passJointAngle(RF2, -1);
}

void trigger_limp( Hubo_Control &hubo )
{
    hubo.passJointAngle(RF2, 0);
}

manip_error_t handle_halt(Hubo_Control &hubo, hubo_manip_state_t &state, hubo_manip_cmd_t &cmd, ArmVector &arm, int side)
{
    ArmVector currentAngles;
    hubo.getArmAngles( side, currentAngles );
    hubo.setArmAngles( side, currentAngles );
    // We set the desired position to wherever we happen to currently be.

    state.mode_state[side] = cmd.m_mode[side];
}

manip_error_t handle_trans_euler(Hubo_Control &hubo, hubo_manip_state_t &state, hubo_manip_cmd_t &cmd, ArmVector &arm, int side)
{
    ArmVector zeroAngles; zeroAngles.setZero();
    ArmVector armAngles, armStates; armAngles.setZero();
    Vector3d trans, angles;
    for(int i=0; i<3; i++)
    {
        trans(i) = cmd.pose[side].data[i];
        angles(i) = cmd.pose[side].data[i+3];
    }
    
    Eigen::Isometry3d B;
    B = Eigen::Matrix4d::Identity();
    B.translate(trans);
    B.rotate( Eigen::AngleAxisd(angles(0), Vector3d(1,0,0)) );
    B.rotate( Eigen::AngleAxisd(angles(1), Vector3d(0,1,0)) );
    B.rotate( Eigen::AngleAxisd(angles(2), Vector3d(0,0,1)) );
    bool valid = hubo.huboArmIK( armAngles, B, zeroAngles, side );
    hubo.setArmAngles( side, armAngles );
    
    if(valid)
        state.error[side] = MC_NO_ERROR;
    else
        state.error[side] = MC_INVALID_POSE;

    hubo.getArmAngleStates( side, armStates );
    if( (armAngles-armStates).norm() < cmd.convergeNorm )
        cmd.m_mode[side] = MC_READY;
    state.mode_state[side] = cmd.m_mode[side];

    // If arm is too far from where it should be
    if( (arm - armStates).norm() < cmd.stopNorm )
        state.mode_state[side] = MC_HALT;

    hubo.getArmAngleStates( side, arm );
}

manip_error_t handle_trans_quat(Hubo_Control &hubo, hubo_manip_state_t &state, hubo_manip_cmd_t &cmd, ArmVector &arm, int side)
{
    // Local variables
    ArmVector zeroAngles; zeroAngles.setZero();
    ArmVector armAngles, armStates; armAngles.setZero();
    Vector3d trans, angles;

    // Set position terms
    for(int i=0; i<3; i++)
        trans(i) = cmd.pose[side].data[i];

    // Convert trans_quat into Isometry3d to be used in IK function   
    Eigen::Isometry3d B;
    B = Eigen::Matrix4d::Identity();
    B.translate(trans);
    
    Eigen::Quaterniond quat;
    quat.w() = cmd.pose[side].w;
    quat.x() = cmd.pose[side].i;
    quat.y() = cmd.pose[side].j;
    quat.z() = cmd.pose[side].k;
    B.rotate(quat);

    // Run IK on pose to get joint angles and return whether or not
    // joints solution is valid. And set the joint angles 
    bool valid = hubo.huboArmIK( armAngles, B, zeroAngles, side );
    hubo.setArmAngles( side, armAngles );
 
    // Set error flag in manip state struct   
    if(valid)
        state.error[side] = MC_NO_ERROR;
    else
        state.error[side] = MC_INVALID_POSE;

    // Fill in arm joint states.
    // If close enough to desired angles then set mode to READY
    // Set state mode to command mode
    hubo.getArmAngleStates( side, armStates );
    if( (armAngles-armStates).norm() < cmd.convergeNorm )
        cmd.m_mode[side] = MC_READY;
    state.mode_state[side] = cmd.m_mode[side];

    // If the arm is too far from where it should be, we stop
    if( (arm - armStates).norm() < cmd.stopNorm )
        state.mode_state[side] = MC_HALT;

    hubo.getArmAngleStates( side, arm );
}










manip_error_t handle_traj(Hubo_Control &hubo, hubo_manip_state_t &state, hubo_manip_cmd_t &cmd, ArmVector &arm, int side)
{
    fprintf(stderr, "Running Trajectories is not yet supported :(\n");
    state.error[side] = MC_INVALID_TRANSITION;
    state.mode_state[side] = MC_READY;
}

manip_error_t handle_angles(Hubo_Control &hubo, hubo_manip_state_t &state, hubo_manip_cmd_t &cmd, ArmVector &arm, int side)
{
    // Set arm joint angles and get arm joint states
    ArmVector armAngles, armStates;

    // Fill in ArmVector from manip_cmd
    for(int j=0; j<ARM_JOINT_COUNT; j++)
        armAngles(j) = cmd.arm_angles[side][j];

    // Set arm angles to desired positions and
    // get arm joint states
    hubo.setArmAngles( side, armAngles );

    // Fill in arm joint states.
    // If close enough to desired angles then set mode to READY
    // Set state mode to command mode
    hubo.getArmAngleStates( side, armStates );
    if( (armAngles-armStates).norm() < cmd.convergeNorm )
        cmd.m_mode[side] = MC_READY;
    state.mode_state[side] = cmd.m_mode[side];

    // If we're within the stop norm then say that manip is stopped
    if( (arm - armStates).norm() < cmd.stopNorm )
        state.mode_state[side] = MC_HALT;

    hubo.getArmAngleStates( side, arm );
}
