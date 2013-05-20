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

#include "manip.h"

manip_error_t handle_trans_euler(Hubo_Control &hubo, hubo_manip_state_t &state, hubo_manip_cmd_t &cmd, int side);
manip_error_t handle_trans_quat(Hubo_Control &hubo, hubo_manip_state_t &state, hubo_manip_cmd_t &cmd, int side);
manip_error_t handle_traj(Hubo_Control &hubo, hubo_manip_state_t &state, hubo_manip_cmd_t &cmd, int side);

void grasp_close( Hubo_Control &hubo, int side );
void grasp_open( Hubo_Control &hubo );
void grasp_limp( Hubo_Control &hubo );

int main( int argc, char **argv )
{
    Hubo_Control hubo("manip-daemon");
    
    ach_channel_t chan_manip_cmd;
    ach_channel_t chan_manip_traj;
    ach_channel_t chan_manip_param;
    ach_channel_t chan_manip_state;

    ach_status_t r = ach_open( &chan_manip_cmd, CHAN_HUBO_MANIP_CMD, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );
    
    r = ach_open( &chan_manip_traj, CHAN_HUBO_MANIP_TRAJ, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );

    r = ach_open( &chan_manip_param, CHAN_HUBO_MANIP_PARAM, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );
    
    r = ach_open( &chan_manip_state, CHAN_HUBO_MANIP_STATE, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );
    
    hubo_manip_cmd_t manip_cmd[2];
    hubo_manip_cmd_t manip_req;
    hubo_manip_traj_t manip_traj;
    hubo_manip_param_t manip_param;
    hubo_manip_state manip_state;
    memset( &(manip_cmd[RIGHT]), 0, sizeof(hubo_manip_cmd_t) );
    memset( &(manip_cmd[LEFT]),  0, sizeof(hubo_manip_cmd_t) );
    memset( &manip_req, 0, sizeof(manip_req) );
    memset( &manip_traj, 0, sizeof(manip_traj) );
    memset( &manip_param, 0, sizeof(manip_param) );
    memset( &manip_state, 0, sizeof(manip_state) );
    
    hubo.update(true);
    
    ArmVector zeroAngles; zeroAngles.setZero();
    
    while( !daemon_sig_quit )
    {
        hubo.update(true);
        ach_get( &chan_manip_cmd, &manip_req, sizeof(manip_req), &fs, NULL, ACH_O_LAST );
        
        
        for(int side=0; side<2; side++)
        {
            if( req.interrupt[side] || manip_state.cmd_state[side] == MC_READY )
                memcpy( &(manip_cmd[side]), &manip_req, sizeof(manip_req) );
            
            manip_state.cmd_state[2] = manip_cmd[side].m_cmd[side];
            
            // Handle arm motions
            switch( manip_cmd.m_cmd[side] )
            {
                case MC_TRANS_EULER:
                    handle_trans_euler(hubo, manip_cmd[side], side); break;
                case MC_TRANS_QUAT:
                    handle_trans_quat(hubo, manip_cmd[side], side); break;
                case MC_TRAJ:
                    handle_traj(hubo, manip_cmd[side], side); break;
            }
            
            // Handle grasping
            if( manip_cmd[side].m_grasp[side]==MC_GRASP_NOW ||
               ( manip_cmd[side].m_grasp[side]==MC_GRASP_AT_END && manip_state.cmd_state[side]==MC_READY ) )
                grasp_close(Hubo_Control &hubo, side);
            else if( manip_cmd[side].m_grasp[side]==MC_RELEASE_NOW ||
                    ( manip_cmd[side].m_grasp[side]==MC_RELEASE_AT_END && manip_state.cmd_state[side]==MC_READY ) )
                grasp_open(Hubo_Control &hubo, side);
            else if( manip_cmd[side].m_grasp[side]==MC_GRASP_LIMP )
                grasp_limp(Hubo_Control &hubo, side);
        }
        
        sendControls();
    }
    
    ach_close( &chan_manip_cmd );
    ach_close( &chan_manip_traj );
    ach_close( &chan_manip_param );
    ach_close( &chan_manip_state );
}

void grasp_close( Hubo_Control &hubo, int side )
{
    if( side == RIGHT )
    {
        hubo.passJointAngle(RF1, 1);
        hubo.passJointAngle(RF2, 1);
        hubo.passJointAngle(RF3, 1);
        hubo.passJointAngle(RF4, 1);
        hubo.passJointAngle(RF5, 1);
    }
    else
    {
        hubo.passJointAngle(LF1, 1);
        hubo.passJointAngle(LF2, 1);
        hubo.passJointAngle(LF3, 1);
        hubo.passJointAngle(LF4, 1);
        hubo.passJointAngle(LF5, 1);
    }
}

void grasp_open( Hubo_Control &hubo )
{
    if( side == RIGHT )
    {
        hubo.passJointAngle(RF1, -1);
        hubo.passJointAngle(RF2, -1);
        hubo.passJointAngle(RF3, -1);
        hubo.passJointAngle(RF4, -1);
        hubo.passJointAngle(RF5, -1);
    }
    else
    {
        hubo.passJointAngle(LF1, -1);
        hubo.passJointAngle(LF2, -1);
        hubo.passJointAngle(LF3, -1);
        hubo.passJointAngle(LF4, -1);
        hubo.passJointAngle(LF5, -1);
    }
}

void grasp_limp( Hubo_Control &hubo )
{
    if( side == RIGHT )
    {
        hubo.passJointAngle(RF1, 0);
        hubo.passJointAngle(RF2, 0);
        hubo.passJointAngle(RF3, 0);
        hubo.passJointAngle(RF4, 0);
        hubo.passJointAngle(RF5, 0);
    }
    else
    {
        hubo.passJointAngle(LF1, 0);
        hubo.passJointAngle(LF2, 0);
        hubo.passJointAngle(LF3, 0);
        hubo.passJointAngle(LF4, 0);
        hubo.passJointAngle(LF5, 0);
    }
}


manip_error_t handle_trans_euler(Hubo_Control &hubo, hubo_manip_state_t &state, hubo_manip_cmd_t &cmd, int side)
{
    ArmVector armAngles, armStates;
    Vector3d trans, angles;
    for(int i=0; i<3; i++)
    {
        trans(i) = cmd.translation[side][i];
        angles(i) = cmd.eulerAngles[side][i];
    }
    
    Isometry3d B;
    B = Eigen::Matrix4d::Identity();
    B.translate(trans);
    B.rotate( Eigen::AngleAxisd(rangles(0), Vector3d(1,0,0)) );
    B.rotate( Eigen::AngleAxisd(rangles(1), Vector3d(0,1,0)) );
    B.rotate( Eigen::AngleAxisd(rangles(2), Vector3d(0,0,1)) );
    hubo.huboArmIK( armAngles, B, zeros, side );
    bool valid = hubo.setArmAngles( side, armAngles );
    
    if(valid)
        state.error[side] = MC_NO_ERROR;
    else
        state.error[side] = MC_INVALID_POSE;
    
    hubo.getArmAngleStates( side, armStates );
    if( (armAngles-armStates).norm() < convergeNorm )
        cmd.m_cmd[side] = MC_READY;
    
    state.cmd_state[side] = cmd.m_cmd[side];
}

manip_error_t handle_trans_quat(Hubo_Control &hubo, hubo_manip_state_t &state, hubo_manip_cmd_t &cmd, int side)
{
    ArmVector armAngles;
    Vector3d trans, angles;
    for(int i=0; i<3; i++)
    {
        trans(i) = cmd.translation[side][i];
        angles(i) = cmd.eulerAngles[side][i];
    }
    
    Eigen::Isometry3d B;
    B = Eigen::Matrix4d::Identity();
    B.translate(trans);
    
    Eigen::Quaternion quat;
    quat.w() = cmd.quaternion[side][0];
    quat.x() = cmd.quaternion[side][1];
    quat.y() = cmd.quaternion[side][2];
    quat.z() = cmd.quaternion[side][3];
    B.rotate(quat);
    
    hubo.huboArmIK( armAngles, B, zeros, side );
    bool valid = hubo.setArmAngles( side, armAngles );
    
    if(valid)
        state.error[side] = MC_NO_ERROR;
    else
        state.error[side] = MC_INVALID_POSE;
    
    hubo.getArmAngleStates( side, armStates );
    if( (armAngles-armStates).norm() < convergeNorm )
        cmd.m_cmd[side] = MC_READY;
    
    state.cmd_state[side] = cmd.m_cmd[side];
}

manip_error_t handle_traj(Hubo_Control &hubo, hubo_manip_state_t &state, hubo_manip_cmd_t &cmd, int side)
{
    fprintf(stderr, "Running Trajectories is not yet supported :(\n");
    state.error[side] = MC_INVALID_TRANSITION;
    state.cmd_state[side] = MC_READY;
}
