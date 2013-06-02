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

extern "C" {
#include "daemonizer.h"
}

Hubo_Control::Hubo_Control()
{
    controlInit();
}

Hubo_Control::Hubo_Control(const char *daemon_name, int priority)
{
    controlInit();
    daemonize(daemon_name, priority);
}

Hubo_Control::~Hubo_Control()
{
    ach_close( &chan_hubo_ref );
    ach_close( &chan_hubo_state );
    ach_close( &chan_hubo_board_cmd );
    ach_close( &chan_hubo_arm_ctrl_left );
    ach_close( &chan_hubo_arm_ctrl_right );
    ach_close( &chan_hubo_leg_ctrl_left );
    ach_close( &chan_hubo_leg_ctrl_right );
    ach_close( &chan_hubo_fin_ctrl_left );
    ach_close( &chan_hubo_fin_ctrl_right );
    ach_close( &chan_hubo_bod_ctrl );
    ach_close( &chan_hubo_nck_ctrl );

    daemon_close();
}

void Hubo_Control::controlInit()
{
    kneeSingularityThreshold = 0.2;
    kneeSingularityDanger = 0.15;
    kneeSingularitySpeed = 0.1;

    shinLength = 0.26;

    memset( &H_Ref,   0, sizeof(H_Ref)   );
    memset( &H_Cmd,   0, sizeof(H_Cmd)   );
    memset( &H_State, 0, sizeof(H_State) );

    memset( H_Arm_Ctrl,  0, 2*sizeof(H_Arm_Ctrl[0]) );
    memset( H_Leg_Ctrl,  0, 2*sizeof(H_Leg_Ctrl[0]) );
    memset( H_Fin_Ctrl,  0, 2*sizeof(H_Fin_Ctrl[0]) );
    memset( &H_Bod_Ctrl, 0, sizeof(H_Bod_Ctrl) );
    memset( &H_Nck_Ctrl, 0, sizeof(H_Nck_Ctrl) );

    memset( ctrlMap, 0, sizeof(ctrlMap[0])*HUBO_JOINT_COUNT );
    memset( localMap, 0, sizeof(localMap[0])*HUBO_JOINT_COUNT );

    memset( jointAngleCalibration, 0, sizeof(jointAngleCalibration[0])*HUBO_JOINT_COUNT );


    for(int i=0; i<8; i++)
        ctrlOn[i] = false;

    int r = ach_open( &chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL );
    assert( ACH_OK == r );

    r = ach_open( &chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL );
    assert( ACH_OK == r );

    r = ach_open( &chan_hubo_board_cmd, HUBO_CHAN_BOARD_CMD_NAME, NULL );
    assert( ACH_OK == r );

    r = ach_open( &chan_hubo_arm_ctrl_right, HUBO_CHAN_RA_CTRL_NAME, NULL );
    assert( ACH_OK == r );

    r = ach_open( &chan_hubo_arm_ctrl_left,  HUBO_CHAN_LA_CTRL_NAME, NULL );
    assert( ACH_OK == r );
    
    r = ach_open( &chan_hubo_leg_ctrl_right, HUBO_CHAN_RL_CTRL_NAME, NULL );
    assert( ACH_OK == r );

    r = ach_open( &chan_hubo_leg_ctrl_left,  HUBO_CHAN_LL_CTRL_NAME, NULL );
    assert( ACH_OK == r );
    
    r = ach_open( &chan_hubo_fin_ctrl_right, HUBO_CHAN_RF_CTRL_NAME, NULL );
    assert( ACH_OK == r );
    
    r = ach_open( &chan_hubo_fin_ctrl_left,  HUBO_CHAN_LF_CTRL_NAME, NULL );
    assert( ACH_OK == r );
    
    r = ach_open( &chan_hubo_bod_ctrl, HUBO_CHAN_BOD_CTRL_NAME, NULL );
    assert( ACH_OK == r );
    
    r = ach_open( &chan_hubo_nck_ctrl, HUBO_CHAN_NCK_CTRL_NAME, NULL );
    assert( ACH_OK == r );
    
    r = ach_open( &chan_ctrl_state, CTRL_CHAN_STATE, NULL );
    assert( ACH_OK == r );

    size_t fs;
    
    

    ach_get( &chan_hubo_ref, &H_Ref, sizeof(H_Ref), &fs, NULL, ACH_O_WAIT );
    ach_get( &chan_hubo_state, &H_State, sizeof(H_State), &fs, NULL, ACH_O_WAIT );

    ach_status_t checkCtrl;

    double quitSec = 0.5;
    struct timespec waitTime;
    clock_gettime( ACH_DEFAULT_CLOCK, &waitTime );
    int nanoWait = waitTime.tv_nsec + (int)(quitSec*1E9);
    waitTime.tv_sec += (int)(nanoWait/1E9);
    waitTime.tv_nsec = (int)(nanoWait%((int)1E9));
    checkCtrl = ach_get( &chan_hubo_arm_ctrl_right, &H_Arm_Ctrl[RIGHT], sizeof(H_Arm_Ctrl[RIGHT]),
                            &fs, &waitTime, ACH_O_LAST | ACH_O_WAIT );
    assert( ACH_TIMEOUT != checkCtrl );

    clock_gettime( ACH_DEFAULT_CLOCK, &waitTime );
    nanoWait = waitTime.tv_nsec + (int)(quitSec*1E9);
    waitTime.tv_sec += (int)(nanoWait/1E9);
    waitTime.tv_nsec = (int)(nanoWait%((int)1E9));
    checkCtrl = ach_get( &chan_hubo_arm_ctrl_left,  &H_Arm_Ctrl[LEFT],  sizeof(H_Arm_Ctrl[LEFT]),
                            &fs, &waitTime, ACH_O_LAST | ACH_O_WAIT );
    assert( ACH_TIMEOUT != checkCtrl );

    clock_gettime( ACH_DEFAULT_CLOCK, &waitTime );
    nanoWait = waitTime.tv_nsec + (int)(quitSec*1E9);
    waitTime.tv_sec += (int)(nanoWait/1E9);
    waitTime.tv_nsec = (int)(nanoWait%((int)1E9));
    checkCtrl = ach_get( &chan_hubo_leg_ctrl_right, &H_Leg_Ctrl[RIGHT], sizeof(H_Leg_Ctrl[RIGHT]),
                            &fs, &waitTime, ACH_O_LAST | ACH_O_WAIT );
    assert( ACH_TIMEOUT != checkCtrl );

    clock_gettime( ACH_DEFAULT_CLOCK, &waitTime );
    nanoWait = waitTime.tv_nsec + (int)(quitSec*1E9);
    waitTime.tv_sec += (int)(nanoWait/1E9);
    waitTime.tv_nsec = (int)(nanoWait%((int)1E9));
    checkCtrl = ach_get( &chan_hubo_leg_ctrl_left,  &H_Leg_Ctrl[LEFT],  sizeof(H_Leg_Ctrl[LEFT]),
                            &fs, &waitTime, ACH_O_LAST | ACH_O_WAIT );
    assert( ACH_TIMEOUT != checkCtrl );

    clock_gettime( ACH_DEFAULT_CLOCK, &waitTime );
    nanoWait = waitTime.tv_nsec + (int)(quitSec*1E9);
    waitTime.tv_sec += (int)(nanoWait/1E9);
    waitTime.tv_nsec = (int)(nanoWait%((int)1E9));
    checkCtrl = ach_get( &chan_hubo_fin_ctrl_right, &H_Fin_Ctrl[RIGHT], sizeof(H_Fin_Ctrl[RIGHT]),
                            &fs, &waitTime, ACH_O_LAST | ACH_O_WAIT );
    assert( ACH_TIMEOUT != checkCtrl );

    clock_gettime( ACH_DEFAULT_CLOCK, &waitTime );
    nanoWait = waitTime.tv_nsec + (int)(quitSec*1E9);
    waitTime.tv_sec += (int)(nanoWait/1E9);
    waitTime.tv_nsec = (int)(nanoWait%((int)1E9));
    checkCtrl = ach_get( &chan_hubo_fin_ctrl_left,  &H_Fin_Ctrl[LEFT],  sizeof(H_Fin_Ctrl[LEFT]),
                            &fs, &waitTime, ACH_O_LAST | ACH_O_WAIT );
    assert( ACH_TIMEOUT != checkCtrl );

    clock_gettime( ACH_DEFAULT_CLOCK, &waitTime );
    nanoWait = waitTime.tv_nsec + (int)(quitSec*1E9);
    waitTime.tv_sec += (int)(nanoWait/1E9);
    waitTime.tv_nsec = (int)(nanoWait%((int)1E9));
    checkCtrl = ach_get( &chan_hubo_bod_ctrl, &H_Bod_Ctrl, sizeof(H_Bod_Ctrl),
                            &fs, &waitTime, ACH_O_LAST | ACH_O_WAIT );
    assert( ACH_TIMEOUT != checkCtrl );

    clock_gettime( ACH_DEFAULT_CLOCK, &waitTime );
    nanoWait = waitTime.tv_nsec + (int)(quitSec*1E9);
    waitTime.tv_sec += (int)(nanoWait/1E9);
    waitTime.tv_nsec = (int)(nanoWait%((int)1E9));
    checkCtrl = ach_get( &chan_hubo_nck_ctrl, &H_Nck_Ctrl, sizeof(H_Nck_Ctrl),
                            &fs, &waitTime, ACH_O_LAST | ACH_O_WAIT );
    assert( ACH_TIMEOUT != checkCtrl );

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        ctrlMap[i] = -1;
    }

    for(int i=0; i<H_Arm_Ctrl[LEFT].count; i++)
    {
        armjoints[LEFT][i] = H_Arm_Ctrl[LEFT].jointIndices[i];
        ctrlMap[ armjoints[LEFT][i] ] = CtrlLA;
        localMap[ armjoints[LEFT][i]  ] = i;
    }

    for(int i=0; i<H_Arm_Ctrl[RIGHT].count; i++)
    {
        armjoints[RIGHT][i] = H_Arm_Ctrl[RIGHT].jointIndices[i];
        ctrlMap[ armjoints[RIGHT][i] ] = CtrlRA;
        localMap[ armjoints[RIGHT][i]  ] = i;
    }
    
    for(int i=0; i<H_Leg_Ctrl[LEFT].count; i++)
    {
        legjoints[LEFT][i]  = H_Leg_Ctrl[LEFT].jointIndices[i];
        ctrlMap[ legjoints[LEFT][i] ]  = CtrlLL;
        localMap[ legjoints[LEFT][i]  ] = i;
    }

    for(int i=0; i<H_Leg_Ctrl[RIGHT].count; i++)
    {
        legjoints[RIGHT][i]  = H_Leg_Ctrl[RIGHT].jointIndices[i];
        ctrlMap[ legjoints[RIGHT][i] ]  = CtrlRL;
        localMap[ legjoints[RIGHT][i]  ] = i;
    }
    
    for(int i=0; i<H_Fin_Ctrl[LEFT].count; i++)
    {
        finjoints[LEFT][i]  = H_Fin_Ctrl[LEFT].jointIndices[i];
        ctrlMap[ finjoints[LEFT][i] ]  = CtrlLF;
        localMap[ finjoints[LEFT][i]  ] = i;
    }
    
    for(int i=0; i<H_Fin_Ctrl[RIGHT].count; i++)
    {
        finjoints[RIGHT][i] = H_Fin_Ctrl[RIGHT].jointIndices[i];
        ctrlMap[ finjoints[RIGHT][i] ]  = CtrlRF;
        localMap[ finjoints[RIGHT][i]  ] = i;
    }

    for(int i=0; i<H_Bod_Ctrl.count; i++)
    {
        bodjoints[i] = H_Bod_Ctrl.jointIndices[i];
        ctrlMap[ bodjoints[i] ] = CtrlBD;
        localMap[ bodjoints[i] ] = i;
    }
    
    for(int i=0; i<H_Nck_Ctrl.count; i++)
    {
        nckjoints[i] = H_Nck_Ctrl.jointIndices[i];
        ctrlMap[ nckjoints[i] ] = CtrlNK;
        localMap[ nckjoints[i] ] = i;
    }
    
}


void Hubo_Control::releaseArm(int side)
{
    if(side==RIGHT)
        releaseRightArm();
    else if(side==LEFT)
        releaseLeftArm();
}

void Hubo_Control::releaseRightArm()
{
    ctrlOn[CtrlRA] = false;
}

void Hubo_Control::releaseLeftArm()
{
    ctrlOn[CtrlLA] = false;
}

void Hubo_Control::releaseLeg(int side)
{
    if(side==RIGHT)
        releaseRightLeg();
    else if(side==LEFT)
        releaseLeftLeg();
}

void Hubo_Control::releaseRightLeg()
{
    ctrlOn[CtrlRL] = false;
}

void Hubo_Control::releaseLeftLeg()
{
    ctrlOn[CtrlLL] = false;
}

void Hubo_Control::releaseFingers(int side)
{
    if(side==RIGHT)
        releaseRightFingers();
    else if(side==LEFT)
        releaseLeftFingers();
}

void Hubo_Control::releaseRightFingers()
{
    ctrlOn[CtrlRF] = false;
}

void Hubo_Control::releaseLeftFingers()
{
    ctrlOn[CtrlLF] = false;
}

void Hubo_Control::releaseBody()
{
    ctrlOn[CtrlBD] = false;
}

void Hubo_Control::releaseNeck()
{
    ctrlOn[CtrlNK] = false;
}

double Hubo_Control::getTime() { return H_State.time; }

ctrl_flag_t Hubo_Control::update(bool stateWait, double quitSec, bool printError)
{
    int r1, r2;
    size_t fs;

    if( !stateWait )
    {
        r2 = ach_get( &chan_hubo_state, &H_State, sizeof(H_State), &fs, NULL, ACH_O_LAST );
    }
    else
    {
        if(quitSec<0)
            r2 = ach_get( &chan_hubo_state, &H_State, sizeof(H_State), &fs, NULL, ACH_O_WAIT );
        else
        {
            struct timespec waitTime;
            clock_gettime( ACH_DEFAULT_CLOCK, &waitTime );
            int nanoWait = waitTime.tv_nsec + (int)(quitSec*1E9);
            waitTime.tv_sec += (int)(nanoWait/1E9);
            waitTime.tv_nsec = (int)(nanoWait%((int)1E9));
            
            r2 = ach_get( &chan_hubo_state, &H_State, sizeof(H_State), &fs,
                            &waitTime, ACH_O_LAST | ACH_O_WAIT );
        }

        if( ACH_OK != r2 && printError )
            fprintf( stdout, "Ach report -- State Channel: %s at time=%f",
                ach_result_to_string((ach_status_t)r2), getTime() );
        
        if( ACH_OK != r2 && printError )
            fprintf( stdout, "Ach report -- State Channel: %s at time=%f",
                ach_result_to_string((ach_status_t)r2), getTime() );
    }
    
    r1 = ach_get( &chan_hubo_ref, &H_Ref, sizeof(H_Ref), &fs, NULL, ACH_O_LAST );
    if( ACH_OK != r1 && printError )
        fprintf( stdout, "Ach report -- Ref Channel: %s at time=%f",
            ach_result_to_string((ach_status_t)r1), getTime() );

    ach_get( &chan_ctrl_state, &C_State, sizeof(C_State), &fs, NULL, ACH_O_LAST );

    if( r1==ACH_OK && r2==ACH_OK )
        return SUCCESS;
    else
    {
        if( r1==ACH_OK )
            return STATE_STALE;
        else if( r2==ACH_OK )
            return REF_STALE;
        else
            return ALL_STALE;
    }
}

void Hubo_Control::sendControls()
{
    if(ctrlOn[CtrlRA])
        ach_put( &chan_hubo_arm_ctrl_right, &H_Arm_Ctrl[RIGHT], sizeof(H_Arm_Ctrl[RIGHT]) );
    if(ctrlOn[CtrlLA]) 
        ach_put( &chan_hubo_arm_ctrl_left, &H_Arm_Ctrl[LEFT], sizeof(H_Arm_Ctrl[LEFT]) );
    if(ctrlOn[CtrlRL]) 
        ach_put( &chan_hubo_leg_ctrl_right, &H_Leg_Ctrl[RIGHT], sizeof(H_Leg_Ctrl[RIGHT]) );
    if(ctrlOn[CtrlLL]) 
        ach_put( &chan_hubo_leg_ctrl_left, &H_Leg_Ctrl[LEFT], sizeof(H_Leg_Ctrl[LEFT]) );
    if(ctrlOn[CtrlRF]) 
        ach_put( &chan_hubo_fin_ctrl_right, &H_Fin_Ctrl[RIGHT], sizeof(H_Fin_Ctrl[RIGHT]) ); 
    if(ctrlOn[CtrlLF])
        ach_put( &chan_hubo_fin_ctrl_left, &H_Fin_Ctrl[LEFT], sizeof(H_Fin_Ctrl[LEFT]) ); 
    if(ctrlOn[CtrlBD])
        ach_put( &chan_hubo_bod_ctrl, &H_Bod_Ctrl, sizeof(H_Bod_Ctrl) );
    if(ctrlOn[CtrlNK])
        ach_put( &chan_hubo_nck_ctrl, &H_Nck_Ctrl, sizeof(H_Nck_Ctrl) );
    
/*    if( r != ACH_OK ) fprintf(stderr, "Problem sending control commands: (%d) %s\n",
                                r, ach_result_to_string((ach_status_t)r));
*/ //TODO: Maybe generate error messages or something
}


// ~~~*** Sending Control Commands ***~~~ //
// ~~** Setting reference values

// ~* General sets
ctrl_flag_t Hubo_Control::resetJointStatus( int joint, bool send )
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_RESET;
                ctrlOn[ctrlMap[joint]] = true; break;
            case CtrlLA:
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_RESET;
                ctrlOn[ctrlMap[joint]] = true; break;
            case CtrlRL:
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_RESET;
                ctrlOn[ctrlMap[joint]] = true; break;
            case CtrlLL:
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_RESET;
                ctrlOn[ctrlMap[joint]] = true; break;
            case CtrlRF:
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_RESET;
                ctrlOn[ctrlMap[joint]] = true; break;
            case CtrlLF:
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_RESET;
                ctrlOn[ctrlMap[joint]] = true; break;
            case CtrlBD:
                H_Bod_Ctrl.joint[localMap[joint]].mode = CTRL_RESET;
                ctrlOn[ctrlMap[joint]] = true; break;
            case CtrlNK:
                H_Nck_Ctrl.joint[localMap[joint]].mode = CTRL_RESET;
                ctrlOn[ctrlMap[joint]] = true; break;
        }
    }
    else
        return JOINT_OOB;

    if( send )
        sendControls();

    return SUCCESS;
}
// Position control
ctrl_flag_t Hubo_Control::setPositionControl(int joint)
{ return setJointAngle( joint, H_State.joint[joint].pos ); }

ctrl_flag_t Hubo_Control::setJointAngle(int joint, double radians, bool send)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA: // Right Arm
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].position = radians;
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_POS;
                H_Arm_Ctrl[RIGHT].active=1; ctrlOn[CtrlRA] = true; break;
            case CtrlLA: // Left Arm
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].position = radians;
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_POS;
                H_Arm_Ctrl[LEFT].active=1; ctrlOn[CtrlLA] = true; break;
            case CtrlRL: // Right Leg
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].position = radians;
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_POS;
                H_Leg_Ctrl[RIGHT].active=1; ctrlOn[CtrlRL] = true; break;
            case CtrlLL: // Left Leg
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].position = radians;
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_POS;
                H_Leg_Ctrl[LEFT].active=1; ctrlOn[CtrlLL] = true; break;
            case CtrlRF: // Right Fingers
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].position = radians;
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_POS;
                H_Fin_Ctrl[RIGHT].active=1; ctrlOn[CtrlRF] = true; break;
            case CtrlLF: // Left Fingers
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].position = radians;
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_POS;
                H_Fin_Ctrl[LEFT].active=1; ctrlOn[CtrlLF] = true; break;
            case CtrlBD: // Right Fingers
                H_Bod_Ctrl.joint[localMap[joint]].position = radians;
                H_Bod_Ctrl.joint[localMap[joint]].mode = CTRL_POS;
                H_Bod_Ctrl.active=1; ctrlOn[CtrlBD] = true; break;
            case CtrlNK: // Right Fingers
                H_Nck_Ctrl.joint[localMap[joint]].position = radians;
                H_Nck_Ctrl.joint[localMap[joint]].mode = CTRL_POS;
                H_Nck_Ctrl.active=1; ctrlOn[CtrlNK] = true; break;
        }

        if(send)
            sendControls();
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setJointNominalSpeed(int joint, double speed)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA: // Right Arm
                    H_Arm_Ctrl[RIGHT].joint[localMap[joint]].speed = speed;
                break;
            case CtrlLA: // Left Arm
                    H_Arm_Ctrl[LEFT].joint[localMap[joint]].speed = speed;
                break;
            case CtrlRL: // Right Leg
                    H_Leg_Ctrl[RIGHT].joint[localMap[joint]].speed = speed;
                break;
            case CtrlLL: // Left Leg
                    H_Leg_Ctrl[LEFT].joint[localMap[joint]].speed = speed;
                break;
            case CtrlRF: // Right Fingers
                    H_Fin_Ctrl[RIGHT].joint[localMap[joint]].speed = speed;
                break;
            case CtrlLF: // Left Fingers
                    H_Fin_Ctrl[LEFT].joint[localMap[joint]].speed = speed;
                break;
            case CtrlBD: // Body
                    H_Bod_Ctrl.joint[localMap[joint]].speed = speed;
                break;
            case CtrlNK: // Neck
                    H_Nck_Ctrl.joint[localMap[joint]].speed = speed;
                break;
        }
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

// Velocity control
ctrl_flag_t Hubo_Control::setVelocityControl( int joint )
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA: // Right Arm
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].velocity = 0;
                break;
            case CtrlLA: // Left Arm
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].velocity = 0;
                break;
            case CtrlRL: // Right Leg
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].velocity = 0;
                break;
            case CtrlLL: // Left Leg
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].velocity = 0;
                break;
            case CtrlRF: // Right Fingers
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].velocity = 0;
                break;
            case CtrlLF: // Left Fingers
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].velocity = 0;
                break;
            case CtrlBD: // Body
                H_Bod_Ctrl.joint[localMap[joint]].mode = CTRL_VEL;
                H_Bod_Ctrl.joint[localMap[joint]].velocity = 0;
                break;
            case CtrlNK: // Neck
                H_Nck_Ctrl.joint[localMap[joint]].mode = CTRL_VEL;
                H_Nck_Ctrl.joint[localMap[joint]].velocity = 0;
                break;
        }
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setJointVelocity(int joint, double vel, bool send)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA: // Right Arm
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].velocity = vel;
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Arm_Ctrl[RIGHT].active=1; ctrlOn[CtrlRA]=true; break;
            case CtrlLA: // Left Arm
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].velocity = vel;
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Arm_Ctrl[LEFT].active=1; ctrlOn[CtrlLA]=true; break;
            case CtrlRL: // Right Leg
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].velocity = vel;
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Leg_Ctrl[RIGHT].active=1; ctrlOn[CtrlRL]=true; break;
            case CtrlLL: // Left Leg
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].velocity = vel;
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Leg_Ctrl[LEFT].active=1; ctrlOn[CtrlLL]=true; break;
            case CtrlRF: // Right Fingers
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].velocity = vel;
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Fin_Ctrl[RIGHT].active=1; ctrlOn[CtrlRF]=true; break;
            case CtrlLF: // Left Fingers
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].velocity = vel;
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_VEL;
                H_Fin_Ctrl[LEFT].active=1; ctrlOn[CtrlLF]=true; break;
            case CtrlBD: // Body
                H_Bod_Ctrl.joint[localMap[joint]].velocity = vel;
                H_Bod_Ctrl.joint[localMap[joint]].mode = CTRL_VEL;
                H_Bod_Ctrl.active=1; ctrlOn[CtrlBD]=true; break;
            case CtrlNK: // Neck
                H_Nck_Ctrl.joint[localMap[joint]].velocity = vel;
                H_Nck_Ctrl.joint[localMap[joint]].mode = CTRL_VEL;
                H_Nck_Ctrl.active=1; ctrlOn[CtrlNK]=true; break;
        }
        if(send)
            sendControls();
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

// Acceleration setting
ctrl_flag_t Hubo_Control::setJointNominalAcceleration(int joint, double acc)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].acceleration = acc; break;
            case CtrlLA:
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].acceleration = acc; break;
            case CtrlRL:
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].acceleration = acc; break;
            case CtrlLL:
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].acceleration = acc; break;
            case CtrlRF:
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].acceleration = acc; break;
            case CtrlLF:
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].acceleration = acc; break;
            case CtrlBD:
                H_Bod_Ctrl.joint[localMap[joint]].acceleration = acc; break;
            case CtrlNK:
                H_Nck_Ctrl.joint[localMap[joint]].acceleration = acc; break;
        }
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}


// ~* Arm control sets
// Position control
ctrl_flag_t Hubo_Control::setArmPosCtrl(int side)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            setPositionControl(armjoints[side][i]);
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setArmAngles(int side, ArmVector angles, bool send)
{
    if( side==LEFT || side==RIGHT )
       for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            setJointAngle(armjoints[side][i], angles[i], false);

    else
        return BAD_SIDE;

    if(send)
        sendControls();

    return SUCCESS;
}

void Hubo_Control::setLeftArmPosCtrl() { setArmPosCtrl(LEFT); }

ctrl_flag_t Hubo_Control::setLeftArmAngles(ArmVector angles, bool send)
{ return setArmAngles( LEFT, angles, send ); }

void Hubo_Control::setRightArmPosCtrl() { setArmPosCtrl(RIGHT); }

ctrl_flag_t Hubo_Control::setRightArmAngles(ArmVector angles, bool send)
{ return setArmAngles( RIGHT, angles, send ); }


ctrl_flag_t Hubo_Control::setArmNomSpeeds(int side, ArmVector speeds)
{
    if( side==LEFT || side==RIGHT )
    {
//        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
//            if( H_Arm_Ctrl[side].joint[armjoints[side][i]].mode != CTRL_POS )
//                return WRONG_MODE;

        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            setJointNominalSpeed( armjoints[side][i], speeds(i) );
    }
    else
        return BAD_SIDE;


    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLeftArmNomSpeeds(ArmVector speeds)
{ return setArmNomSpeeds(LEFT, speeds); }

ctrl_flag_t Hubo_Control::setRightArmNomSpeeds(ArmVector speeds)
{ return setArmNomSpeeds(RIGHT, speeds); }


// Velocity Control
ctrl_flag_t Hubo_Control::setArmVelCtrl(int side)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            setVelocityControl( armjoints[side][i] );
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setArmVels(int side, ArmVector vels, bool send)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            setJointVelocity(armjoints[side][i], vels(i), false);
    else
        return BAD_SIDE;

    if(send)
        sendControls();

    return SUCCESS;
}

void Hubo_Control::setLeftArmVelCtrl() { setArmVelCtrl(LEFT); }

ctrl_flag_t Hubo_Control::setLeftArmVels(ArmVector vels, bool send)
{ return setArmVels(LEFT, vels, send); }

void Hubo_Control::setRightArmVelCtrl() { setArmVelCtrl(RIGHT); }

ctrl_flag_t Hubo_Control::setRightArmVels(ArmVector vels, bool send)
{ return setArmVels(RIGHT, vels, send); }


// Acceleration settings
ctrl_flag_t Hubo_Control::setArmNomAcc(int side, ArmVector acc)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            setJointNominalAcceleration( armjoints[side][i], acc(i) );
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLeftArmNomAcc(ArmVector acc)
{ return setArmNomAcc( LEFT, acc ); }

ctrl_flag_t Hubo_Control::setRightArmNomAcc(ArmVector acc)
{ return setArmNomAcc( RIGHT, acc ); }


// ~* Leg control sets
// Position control
ctrl_flag_t Hubo_Control::setLegPosCtrl(int side)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            setPositionControl(legjoints[side][i]);
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLegAngles(int side, LegVector angles, bool send)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            setJointAngle(legjoints[side][i], angles[i], false);
    else
        return BAD_SIDE;


    if(send)
        sendControls();

    return SUCCESS;
}

void Hubo_Control::setLeftLegPosCtrl() { setLegPosCtrl(LEFT); }

ctrl_flag_t Hubo_Control::setLeftLegAngles(LegVector angles, bool send)
{ return setLegAngles( LEFT, angles, send ); }

void Hubo_Control::setRightLegPosCtrl() { setLegPosCtrl(RIGHT); }

ctrl_flag_t Hubo_Control::setRightLegAngles(LegVector angles, bool send)
{ return setLegAngles( RIGHT, angles, send ); }


ctrl_flag_t Hubo_Control::setLegNomSpeeds(int side, LegVector speeds)
{
    if( side==LEFT || side==RIGHT )
    {
//        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
//            if( H_Leg_Ctrl[side].joint[legjoints[side][i]].mode != CTRL_POS )
//                return WRONG_MODE;

        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            setJointNominalSpeed( legjoints[side][i], speeds(i) );
    }
    else
        return BAD_SIDE;


    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLeftLegNomSpeeds(LegVector speeds)
{ return setLegNomSpeeds(LEFT, speeds); }

ctrl_flag_t Hubo_Control::setRightLegNomSpeeds(LegVector speeds)
{ return setLegNomSpeeds(RIGHT, speeds); }


// Velocity Control
ctrl_flag_t Hubo_Control::setLegVelCtrl(int side)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            setVelocityControl( legjoints[side][i] );
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLegVels(int side, LegVector vels, bool send)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            setJointVelocity(legjoints[side][i], vels(i), false);
    else
        return BAD_SIDE;

    if(send)
        sendControls();

    return SUCCESS;
}

void Hubo_Control::setLeftLegVelCtrl() { setLegVelCtrl(LEFT); }

ctrl_flag_t Hubo_Control::setLeftLegVels(LegVector vels, bool send)
{ return setLegVels(LEFT, vels, send); }

void Hubo_Control::setRightLegVelCtrl() { setLegVelCtrl(RIGHT); }

ctrl_flag_t Hubo_Control::setRightLegVels(LegVector vels, bool send)
{ return setLegVels(RIGHT, vels, send); }


// Acceleration settings
ctrl_flag_t Hubo_Control::setLegNomAcc(int side, LegVector acc)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            setJointNominalAcceleration( legjoints[side][i], acc(i) );
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLeftLegNomAcc(LegVector acc)
{ return setLegNomAcc( LEFT, acc ); }

ctrl_flag_t Hubo_Control::setRightLegNomAcc(LegVector acc)
{ return setLegNomAcc( RIGHT, acc ); }


// ~~** Setting limit values
// ~* General sets
ctrl_flag_t Hubo_Control::setJointAngleMin(int joint, double radians)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].pos_min = radians; break;
            case CtrlLA:
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].pos_min = radians; break;
            case CtrlRL:
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].pos_min = radians; break;
            case CtrlLL:
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].pos_min = radians; break;
            case CtrlRF:
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].pos_min = radians; break;
            case CtrlLF:
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].pos_min = radians; break;
            case CtrlBD:
                H_Bod_Ctrl.joint[localMap[joint]].pos_min = radians; break;
            case CtrlNK:
                H_Nck_Ctrl.joint[localMap[joint]].pos_min = radians; break;
        }
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setJointAngleMax(int joint, double radians)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].pos_max = radians; break;
            case CtrlLA:
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].pos_max = radians; break;
            case CtrlRL:
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].pos_max = radians; break;
            case CtrlLL:
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].pos_max = radians; break;
            case CtrlRF:
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].pos_max = radians; break;
            case CtrlLF:
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].pos_max = radians; break;
            case CtrlBD:
                H_Bod_Ctrl.joint[localMap[joint]].pos_max = radians; break;
            case CtrlNK:
                H_Nck_Ctrl.joint[localMap[joint]].pos_max = radians; break;
        }
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setJointErrorMax(int joint, double speed)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].error_limit = speed; break;
            case CtrlLA:
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].error_limit = speed; break;
            case CtrlRL:
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].error_limit = speed; break;
            case CtrlLL:
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].error_limit = speed; break;
            case CtrlRF:
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].error_limit = speed; break;
            case CtrlLF:
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].error_limit = speed; break;
            case CtrlBD:
                H_Bod_Ctrl.joint[localMap[joint]].error_limit = speed; break;
            case CtrlNK:
                H_Nck_Ctrl.joint[localMap[joint]].error_limit = speed; break;
        }
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}


// ~~** Getting Reference Values
// ~* General gets
// Position control
hubo_ctrl_mode_t Hubo_Control::getCtrlMode(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                return H_Arm_Ctrl[RIGHT].joint[localMap[joint]].mode; break;
            case CtrlLA:
                return H_Arm_Ctrl[LEFT].joint[localMap[joint]].mode; break;
            case CtrlRL:
                return H_Leg_Ctrl[RIGHT].joint[localMap[joint]].mode; break;
            case CtrlLL:
                return H_Leg_Ctrl[LEFT].joint[localMap[joint]].mode; break;
            case CtrlRF:
                return H_Fin_Ctrl[RIGHT].joint[localMap[joint]].mode; break;
            case CtrlLF:
                return H_Fin_Ctrl[LEFT].joint[localMap[joint]].mode; break;
            case CtrlBD:
                return H_Bod_Ctrl.joint[localMap[joint]].mode; break;
            case CtrlNK:
                return H_Nck_Ctrl.joint[localMap[joint]].mode; break;
        }
    }
    else
        return CTRL_OFF;
}

double Hubo_Control::getJointAngle(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
        return H_Ref.ref[joint] - jointAngleCalibration[joint];
    else
        return 0;
}

double Hubo_Control::getJointAngleCtrl(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                return H_Arm_Ctrl[RIGHT].joint[localMap[joint]].position; break;
            case CtrlLA:
                return H_Arm_Ctrl[LEFT].joint[localMap[joint]].position; break;
            case CtrlRL:
                return H_Leg_Ctrl[RIGHT].joint[localMap[joint]].position; break;
            case CtrlLL:
                return H_Leg_Ctrl[LEFT].joint[localMap[joint]].position; break;
            case CtrlRF:
                return H_Fin_Ctrl[RIGHT].joint[localMap[joint]].position; break;
            case CtrlLF:
                return H_Fin_Ctrl[LEFT].joint[localMap[joint]].position; break;
            case CtrlBD:
                return H_Bod_Ctrl.joint[localMap[joint]].position; break;
            case CtrlNK:
                return H_Nck_Ctrl.joint[localMap[joint]].position; break;
        }
    }
    else
        return 0;
}

double Hubo_Control::getJointNominalSpeed(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                return H_Arm_Ctrl[RIGHT].joint[localMap[joint]].speed; break;
            case CtrlLA:
                return H_Arm_Ctrl[LEFT].joint[localMap[joint]].speed; break;
            case CtrlRL:
                return H_Leg_Ctrl[RIGHT].joint[localMap[joint]].speed; break;
            case CtrlLL:
                return H_Leg_Ctrl[LEFT].joint[localMap[joint]].speed; break;
            case CtrlRF:
                return H_Fin_Ctrl[RIGHT].joint[localMap[joint]].speed; break;
            case CtrlLF:
                return H_Fin_Ctrl[LEFT].joint[localMap[joint]].speed; break;
            case CtrlBD:
                return H_Bod_Ctrl.joint[localMap[joint]].speed; break;
            case CtrlNK:
                return H_Nck_Ctrl.joint[localMap[joint]].speed; break;
        }
    }
    else
        return 0;
}

double Hubo_Control::getJointVelocity(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
        return C_State.requested_vel[joint];
    else
        return 0;
}

// Velocity control
double Hubo_Control::getJointVelocityCtrl(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                return H_Arm_Ctrl[RIGHT].joint[localMap[joint]].velocity; break;
            case CtrlLA:
                return H_Arm_Ctrl[LEFT].joint[localMap[joint]].velocity; break;
            case CtrlRL:
                return H_Leg_Ctrl[RIGHT].joint[localMap[joint]].velocity; break;
            case CtrlLL:
                return H_Leg_Ctrl[LEFT].joint[localMap[joint]].velocity; break;
            case CtrlRF:
                return H_Fin_Ctrl[RIGHT].joint[localMap[joint]].velocity; break;
            case CtrlLF:
                return H_Fin_Ctrl[LEFT].joint[localMap[joint]].velocity; break;
            case CtrlBD:
                return H_Bod_Ctrl.joint[localMap[joint]].velocity; break;
            case CtrlNK:
                return H_Nck_Ctrl.joint[localMap[joint]].velocity; break;
        }
    }
    else
        return 0;
}

// Acceleration setting
double Hubo_Control::getJointNominalAcceleration(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                return H_Arm_Ctrl[RIGHT].joint[localMap[joint]].acceleration; break;
            case CtrlLA:
                return H_Arm_Ctrl[LEFT].joint[localMap[joint]].acceleration; break;
            case CtrlRL:
                return H_Leg_Ctrl[RIGHT].joint[localMap[joint]].acceleration; break;
            case CtrlLL:
                return H_Leg_Ctrl[LEFT].joint[localMap[joint]].acceleration; break;
            case CtrlRF:
                return H_Fin_Ctrl[RIGHT].joint[localMap[joint]].acceleration; break;
            case CtrlLF:
                return H_Fin_Ctrl[LEFT].joint[localMap[joint]].acceleration; break;
            case CtrlBD:
                return H_Bod_Ctrl.joint[localMap[joint]].acceleration; break;
            case CtrlNK:
                return H_Nck_Ctrl.joint[localMap[joint]].acceleration; break;
        }
    }
    else
        return 0;
}

int Hubo_Control::getJointStatus( int joint )
{
    if( joint < HUBO_JOINT_COUNT )
        return C_State.status[joint];
    else
        return 0;
}

bool Hubo_Control::isHomed( int joint )
{
    if(H_State.status[joint].homeFlag != HUBO_HOME_OK)
        return false;
    else
        return true;
}

bool Hubo_Control::errorsExist( int joint )
{
    if(H_State.status[joint].jam == true ||
       H_State.status[joint].pwmSaturated == true ||
       H_State.status[joint].bigError == true ||
       H_State.status[joint].encError == true ||
       H_State.status[joint].driverFault == true ||
       H_State.status[joint].motorFail0 == true ||
       H_State.status[joint].motorFail1 == true)
        return true;
    else
        return false;
}

// ~* Arm control gets
// Position control
ctrl_flag_t Hubo_Control::getArmAngles(int side, ArmVector &angles)
{
    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            angles[i] = getJointAngle(armjoints[side][i]);
    } // TODO: make getArmAngleCtrls
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftArmAngles(ArmVector &angles)
{ getArmAngles(LEFT, angles); }
void Hubo_Control::getRightArmAngles(ArmVector &angles)
{ getArmAngles(RIGHT, angles); }

ctrl_flag_t Hubo_Control::getArmNomSpeeds(int side, ArmVector &speeds)
{
    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            speeds[i] = getJointNominalSpeed(armjoints[side][i]);
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftArmNomSpeeds(ArmVector &speeds)
{ getArmNomSpeeds(LEFT, speeds); }
void Hubo_Control::getRightArmNomSpeeds(ArmVector &speeds)
{ getArmNomSpeeds(RIGHT, speeds); }

ctrl_flag_t Hubo_Control::getArmVels(int side, ArmVector &vels)
{
    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            vels[i] = getJointVelocity(armjoints[side][i]);
    } // TODO: make getArmAngleCtrls
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftArmVels(ArmVector &vels)
{ getArmVels(LEFT, vels); }
void Hubo_Control::getRightArmVels(ArmVector &vels)
{ getArmVels(RIGHT, vels); }


// Velocity control
ctrl_flag_t Hubo_Control::getArmVelCtrls(int side, ArmVector &vels)
{
    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            vels[i] = getJointVelocityCtrl(armjoints[side][i]);
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftArmVelCtrls(ArmVector &vels)
{ getArmVelCtrls(LEFT, vels); }
void Hubo_Control::getRightArmVelCtrls(ArmVector &vels)
{ getArmVelCtrls(RIGHT, vels); }

// Acceleration settings
ctrl_flag_t Hubo_Control::getArmNomAcc(int side, ArmVector &acc)
{
    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            acc[i] = getJointNominalAcceleration(armjoints[side][i]);
    }
    else
        return BAD_SIDE;
}
void Hubo_Control::getLeftArmNomAcc(ArmVector &acc)
{ getArmNomAcc(LEFT, acc); }
void Hubo_Control::getRightArmNomAcc(ArmVector &acc)
{ getArmNomAcc(RIGHT, acc); }


// ~* Leg control gets
// Position control
ctrl_flag_t Hubo_Control::getLegAngles(int side, LegVector &angles)
{
    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            angles[i] = getJointAngle(legjoints[side][i]);
    } // TODO: getLegAngleCtrls
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftLegAngles(LegVector &angles)
{ getLegAngles(LEFT, angles); }
void Hubo_Control::getRightLegAngles(LegVector &angles)
{ getLegAngles(RIGHT, angles); }

ctrl_flag_t Hubo_Control::getLegNomSpeeds(int side, LegVector &speeds)
{
    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            speeds[i] = getJointNominalSpeed(legjoints[side][i]);
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftLegNomSpeeds(LegVector &speeds)
{ getLegNomSpeeds(LEFT, speeds); }
void Hubo_Control::getRightLegNomSpeeds(LegVector &speeds)
{ getLegNomSpeeds(RIGHT, speeds); }

ctrl_flag_t Hubo_Control::getLegVels(int side, LegVector &vels)
{
    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            vels[i] = getJointVelocity(legjoints[side][i]);
    } // TODO: make getLegAngleCtrls
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftLegVels(LegVector &vels)
{ getLegVels(LEFT, vels); }
void Hubo_Control::getRightLegVels(LegVector &vels)
{ getLegVels(RIGHT, vels); }

// Velocity control
ctrl_flag_t Hubo_Control::getLegVelCtrls(int side, LegVector &vels)
{
    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            vels[i] = getJointVelocityCtrl(legjoints[side][i]);
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftLegVelCtrls(LegVector &vels)
{ getLegVelCtrls(LEFT, vels); }
void Hubo_Control::getRightLegVelCtrls(LegVector &vels)
{ getLegVelCtrls(RIGHT, vels); }

// Acceleration settings
ctrl_flag_t Hubo_Control::getLegNomAcc(int side, LegVector &acc)
{
    if( side==LEFT || side==RIGHT )
    {
        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            acc[i] = getJointNominalAcceleration(legjoints[side][i]);
    }
    else
        return BAD_SIDE;
}
void Hubo_Control::getLeftLegNomAcc(LegVector &acc)
{ getLegNomAcc(LEFT, acc); }
void Hubo_Control::getRightLegNomAcc(LegVector &acc)
{ getLegNomAcc(RIGHT, acc); }


// ~~** Getting limit values
// ~* General gets
double Hubo_Control::getJointAngleMin( int joint )
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                return H_Arm_Ctrl[RIGHT].joint[localMap[joint]].pos_min; break;
            case CtrlLA:
                return H_Arm_Ctrl[LEFT].joint[localMap[joint]].pos_min; break;
            case CtrlRL:
                return H_Leg_Ctrl[RIGHT].joint[localMap[joint]].pos_min; break;
            case CtrlLL:
                return H_Leg_Ctrl[LEFT].joint[localMap[joint]].pos_min; break;
            case CtrlRF:
                return H_Fin_Ctrl[RIGHT].joint[localMap[joint]].pos_min; break;
            case CtrlLF:
                return H_Fin_Ctrl[LEFT].joint[localMap[joint]].pos_min; break;
            case CtrlBD:
                return H_Bod_Ctrl.joint[localMap[joint]].pos_min; break;
            case CtrlNK:
                return H_Nck_Ctrl.joint[localMap[joint]].pos_min; break;
        }
    }
    else
        return 0;
}

double Hubo_Control::getJointAngleMax(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                return H_Arm_Ctrl[RIGHT].joint[localMap[joint]].pos_max; break;
            case CtrlLA:
                return H_Arm_Ctrl[LEFT].joint[localMap[joint]].pos_max; break;
            case CtrlRL:
                return H_Leg_Ctrl[RIGHT].joint[localMap[joint]].pos_max; break;
            case CtrlLL:
                return H_Leg_Ctrl[LEFT].joint[localMap[joint]].pos_max; break;
            case CtrlRF:
                return H_Fin_Ctrl[RIGHT].joint[localMap[joint]].pos_max; break;
            case CtrlLF:
                return H_Fin_Ctrl[LEFT].joint[localMap[joint]].pos_max; break;
            case CtrlBD:
                return H_Bod_Ctrl.joint[localMap[joint]].pos_max; break;
            case CtrlNK:
                return H_Nck_Ctrl.joint[localMap[joint]].pos_max; break;
        }
    }
    else
        return 0;
}

double Hubo_Control::getJointErrorMax(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA:
                return H_Arm_Ctrl[RIGHT].joint[localMap[joint]].error_limit; break;
            case CtrlLA:
                return H_Arm_Ctrl[LEFT].joint[localMap[joint]].error_limit; break;
            case CtrlRL:
                return H_Leg_Ctrl[RIGHT].joint[localMap[joint]].error_limit; break;
            case CtrlLL:
                return H_Leg_Ctrl[LEFT].joint[localMap[joint]].error_limit; break;
            case CtrlRF:
                return H_Fin_Ctrl[RIGHT].joint[localMap[joint]].error_limit; break;
            case CtrlLF:
                return H_Fin_Ctrl[LEFT].joint[localMap[joint]].error_limit; break;
            case CtrlBD:
                return H_Bod_Ctrl.joint[localMap[joint]].error_limit; break;
            case CtrlNK:
                return H_Nck_Ctrl.joint[localMap[joint]].error_limit; break;
        }
    }
    else
        return 0;
}


// ~~~*** State Readings ***~~~ //

// ~~** State
// TODO: Stuff like state position, velocity, whatever
double Hubo_Control::getJointAngleState(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
        return H_State.joint[joint].pos; //- jointAngleCalibration[joint];
    else
        return 0;
}

ctrl_flag_t Hubo_Control::getArmAngleStates( int side, ArmVector &angles )
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<H_Arm_Ctrl[side].count; i++)
            angles[i] = getJointAngleState( armjoints[side][i] );
    else
        return BAD_SIDE;

    return SUCCESS;
} 
void Hubo_Control::getRightArmAngleStates( ArmVector &angles )
{ getArmAngleStates( RIGHT, angles ); }
void Hubo_Control::getLeftArmAngleStates( ArmVector &angles )
{ getArmAngleStates( LEFT, angles ); }

ctrl_flag_t Hubo_Control::getLegAngleStates( int side, LegVector &angles )
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<H_Leg_Ctrl[side].count; i++)
            angles[i] = getJointAngleState( legjoints[side][i] );
    else
        return BAD_SIDE;

    return SUCCESS;
} 
void Hubo_Control::getRightLegAngleStates( LegVector &angles )
{ getLegAngleStates( RIGHT, angles ); }
void Hubo_Control::getLeftLegAngleStates( LegVector &angles )
{ getLegAngleStates( LEFT, angles ); }

// ~~** Sensors
// ~* Force-torque
// Mx
double Hubo_Control::getMx(hubo_sensor_index_t sensor)
{
    if( sensor==HUBO_FT_R_FOOT || sensor==HUBO_FT_R_HAND ||
        sensor==HUBO_FT_L_FOOT || sensor==HUBO_FT_L_HAND )
        return H_State.ft[sensor].m_x;
    else
        return 0;
}
double Hubo_Control::getRightHandMx() { getMx(HUBO_FT_R_HAND); }
double Hubo_Control::getLeftHandMx()  { getMx(HUBO_FT_L_HAND); }
double Hubo_Control::getRightFootMx() { getMx(HUBO_FT_R_FOOT); }
double Hubo_Control::getLeftFootMx()  { getMx(HUBO_FT_L_FOOT); }

// My
double Hubo_Control::getMy(hubo_sensor_index_t sensor)
{
    if( sensor==HUBO_FT_R_FOOT || sensor==HUBO_FT_R_HAND ||
        sensor==HUBO_FT_L_FOOT || sensor==HUBO_FT_L_HAND )
        return H_State.ft[sensor].m_y;
    else
        return 0;
}
double Hubo_Control::getRightHandMy() { getMy(HUBO_FT_R_HAND); }
double Hubo_Control::getLeftHandMy()  { getMy(HUBO_FT_L_HAND); }
double Hubo_Control::getRightFootMy() { getMy(HUBO_FT_R_FOOT); }
double Hubo_Control::getLeftFootMy()  { getMy(HUBO_FT_L_FOOT); }

// Fz
double Hubo_Control::getFz(hubo_sensor_index_t sensor)
{
    if( sensor==HUBO_FT_R_FOOT || sensor==HUBO_FT_L_FOOT )
        return H_State.ft[sensor].f_z;
    else
        return 0;
}
double Hubo_Control::getRightFootFz( bool calibrated )
{
    if( calibrated )    
        return getFz(HUBO_FT_R_FOOT) + afc[RIGHT];
    else
        return getFz(HUBO_FT_R_FOOT);
}
double Hubo_Control::getLeftFootFz( bool calibrated )
{
    if( calibrated )
        return getFz(HUBO_FT_L_FOOT) + afc[LEFT];
    else
        return getFz(HUBO_FT_L_FOOT);
}


// ~* Accelerometers
// TiltX
double Hubo_Control::getTiltX(int side)
{
    if( side==LEFT )
        return H_State.imu[TILT_L].a_x;
    else if( side==RIGHT )
        return H_State.imu[TILT_R].a_y;
    else
        return 0;
}
double Hubo_Control::getLeftTiltX() { return getTiltX(LEFT); }
double Hubo_Control::getRightTiltX() { return getTiltX(RIGHT); }

// TiltY
double Hubo_Control::getTiltY(int side)
{
    if( side==LEFT )
        return H_State.imu[TILT_L].a_y;
    else if( side==RIGHT )
        return H_State.imu[TILT_R].a_y;
    else
        return 0;
}
double Hubo_Control::getLeftTiltY() { return getTiltY(LEFT); }
double Hubo_Control::getRightTiltY() { return getTiltY(RIGHT); }

// TiltZ
double Hubo_Control::getTiltZ(int side)
{
    if( side==LEFT )
        return H_State.imu[TILT_L].a_z;
    else if( side==RIGHT )
        return H_State.imu[TILT_R].a_z;
    else
        return 0;
}
double Hubo_Control::getLeftTiltZ() { return getTiltZ(LEFT); }
double Hubo_Control::getRightTiltZ() { return getTiltZ(RIGHT); }

// ~* IMU
double Hubo_Control::getAngleX() { return H_State.imu[IMU].a_x; }
double Hubo_Control::getAngleY() { return H_State.imu[IMU].a_y; }
double Hubo_Control::getRotVelX() { return H_State.imu[IMU].w_x; }
double Hubo_Control::getRotVelY() { return H_State.imu[IMU].w_y; }


ctrl_flag_t Hubo_Control::passJointAngle(int joint, double radians, bool send)
{

    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        {
            case CtrlRA: // Right Arm
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].position = radians;
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_PASS;
                H_Arm_Ctrl[RIGHT].active=1; ctrlOn[CtrlRA] = true; break;
            case CtrlLA: // Left Arm
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].position = radians;
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_PASS;
                H_Arm_Ctrl[LEFT].active=1; ctrlOn[CtrlLA] = true; break;
            case CtrlRL: // Right Leg
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].position = radians;
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_PASS;
                H_Leg_Ctrl[RIGHT].active=1; ctrlOn[CtrlRL] = true; break;
            case CtrlLL: // Left Leg
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].position = radians;
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_PASS;
                H_Leg_Ctrl[LEFT].active=1; ctrlOn[CtrlLL] = true; break;
            case CtrlRF: // Right Fingers
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].position = radians;
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_PASS;
                H_Fin_Ctrl[RIGHT].active=1; ctrlOn[CtrlRF] = true; break;
            case CtrlLF: // Left Fingers
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].position = radians;
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_PASS;
                H_Fin_Ctrl[LEFT].active=1; ctrlOn[CtrlLF] = true; break;
            case CtrlBD: // Right Fingers
                H_Bod_Ctrl.joint[localMap[joint]].position = radians;
                H_Bod_Ctrl.joint[localMap[joint]].mode = CTRL_PASS;
                H_Bod_Ctrl.active=1; ctrlOn[CtrlBD] = true; break;
            case CtrlNK: // Right Fingers
                H_Nck_Ctrl.joint[localMap[joint]].position = radians;
                H_Nck_Ctrl.joint[localMap[joint]].mode = CTRL_PASS;
                H_Nck_Ctrl.active=1; ctrlOn[CtrlNK] = true; break;
        }

        if(send)
            sendControls();
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

// ~~~*** Kinematics ***~~~ //
inline double min(double x, double y) { return ( x > y ) ? y : x; }
inline double max(double x, double y) { return ( x < y ) ? y : x; }

void Hubo_Control::DH2HG(Eigen::Isometry3d &B, double t, double f, double r, double d)
{
    // Convert DH parameters (standard convention) to Homogenuous transformation matrix.
    B = Eigen::Matrix4d::Identity();
    
    B.translate(Eigen::Vector3d(0.,0.,d));
    B.rotate(Eigen::AngleAxisd(t, Eigen::Vector3d::UnitZ()));
    B.translate(Eigen::Vector3d(r,0,0));
    B.rotate(Eigen::AngleAxisd(f, Eigen::Vector3d::UnitX()));
    
}

void Hubo_Control::huboArmFK(Eigen::Isometry3d &B, ArmVector &q, int side)
{
    Vector6d q6;
    for(int i=0; i<6; i++)
        q6[i] = q[i];
    
    huboArmFK(B, q6, side);
    
    for(int i=0; i<6; i++)
        q[i] = q6[i];
}

void Hubo_Control::huboArmFK(Eigen::Isometry3d &B, Vector6d &q, int side)
{
    Eigen::Isometry3d hand;
    hand(0,0) =  1; hand(0,1) =  0; hand(0,2) = 0; hand(0,3) =   0;
    hand(1,0) =  0; hand(1,1) =  0; hand(1,2) =-1; hand(1,3) =   0;
    hand(2,0) =  0; hand(2,1) =  1; hand(2,2) = 0; hand(2,3) =   0;
    hand(3,0) =  0; hand(3,1) =  0; hand(3,2) = 0; hand(3,3) =   1;
    
    huboArmFK(B, q, side, hand);
}

void Hubo_Control::huboArmFK(Eigen::Isometry3d &B, ArmVector &q, int side,  const Eigen::Isometry3d &endEffector)
{
    Vector6d q6;
    for(int i=0; i<6; i++)
        q6[i] = q[i];
    
    huboArmFK(B, q6, side, endEffector);
    
    for(int i=0; i<6; i++)
        q[i] = q6[i];
}

void Hubo_Control::huboArmFK(Eigen::Isometry3d &B, Vector6d &q, int side,  const Eigen::Isometry3d &endEffector)
{
    // Declarations
    Eigen::Isometry3d neck, hand, T;
    Eigen::MatrixXd limits(6,2);
    Vector6d offset; offset.setZero();
    
    // Parameters
    double l1 = 214.5/1000.0;
    double l2 = 179.14/1000.0;
    double l3 = 181.59/1000.0;
    double l4 = 4.75*25.4/1000.0;
    
    Vector6d t, f, r, d;
    t <<  M_PI/2, -M_PI/2,  M_PI/2,       0,       0,  M_PI/2;
    f <<  M_PI/2,  M_PI/2, -M_PI/2,  M_PI/2, -M_PI/2,       0;
    r <<       0,       0,       0,       0,       0,      l4;
    d <<       0,       0,     -l2,       0,     -l3,       0;

    limits <<
        H_Arm_Ctrl[side].joint[0].pos_min, H_Arm_Ctrl[side].joint[0].pos_max,
        H_Arm_Ctrl[side].joint[1].pos_min, H_Arm_Ctrl[side].joint[1].pos_max,
        H_Arm_Ctrl[side].joint[2].pos_min, H_Arm_Ctrl[side].joint[2].pos_max,
        H_Arm_Ctrl[side].joint[3].pos_min, H_Arm_Ctrl[side].joint[3].pos_max,
        H_Arm_Ctrl[side].joint[4].pos_min, H_Arm_Ctrl[side].joint[4].pos_max,
        H_Arm_Ctrl[side].joint[5].pos_min, H_Arm_Ctrl[side].joint[5].pos_max;

    
    if (side == RIGHT) {
        neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
        neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) = -l1;
        neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
        neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;

        offset(1) = limits(1,1);
    } else {
        // Transformation from Neck frame to left shoulder pitch frame
        neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
        neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) =  l1;
        neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
        neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;

        // Set offsets
        offset(1) = limits(1,0);
    }
     
    // Calculate forward kinematics
    B = neck;
    for (int i = 0; i < 6; i++) {
        DH2HG(T, t(i)+q(i)-offset(i), f(i), r(i), d(i));
        B = B*T;
    }
    B = B*endEffector;
    
}

bool Hubo_Control::huboArmIK(ArmVector &q, const Eigen::Isometry3d B, ArmVector qPrev, int side)
{
    Vector6d q6, qp6;
    for(int i=0; i<6; i++)
    {
        q6[i] = q[i];
        qp6[i] = qPrev[i];
    }

    bool valid = huboArmIK(q6, B, qp6, side);
    
    for(int i=0; i<6; i++)
        q[i] = q6[i];
    
    return valid;
}

bool Hubo_Control::huboArmIK(Vector6d &q, const Eigen::Isometry3d B, Vector6d qPrev, int side) {
    // Hand	
    Eigen::Isometry3d hand;
    hand(0,0) =  1; hand(0,1) =  0; hand(0,2) = 0; hand(0,3) =   0;
    hand(1,0) =  0; hand(1,1) =  0; hand(1,2) =-1; hand(1,3) =   0;
    hand(2,0) =  0; hand(2,1) =  1; hand(2,2) = 0; hand(2,3) =   0;
    hand(3,0) =  0; hand(3,1) =  0; hand(3,2) = 0; hand(3,3) =   1;
    
    return huboArmIK(q, B, qPrev, side, hand);
}

bool Hubo_Control::huboArmIK(ArmVector &q, const Eigen::Isometry3d B, ArmVector qPrev, int side, const Eigen::Isometry3d &endEffector)
{
    Vector6d q6, qp6;
    for(int i=0; i<6; i++)
    {
        q6[i] = q[i];
        qp6[i] = qPrev[i];
    }

    bool valid = huboArmIK(q6, B, qp6, side, endEffector);
    
    for(int i=0; i<6; i++)
        q[i] = q6[i];
    
    return valid;
}

bool Hubo_Control::huboArmIK(Vector6d &q, const Eigen::Isometry3d B, Vector6d qPrev, int side, const Eigen::Isometry3d &endEffector)
{
    Eigen::ArrayXXd qAll(6,8);
    
    // Declarations
    Eigen::Isometry3d neck, neckInv, endEffectorInv, BInv;
    Eigen::MatrixXd limits(6,2);
    Vector6d offset; offset.setZero();
    double nx, sx, ax, px;
    double ny, sy, ay, py;
    double nz, sz, az, pz;
    double q1, q2, q3, q4, q5, q6;
    double qP1, qP3;
    double qT;
    Eigen::Matrix<int, 8, 3> m;
    
    double S2, S4, S5, S6;
    double C2, C4, C5, C6;
    
    // Parameters
    double l1 = 214.5/1000.0;
    double l2 = 179.14/1000.0;
    double l3 = 181.59/1000.0;
    double l4 = 4.75*25.4/1000.0;
    

    limits <<
        H_Arm_Ctrl[side].joint[0].pos_min, H_Arm_Ctrl[side].joint[0].pos_max,
        H_Arm_Ctrl[side].joint[1].pos_min, H_Arm_Ctrl[side].joint[1].pos_max,
        H_Arm_Ctrl[side].joint[2].pos_min, H_Arm_Ctrl[side].joint[2].pos_max,
        H_Arm_Ctrl[side].joint[3].pos_min, H_Arm_Ctrl[side].joint[3].pos_max,
        H_Arm_Ctrl[side].joint[4].pos_min, H_Arm_Ctrl[side].joint[4].pos_max,
        H_Arm_Ctrl[side].joint[5].pos_min, H_Arm_Ctrl[side].joint[5].pos_max;

    if (side == RIGHT) {
        // Transformation from Neck frame to right shoulder pitch frame
        neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
        neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) = -l1;
        neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
        neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;

        // Set offsets
        offset(1) = limits(1,1); 
        
    } else {
        // Transformation from Neck frame to left shoulder pitch frame
        neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
        neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) =  l1;
        neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
        neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;

        // Set offsets
        offset(1) = limits(1,0); 
    }
    neckInv = neck.inverse();
    
    endEffectorInv = endEffector.inverse();        

    double zeroSize = .000001;
    
    // Variables
//    B = neckInv*B*endEffectorInv;
    BInv = (neckInv*B*endEffectorInv).inverse();
    
    nx = BInv(0,0); sx = BInv(0,1); ax = BInv(0,2); px = BInv(0,3);
    ny = BInv(1,0); sy = BInv(1,1); ay = BInv(1,2); py = BInv(1,3);
    nz = BInv(2,0); sz = BInv(2,1); az = BInv(2,2); pz = BInv(2,3);

    
    qP1 = qPrev(0); qP3 = qPrev(2);
    
    m <<
    1,  1,  1,
    1,  1, -1,
    1, -1,  1,
    1, -1, -1,
    -1,  1,  1,
    -1,  1, -1,
    -1, -1,  1,
    -1, -1, -1;
    
    // Calculate inverse kinematics
    for (int i = 0; i < 8; i++) {
        
        // Solve for q4
        C4 = max(min((2*l4*px - l2*l2 - l3*l3 + l4*l4 + px*px + py*py + pz*pz)/(2*l2*l3),1),-1);
        if (fabs(C4 - 1) < zeroSize) { // Case 1: q4 == 0
            
            // Set q4
            q4 = 0;
            
            // Set q3
            q3 = qP3;
            
            // Solve for q6
            S6 = max(min( py/(l2 + l3), 1),-1);
            C6 = max(min( -(l4 + px)/(l2 + l3), 1), -1);
            q6 = atan2(S6,C6);
            

            // Solve for q2
            S2 = max(min( C4*C6*ax - C4*S6*ay, 1),-1);
            if (fabs(S2 - 1) < zeroSize) {
                q2 = M_PI/2;
            } else if (fabs(S2 + 1) < zeroSize) {
                q2 = -M_PI/2;
            } else {
                double complex radical = 1-S2*S2;
                q2 = atan2(S2,m(i,2)*creal(csqrt(radical)));
            }
            
            // Solve for q5
            qT = atan2(-C6*ay - S6*ax,az);
            C2 = cos(q2);
            
            if (fabs(C2) < zeroSize) { // Case 3: q2 = pi/2 or -pi/2
                
                q1 = qP1;
                q3 = qP3;
                
                // Solve for q5
                if (S2 > 0) { // Case 3a: q2 = pi/2
                    qT = atan2(nz,-sz);
                    q5 = wrapToPi(q1 - q3 - qT);
                } else { // Case 3b: q2 = -pi/2
                    qT = atan2(-nz,sz);
                    q5 = wrapToPi(qT - q1 - q3);
                }
                
                
            } else {
                
                if (C2 < 0) {
                    qT = qT + M_PI;
                }
                q5 = wrapToPi(qT - q3);
                
                // Solve for q1
                q1 = atan2(S6*ny - C6*nx,C6*sx - S6*sy);
                if (C2 < 0) {
                    q1 = q1 + M_PI;
                }
                q1 = wrapToPi(q1);
            }
            
        } else {
            
            // Solve for q4
            double complex radical = 1-C4*C4;
            q4 = atan2(m(i,0)*creal(csqrt(radical)),C4);
            
            // Solve for q5
            S4 = sin(q4);
            S5 = pz/(S4*l2);
            if (fabs(S5 - 1) < zeroSize) {
                q5 = M_PI/2;
            } else if (fabs(S5 + 1) < zeroSize) {
                q5 = -M_PI/2;
            } else {
                radical = 1-S5*S5;
                q5 = atan2(S5,m(i,1)*creal(csqrt(radical)));
            }
            
            // Solve for q6
            C5 = cos(q5);
            S6 =max(min( (C5*S4*l2 + (py*(l3 + C4*l2 - (C5*S4*l2*py)/(l4 + px)))/(l4 + px + py*py/(l4 + px)))/(l4 + px), 1),-1);
            C6 = max(min( -(l3 + C4*l2 - (C5*S4*l2*py)/(l4 + px))/(l4 + px + py*py/(l4 + px)), 1),-1);
            q6 = atan2(S6,C6);
            
            // Solve for q2
            S2 = max(min(ax*(C4*C6 - C5*S4*S6) - ay*(C4*S6 + C5*C6*S4) - S4*S5*az,1),-1);
            if (fabs(S2 - 1) < zeroSize) {
                q2 = M_PI/2;
            } else if (fabs(S2 + 1) < zeroSize) {
                q2 = -M_PI/2;
            } else {
                radical = 1-S2*S2;
                q2 = atan2(S2,m(i,2)*creal(csqrt(radical)));
            }
            
            // Solve for q3
            C2 = cos(q2);
            
            if (fabs(C2) < zeroSize) { // Case 2: q2 = pi/2 or -pi/2
                
                q3 = qP3;
                // Solve for q1
                if (S2 > 0) { // Case 2a: q2 = pi/2
                    qT = atan2(S6*sy - C6*sx,S6*ny - C6*nx);
                    if (S4 < 0) {
                        qT = qT + M_PI;
                    }
                    q1 = wrapToPi(qT + q3);
                } else { // Case 2b: q2 = -pi/2
                    qT = atan2(S6*sy - C6*sx,S6*ny - C6*nx);
                    if (S4 < 0) {
                        qT = qT + M_PI;
                    }
                    q1 = wrapToPi(qT - q3);
                }
                
            } else {
                q3 = atan2(S4*S6*ay - C4*S5*az - C6*S4*ax - C4*C5*C6*ay - C4*C5*S6*ax,C5*az - C6*S5*ay - S5*S6*ax);
                if (C2 < 0) {
                    q3 = q3 - M_PI;
                }
                q3 = wrapToPi(q3);
                
                // Solve for q1
                q1 = atan2(C4*S6*ny - C4*C6*nx + S4*S5*nz + C5*C6*S4*ny + C5*S4*S6*nx,C4*C6*sx - C4*S6*sy - S4*S5*sz - C5*C6*S4*sy - C5*S4*S6*sx);
                if (C2 < 0) {
                    q1 = q1 + M_PI;
                }
                q1 = wrapToPi(q1);
            }
        }
        
        qAll(0,i) = q1;
        qAll(1,i) = q2;
        qAll(2,i) = q3;
        qAll(3,i) = q4;
        qAll(4,i) = q5;
        qAll(5,i) = q6;

    }
    // Set to offsets
    for( int j=0; j<8; j++) {
        for (int i = 0; i < 6; i++) {
            if (side==RIGHT) {
                qAll(i,j) = wrapToPi(qAll(i,j) + offset(i));
            } else {
                qAll(i,j) = wrapToPi(qAll(i,j) + offset(i));
            } 
        }
    }
    // TODO: Find best solution using better method

    Eigen::ArrayXd qDiff(6,1); qDiff.setZero();
    Eigen::ArrayXd qDiffSum(8,1);
    bool withinLim[8];
    int minInd;

    // if any joint solution is infintesimal, set it to zero
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if(qAll(j,i) < zeroSize && qAll(j,i) > -zeroSize)
                qAll(j,i) = 0.0;

    // Initialize withinLim to all trues for all eight solutions
    for(int i=0; i<8; i++)
        withinLim[i] = true;

    // Check each set of solutions to see if any are outside the limits
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if( limits(j,0) > qAll(j,i) || qAll(j,i) > limits(j,1) )
                withinLim[i] = false;
    
    // Initialze anyWithin boolean array to all trues
    bool anyWithin=false;
    for(int i=0; i<8; i++)
        if( withinLim[i] )
            anyWithin = true;

    // If any solution has all joints within the limits...
    if(anyWithin)
    {
        outOfWorkspace = false;
        // for each solution...
        for (int i = 0; i < 8; i++) {
            // if all the joints of solution i are within the limits...
            if( withinLim[i] )
            {
                // calculate the differences between solution angles, j, and previous angles
                for (int j=0; j < 6; j++)
                    qDiff(j) = wrapToPi(qAll(j,i) - qPrev(j));
                // sum the absolute values of the differences to get total difference
                qDiffSum(i) = qDiff.abs().sum();
            }
            // if the solution doesn't have all the joints within the limits...
            else
                // set the difference for that solution to infinity
                qDiffSum(i) = std::numeric_limits<double>::infinity();
        }
        // and take the solution closest to previous solution
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }
    // if no solution has all the joints within the limits...
    else
    {
        outOfWorkspace = true;
        // then for each solution...
        for( int i=0; i<8; i++)
        {
            // create a 6d vector of angles of solution i
            Vector6d qtemp = qAll.col(i).matrix();
            // take the min of the angles and the joint upper limits
            qtemp = qtemp.cwiseMin(limits.col(1));
            // then take the max of those angles and the joint lower limits
            qtemp = qtemp.cwiseMax(limits.col(0));
            // create an Isometry3d 4x4 matrix for the temp pose
            Eigen::Isometry3d Btemp;
            // find the pose associated with the temp angles
            huboArmFK( Btemp, qtemp, side );
            // calculate the distance from previous pose to temp pose locations
            qDiffSum(i) = (Btemp.translation() - B.translation()).norm();
        }
        // find the solution that's closest the previous position
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }
    // set the final joint angles to the solution closest to the previous solution
    for( int i=0; i<6; i++ )
        q(i) = max( min( q(i), limits(i,1)), limits(i,0) );
    
    //q = q.cwiseMin(limits.col(1)); //TODO: Put these back
    //q = q.cwiseMax(limits.col(0));
    return outOfWorkspace;
}


void Hubo_Control::huboLegFK(Eigen::Isometry3d &B, LegVector &q, int side)
{
    Vector6d q6;
    for(int i=0; i<6; i++)
        q6[i] = q[i];
    
    huboLegFK(B, q6, side);
    
    for(int i=0; i<6; i++)
        q[i] = q6[i];
}

void Hubo_Control::huboLegFK(Eigen::Isometry3d &B, Vector6d &q, int side) {
    // Declarations
    Eigen::Isometry3d neck, waist, T, foot;
    Eigen::MatrixXd limits(6,2);
    Vector6d offset; offset.setZero();
    
    // Parameters
    double l1 = (79.5+107)/1000.0;
    double l2 = 88.43/1000.0;
    double l3 = (289.47-107)/1000.0;
    double l4 = 300.03/1000.0;
    double l5 = 300.38/1000.0;
    double l6 = 94.97/1000.0;

    // Denavit-Hartenberg parameters 
    Vector6d t, f, r, d;
    t <<       0, -M_PI/2,       0,       0,       0,       0;
    f <<  M_PI/2, -M_PI/2,       0,       0,  M_PI/2,       0;
    r <<       0,       0,      l4,      l5,       0,      l6;
    d <<       0,       0,       0,       0,       0,       0;
    
    // Transformation from Neck frame to Waist frame
    neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
    neck(1,0) = 0; neck(1,1) =  1; neck(1,2) = 0; neck(1,3) =   0;
    neck(2,0) = 0; neck(2,1) =  0; neck(2,2) = 1; neck(2,3) = -l1;
    neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
    
    limits <<
        H_Leg_Ctrl[side].joint[0].pos_min, H_Leg_Ctrl[side].joint[0].pos_max,
        H_Leg_Ctrl[side].joint[1].pos_min, H_Leg_Ctrl[side].joint[1].pos_max,
        H_Leg_Ctrl[side].joint[2].pos_min, H_Leg_Ctrl[side].joint[2].pos_max,
        H_Leg_Ctrl[side].joint[3].pos_min, H_Leg_Ctrl[side].joint[3].pos_max,
        H_Leg_Ctrl[side].joint[4].pos_min, H_Leg_Ctrl[side].joint[4].pos_max,
        H_Leg_Ctrl[side].joint[5].pos_min, H_Leg_Ctrl[side].joint[5].pos_max;

    if (side == RIGHT) {
        // Transformation from Waist frame to right hip yaw frame
        waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
        waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) = -l2;
        waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
        waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
/*        
                limits <<
                -1.80,   0.0,
                -0.58,   0.0,
                -1.30,   1.30,
                0.0,     2.50,
                -1.26,   1.80,
                -0.23,   0.31;
*/        
        // Set offsets
        //        offset(1) = limits(1,1);
        
    } else {
        // Transformation from Waist frame to left hip yaw frame
        waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
        waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) =  l2;
        waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
        waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
/*        
                limits <<
                0.0,     1.80,
                0.0,     0.58,
                -1.30,   1.30,
                0.0,     2.50,
                -1.26,   1.80,
                -0.31,   0.23;
*/        
        // Set offsets
        //        offset(1) = limits(1,0);
    }

    // Rotation of -90 about y to make x forward, y left, z up
//    foot(0,0) = 0; foot(0,1) =  0; foot(0,2) =1; foot(0,3) = 0;
 //   foot(1,0) = 0; foot(1,1) =  1; foot(1,2) = 0; foot(1,3) = 0;
  //  foot(2,0) = -1; foot(2,1) =  0; foot(2,2) = 0; foot(2,3) = 0;
   // foot(3,0) = 0; foot(3,1) =  0; foot(3,2) = 0; foot(3,3) = 1;   

    // Calculate forward kinematics
    B = waist*neck;
    for (int i = 0; i < 6; i++) {
        DH2HG(T, t(i)+q(i)+offset(i), f(i), r(i), d(i));
        B = B*T;//*foot;
    }
}

bool Hubo_Control::huboLegIK(LegVector &q, const Eigen::Isometry3d B, LegVector qPrev, int side)
{
    Vector6d q6, qp6;
    for(int i=0; i<6; i++)
    {
        q6[i] = q[i];
        qp6[i] = qPrev[i];
    }
    
    bool valid = huboLegIK(q6, B, qp6, side);
    
    for(int i=0; i<6; i++)
        q[i] = q6[i];
    
    return valid;
}

bool Hubo_Control::huboLegIK(Vector6d &q, const Eigen::Isometry3d B, Vector6d qPrev, int side) {
    Eigen::ArrayXXd qAll(6,8);
    
    // Declarations
    Eigen::Isometry3d neck, neckInv, waist, waistInv, BInv, foot, footInv;
    Eigen::MatrixXd limits(6,2);
    Vector6d offset; offset.setZero();
    double nx, sx, ax, px;
    double ny, sy, ay, py;
    double nz, sz, az, pz;
    double q1, q2, q3, q4, q5, q6;
    double C45, psi, q345;
    Eigen::Matrix<int, 8, 3> m;
    
    double S2, S4, S6;
    double C2, C4, C5, C6;
    
    // Parameters
    double l1 = (79.5+107)/1000.0;
    double l2 = 88.43/1000.0;
    double l3 = (289.47-107)/1000.0;
    double l4 = 300.03/1000.0;
    double l5 = 300.38/1000.0;
    double l6 = 94.97/1000.0;

    // Transformation from Neck frame to Waist frame
    neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
    neck(1,0) = 0; neck(1,1) =  1; neck(1,2) = 0; neck(1,3) =   0;
    neck(2,0) = 0; neck(2,1) =  0; neck(2,2) = 1; neck(2,3) = -l1;
    neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;

    limits <<
        H_Leg_Ctrl[side].joint[0].pos_min, H_Leg_Ctrl[side].joint[0].pos_max,
        H_Leg_Ctrl[side].joint[1].pos_min, H_Leg_Ctrl[side].joint[1].pos_max,
        H_Leg_Ctrl[side].joint[2].pos_min, H_Leg_Ctrl[side].joint[2].pos_max,
        H_Leg_Ctrl[side].joint[3].pos_min, H_Leg_Ctrl[side].joint[3].pos_max,
        H_Leg_Ctrl[side].joint[4].pos_min, H_Leg_Ctrl[side].joint[4].pos_max,
        H_Leg_Ctrl[side].joint[5].pos_min, H_Leg_Ctrl[side].joint[5].pos_max;
    
    if (side == RIGHT) {
        // Transformation from Waist frame to right hip yaw frame
        waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
        waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) = -l2;
        waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
        waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
/*        
                limits <<
                -1.80,   0.0,
                -0.58,   0.0,
                -1.30,   1.30,
                0.0,     2.50,
                -1.26,   1.80,
                -0.23,   0.31;
*/        
        // Set offsets
        //        offset(1) = limits(1,1);
        
    } else {
        // Transformation from Waist frame to left hip yaw frame
        waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
        waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) =  l2;
        waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
        waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;
/*        
                limits <<
                0.0,     1.80,
                0.0,     0.58,
                -1.30,   1.30,
                0.0,     2.50,
                -1.26,   1.80,
                -0.31,   0.23;
*/        // Set offsets
        //        offset(1) = limits(1,0);
    }

    // Rotation of -90 about y to make x forward, y left, z up
//    foot(0,0) = 0; foot(0,1) =  0; foot(0,2) =1; foot(0,3) = 0;
//    foot(1,0) = 0; foot(1,1) =  1; foot(1,2) = 0; foot(1,3) = 0;
//    foot(2,0) = -1; foot(2,1) =  0; foot(2,2) = 0; foot(2,3) = 0;
//    foot(3,0) = 0; foot(3,1) =  0; foot(3,2) = 0; foot(3,3) = 1;   

    neckInv = neck.inverse();
    waistInv = waist.inverse();
//    footInv = foot.inverse();
    
    // Variables
    BInv = (neckInv*waistInv*B).inverse();
    
    nx = BInv(0,0); sx = BInv(0,1); ax = BInv(0,2); px = BInv(0,3);
    ny = BInv(1,0); sy = BInv(1,1); ay = BInv(1,2); py = BInv(1,3);
    nz = BInv(2,0); sz = BInv(2,1); az = BInv(2,2); pz = BInv(2,3);

    m <<
    1,  1,  1,
    1,  1, -1,
    1, -1,  1,
    1, -1, -1,
    -1,  1,  1,
    -1,  1, -1,
    -1, -1,  1,
    -1, -1, -1;
    
    for (int i = 0; i < 8; i++) {
        C4 = ((l6 + px)*(l6 + px) - l4*l4 - l5*l5 + py*py + pz*pz)/(2*l4*l5);
        double complex radical = 1-C4*C4;
        q4 = atan2(m(i,0)*creal(csqrt(radical)),C4);

        S4 = sin(q4);
        psi = atan2(S4*l4, C4*l4+l5);
        radical = ((px+l6)*(px+l6)+(py*py));
        q5 = wrapToPi(atan2(-pz, m(i,1)*creal(csqrt(radical)))-psi);

        q6 = atan2(py, -px-l6);
        C45 = cos(q4+q5);
        C5 = cos(q5);
        if (C45*l4 + C5*l5 < 0)
        {
            q6 = wrapToPi(q6 + M_PI);
        }

        S6 = sin(q6);
        C6 = cos(q6);

        S2 = C6*ay + S6*ax;
        radical = 1-S2*S2;
        q2 = atan2(S2,m(i,2)*creal(csqrt(radical)));

        q1 = atan2(C6*sy + S6*sx,C6*ny + S6*nx);
        C2 = cos(q2);
        if (C2 < 0) {
            q1 = wrapToPi(q1 + M_PI);
        }

        q345 = atan2(-az/C2,-(C6*ax - S6*ay)/C2);
        q3 = wrapToPi(q345-q4-q5);

        qAll(0,i) = q1;
        qAll(1,i) = q2;
        qAll(2,i) = q3;
        qAll(3,i) = q4;
        qAll(4,i) = q5;
        qAll(5,i) = q6;
    }

    // Set to offsets
    for (int i = 0; i < 6; i++) {
        if (side==RIGHT) {
            q(i) = wrapToPi(q(i) + offset(i));
        } else {
            q(i) = wrapToPi(q(i) + offset(i));
        }
    }

    // Find best solution
    Eigen::ArrayXd qDiff(6,1); qDiff.setZero();
    Eigen::ArrayXd qDiffSum(8,1);
    bool withinLim[8];
    int minInd;
    double zeroSize = 0.000001;

    // if any joint solution is infintesimal, set it to zero
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if(qAll(j,i) < zeroSize && qAll(j,i) > -zeroSize)
                qAll(j,i) = 0.0;

    // Initialize withinLim to all trues for all eight solutions
    for(int i=0; i<8; i++)
        withinLim[i] = true;

    // Check each set of solutions to see if any are outside the limits
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if( limits(j,0) > qAll(j,i) || qAll(j,i) > limits(j,1) )
                withinLim[i] = false;

    // Initialze anyWithin boolean array to all trues
    bool anyWithin=false;
    for(int i=0; i<8; i++)
        if( withinLim[i] )
            anyWithin = true;

    // If any solution has all joints within the limits...
    if(anyWithin)
    {
        outOfWorkspace = false;
        // for each solution...
        for (int i = 0; i < 8; i++)
        {
            // if all the joints of solution i are within the limits...
            if( withinLim[i] )
            {
                // calculate the differences between solution angles, j, and previous angles
                for (int j=0; j < 6; j++)
                {
                    qDiff(j) = wrapToPi(qAll(j,i) - qPrev(j));
                }
                // sum the absolute values of the differences to get total difference
                qDiffSum(i) = qDiff.abs().sum();
            }
            // if the solution doesn't have all the joints within the limits...
            else
                // set the difference for that solution to infinity
                qDiffSum(i) = std::numeric_limits<double>::infinity();
        }
        // and take the solution closest to previous solution
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }

    // if no solution has all the joints within the limits...
    else
    {
        outOfWorkspace = true;
        // then for each solution...
        for(int i=0; i<8; i++)
        {
            // create a 6d vector of angles of solution i
            Vector6d qtemp = qAll.col(i).matrix();
            // take the min of the angles and the joint upper limits
            qtemp = qtemp.cwiseMin(limits.col(1));
            // then take the max of those angles and the joint lower limits
            qtemp = qtemp.cwiseMax(limits.col(0));
            // create an Isometry3d 4x4 matrix for the temp pose
            Eigen::Isometry3d Btemp;
            // find the pose associated with the temp angles
            huboLegFK( Btemp, qtemp, side );
            // calculate the distance from previous pose to temp pose locations
            qDiffSum(i) = (Btemp.translation() - B.translation()).norm();
        }
        // find the solution that's closest the previous position
        qDiffSum.minCoeff(&minInd);
        q = qAll.col(minInd);
    }
    // set the final joint angles to the solution closest to the previous solution
    for( int i=0; i<6; i++)
        q(i) = max( min( q(i), limits(i,1)), limits(i,0) ); 

    return outOfWorkspace;
}

ctrl_flag_t Hubo_Control::footVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity, int side )
{
    Eigen::Vector3d rotVel; rotVel.setZero();
    return footVelocityIK( qdot, velocity, rotVel, side );
}

ctrl_flag_t Hubo_Control::footVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity,
        Eigen::Vector3d &angularVel, int side )
{
    Eigen::Vector3d hipVelocity, hipRotVel, r;

    double hp, ap, ar, kn, yaw;
    
    if( side == RIGHT)
    {
        hp = getJointAngleState(RHP);
        ap = getJointAngleState(RAP);
        ar = getJointAngleState(RAR);
        kn = getJointAngleState(RKN);
        yaw = getJointAngleState(RHY);
    }
    else
    {
        hp = getJointAngleState(LHP);
        ap = getJointAngleState(LAP);
        ar = getJointAngleState(LAR);
        kn = getJointAngleState(LKN);
        yaw = getJointAngleState(LHY);
    }
    r << shinLength*(sin(hp)-sin(ap)), shinLength*(cos(hp)+cos(ap))*sin(ar), shinLength*(cos(hp)+cos(ap))*cos(ar);

    Eigen::AngleAxisd T(yaw, Vector3d(0,0,1));
    r = T*r;

    hipRotVel = -angularVel;
    hipVelocity = -velocity - angularVel.cross(r);

    return hipVelocityIK( qdot, hipVelocity, hipRotVel, side );
    
}

ctrl_flag_t Hubo_Control::hipVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity, int side ) 
{
    Eigen::Vector3d rotVel; rotVel.setZero();
    return hipVelocityIK( qdot, velocity, rotVel, side );
}

ctrl_flag_t Hubo_Control::hipVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity, const LegVector &q ) 
{
    Eigen::Vector3d rotVel; rotVel.setZero();
    return hipVelocityIK( qdot, velocity, rotVel, q );
}

ctrl_flag_t Hubo_Control::hipVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity,
        Eigen::Vector3d &angularVel, int side )
{
    LegVector q;
    if( side!=RIGHT && side!=LEFT )
        side = RIGHT;
    getLegAngleStates( side, q );
    hipVelocityIK( qdot, velocity, angularVel, q );
}

ctrl_flag_t Hubo_Control::hipVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity,
        Eigen::Vector3d &angularVel, const LegVector &q )
{
    ctrl_flag_t flag = SUCCESS;

    Eigen::Vector3d vel;

    qdot.setZero();

    double hp, ap, ar, kn;
    
    vel(0) = -(  velocity(0)*cos(q(HY)) + velocity(1)*sin(q(HY)) );
    vel(1) = -( -velocity(0)*sin(q(HY)) + velocity(1)*cos(q(HY)) );
    vel(2) =  velocity(2);

    hp = q(HP);
    ap = q(AP);
    ar = q(AR);
    kn = q(KN);

    double L = 2*shinLength;


    if( kn <= kneeSingularityDanger || fabs(-hp-ap) <= kneeSingularityDanger )
    {
        qdot(HP) = -fabs(kneeSingularitySpeed/2);
        qdot(KN) =  fabs(kneeSingularitySpeed);
        qdot(AP) = -fabs(kneeSingularitySpeed/2);
        return IK_DANGER;
    }


    if( kn <= kneeSingularityThreshold || fabs(-hp-ap) <= kneeSingularityThreshold )
    {
        
        Eigen::Vector3d lhat;

        lhat <<  sin(ap)-sin(hp), -(cos(ap)+cos(hp))*sin(ar), (cos(ap)+cos(hp))*cos(ar);
        lhat.normalize();

        double lv = vel.dot(lhat);

        if( lv > 0 )
        {
            vel = vel - lv*lhat;
            flag = IK_DANGER;
        }
        
    }

    qdot(HP) = -2.0/(L*sin(hp+ap))*( vel(0)*sin(ap) - vel(1)*sin(ar)*cos(ap) + vel(2)*cos(ar)*cos(ap) );
    qdot(AR) = -2.0*( vel(1)*cos(ar) + vel(2)*sin(ar) )/( L*( cos(ap) + cos(hp) ) );
    qdot(AP) =  2.0*vel(0)/(L*cos(ap)) + qdot(HP)*cos(hp)/cos(ap);
    qdot(HR) =  -qdot(AR);
    qdot(KN) = -qdot(HP) - qdot(AP);

    qdot(HR) -= angularVel(0);
    qdot(HP) -= angularVel(1);
    qdot(HY) -= angularVel(2);

    return flag;

}

ctrl_flag_t Hubo_Control::calibrateJoint( int joint, double offset )
{
    if( joint < HUBO_JOINT_COUNT )
        jointAngleCalibration[joint] = getJointAngleState(joint) + offset;
    else
        return JOINT_OOB;

    return SUCCESS;
}

void Hubo_Control::calibrateAnkleForces()
{
    afc[RIGHT] = (getRightFootFz()+getLeftFootFz())/2.0 - getRightFootFz();
    afc[LEFT]  = (getRightFootFz()+getLeftFootFz())/2.0 - getLeftFootFz();
}





void Hubo_Control::HuboDrillFK(Eigen::Isometry3d &B, ArmVector &q) {
    Eigen::Isometry3d drill;
    
    double ld = 7*25.4/1000.0;
    double ad = M_PI/4;
    
    drill(0,0) = cos(ad);  drill(0,1) = 0; drill(0,2) = sin(ad); drill(0,3) = ld*cos(ad);
    drill(1,0) = 0;        drill(1,1) = 1; drill(1,2) = 0;       drill(1,3) = 0;
    drill(2,0) = -sin(ad); drill(2,1) = 0; drill(2,2) = cos(ad); drill(2,3) = -ld*sin(ad);
    drill(3,0) = 0;        drill(3,1) = 0; drill(3,2) = 0;       drill(3,3) = 1;
    
    huboArmFK(B, q, RIGHT, drill);
}

void Hubo_Control::HuboDrillIK(ArmVector &q, double y) {
    ArmVector qPrev; qPrev.setZero();
    Eigen::Isometry3d drill, B;
    
    double l1 = 214.5/1000;
    double l2 = 179.14/1000;
    double l3 = 181.59/1000;
    double l4 = 4.75*25.4/1000;
    double ld = 7*25.4/1000.0;
    double ad = M_PI/4;
    
    drill(0,0) = cos(ad);  drill(0,1) = 0; drill(0,2) = sin(ad); drill(0,3) = ld*cos(ad);
    drill(1,0) = 0;        drill(1,1) = 1; drill(1,2) = 0;       drill(1,3) = 0;
    drill(2,0) = -sin(ad); drill(2,1) = 0; drill(2,2) = cos(ad); drill(2,3) = -ld*sin(ad);
    drill(3,0) = 0;        drill(3,1) = 0; drill(3,2) = 0;       drill(3,3) = 1;
    
    double pyLimits[2] = {-0.2050, 0.1850};
    
    double px = 0.0925;
    double py = min(max(y,pyLimits[0]),pyLimits[1]);
    double pz = 0.04;
    
    double px0 = (l3+l4)*sin(M_PI/4)+ld;
    double py0 = -l1;
    double pz0 = (l3+l4)*cos(M_PI/4)-l2;
    
    B(0,0) = 1; B(0,1) = 0; B(0,2) = 0; B(0,3) = px0+px;
    B(1,0) = 0; B(1,1) = 1; B(1,2) = 0; B(1,3) = py0+py;
    B(2,0) = 0; B(2,1) = 0; B(2,2) = 1; B(2,3) = pz0+pz;
    B(3,0) = 0; B(3,1) = 0; B(3,2) = 0; B(3,3) = 1;
    
    huboArmIK(q, B, qPrev, RIGHT, drill);
    
}

void Hubo_Control::storeArmDefaults(int side)
{
    if( side == LEFT || side == RIGHT )
        memcpy( &(H_Arm_Ctrl_Defaults[side]), &(H_Arm_Ctrl[side]), sizeof(H_Arm_Ctrl_Defaults[side]) );
    else
        fprintf(stderr, "Invalid state parameter for Storing Arm Defaults! %d\n", side);
}

void Hubo_Control::storeRightArmDefaults()
{ storeArmDefaults(RIGHT); }

void Hubo_Control::storeLeftArmDefaults()
{ storeArmDefaults(LEFT); }


void Hubo_Control::storeLegDefaults(int side)
{
    if( side == LEFT || side == RIGHT )
        memcpy( &(H_Leg_Ctrl_Defaults[side]), &(H_Leg_Ctrl[side]), sizeof(H_Leg_Ctrl_Defaults[side]) );
    else
        fprintf(stderr, "Invalid side parameter for Storing Leg Defaults! %d\n", side);
}

void Hubo_Control::storeRightLegDefaults()
{ storeLegDefaults(RIGHT); }

void Hubo_Control::storeLeftLegDefaults()
{ storeLegDefaults(LEFT); }


void Hubo_Control::storeBodyDefaults()
{
    memcpy( &H_Bod_Ctrl_Defaults, &H_Bod_Ctrl, sizeof(H_Bod_Ctrl_Defaults) );
}


void Hubo_Control::storeNeckDefaults()
{
    memcpy( &H_Nck_Ctrl_Defaults, &H_Nck_Ctrl, sizeof(H_Nck_Ctrl_Defaults) );
}


void Hubo_Control::storeAllDefaults()
{
    storeLeftArmDefaults();
    storeRightArmDefaults();
    storeLeftLegDefaults();
    storeRightLegDefaults();
    storeBodyDefaults();
    storeNeckDefaults();
}


void Hubo_Control::resetArmDefaults(int side, bool send)
{
    if( side == LEFT || side == RIGHT )
    {
        for(int i=0; i<H_Arm_Ctrl_Defaults[side].count; i++)
            H_Arm_Ctrl_Defaults[side].joint[i].position = H_Arm_Ctrl[side].joint[i].position;

        memcpy( &(H_Arm_Ctrl[side]), &(H_Arm_Ctrl_Defaults[side]), sizeof(H_Arm_Ctrl[side]) );
    }
    else
        fprintf(stderr, "Invalid side parameter for Resetting Arm Defaults! %d\n", side);

    if(send)
        sendControls();
}

void Hubo_Control::resetRightArmDefaults(bool send)
{ resetArmDefaults(RIGHT, send); }

void Hubo_Control::resetLeftArmDefaults(bool send)
{ resetArmDefaults(LEFT, send); }


void Hubo_Control::resetLegDefaults(int side, bool send)
{
    if( side == LEFT || side == RIGHT )
    {
        for(int i=0; i<H_Leg_Ctrl_Defaults[side].count; i++)
            H_Leg_Ctrl_Defaults[side].joint[i].position = H_Leg_Ctrl[side].joint[i].position;

        memcpy( &(H_Leg_Ctrl[side]), &(H_Leg_Ctrl_Defaults[side]), sizeof(H_Leg_Ctrl[side]) );
    }
    else
        fprintf(stderr, "Invalid side parameter for Resetting Leg Defaults! %d\n", side);

    if(send)
        sendControls();
}

void Hubo_Control::resetRightLegDefaults(bool send)
{ resetLegDefaults(RIGHT, send); }

void Hubo_Control::resetLeftLegDefaults(bool send)
{ resetLegDefaults(LEFT, send); }



void Hubo_Control::resetBodyDefaults(bool send)
{
    for(int i=0; i<H_Bod_Ctrl_Defaults.count; i++)
        H_Bod_Ctrl_Defaults.joint[i].position = H_Bod_Ctrl.joint[i].position;

    memcpy( &H_Bod_Ctrl, &H_Bod_Ctrl_Defaults, sizeof(H_Bod_Ctrl) );

    if(send)
        sendControls();
}


void Hubo_Control::resetNeckDefaults(bool send)
{
    for(int i=0; i<H_Nck_Ctrl_Defaults.count; i++)
        H_Nck_Ctrl_Defaults.joint[i].position = H_Nck_Ctrl.joint[i].position;

    memcpy( &H_Nck_Ctrl, &H_Nck_Ctrl_Defaults, sizeof(H_Nck_Ctrl) );

    if(send)
        sendControls();
}

void Hubo_Control::resetAllDefaults(bool send)
{
    resetLeftArmDefaults();
    resetRightArmDefaults();
    resetLeftLegDefaults();
    resetRightLegDefaults();
    resetBodyDefaults();
    resetNeckDefaults(send);
}


