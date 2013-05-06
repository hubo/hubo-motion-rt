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
    ach_close( &chan_hubo_aux_ctrl );

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
    memset( &H_Param, 0, sizeof(H_Param) );

    memset( H_Arm_Ctrl,  0, 2*sizeof(H_Arm_Ctrl[0]) );
    memset( H_Leg_Ctrl,  0, 2*sizeof(H_Leg_Ctrl[0]) );
    memset( H_Fin_Ctrl,  0, 2*sizeof(H_Fin_Ctrl[0]) );
    memset( &H_Aux_Ctrl,  0, 2*sizeof(H_Aux_Ctrl) );

    memset( ctrlMap, 0, sizeof(ctrlMap[0])*HUBO_JOINT_COUNT );
    memset( localMap, 0, sizeof(localMap[0])*HUBO_JOINT_COUNT );

    memset( jointAngleCalibration, 0, sizeof(jointAngleCalibration[0])*HUBO_JOINT_COUNT );

    setJointParams( &H_Param, &H_State );

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
    
    r = ach_open( &chan_hubo_aux_ctrl, HUBO_CHAN_AUX_CTRL_NAME, NULL );
    assert( ACH_OK == r );
    
    r = ach_open( &chan_ctrl_state, CTRL_CHAN_STATE, NULL );
    assert( ACH_OK == r );

    size_t fs;

    ach_get( &chan_hubo_ref, &H_Ref, sizeof(H_Ref), &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_state, &H_State, sizeof(H_State), &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_arm_ctrl_right, &H_Arm_Ctrl[RIGHT], sizeof(H_Arm_Ctrl[RIGHT]), &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_arm_ctrl_left,  &H_Arm_Ctrl[LEFT],  sizeof(H_Arm_Ctrl[LEFT]),  &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_leg_ctrl_right, &H_Leg_Ctrl[RIGHT], sizeof(H_Leg_Ctrl[RIGHT]), &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_leg_ctrl_left,  &H_Leg_Ctrl[LEFT],  sizeof(H_Leg_Ctrl[LEFT]),  &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_fin_ctrl_right, &H_Fin_Ctrl[RIGHT], sizeof(H_Fin_Ctrl[RIGHT]), &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_fin_ctrl_left,  &H_Fin_Ctrl[LEFT],  sizeof(H_Fin_Ctrl[LEFT]),  &fs, NULL, ACH_O_LAST );
    ach_get( &chan_hubo_aux_ctrl, &H_Aux_Ctrl, sizeof(H_Aux_Ctrl), &fs, NULL, ACH_O_LAST );


    for(int i=0; i<ARM_JOINT_COUNT; i++)
    {
        armjoints[LEFT][i] = leftarmjoints[i];
        armjoints[RIGHT][i] = rightarmjoints[i];
        ctrlMap[ armjoints[RIGHT][i] ] = CtrlRA;
        ctrlMap[ armjoints[LEFT][i]  ] = CtrlLA;
        localMap[ armjoints[RIGHT][i] ] = i;
        localMap[ armjoints[LEFT][i]  ] = i;
    }

    for(int i=0; i<LEG_JOINT_COUNT; i++)
    {
        legjoints[RIGHT][i] = rightlegjoints[i];
        legjoints[LEFT][i]  = leftlegjoints[i];
        ctrlMap[ legjoints[RIGHT][i] ] = CtrlRL;
        ctrlMap[ legjoints[LEFT][i] ]  = CtrlLL;
        localMap[ legjoints[RIGHT][i] ] = i;
        localMap[ legjoints[LEFT][i]  ] = i;
    }

    for(int i=0; i<FIN_JOINT_COUNT; i++)
    {
        finjoints[RIGHT][i] = rightfinjoints[i];
        finjoints[LEFT][i]  = leftfinjoints[i];
        ctrlMap[ finjoints[RIGHT][i] ] = CtrlRF;
        ctrlMap[ finjoints[LEFT][i] ]  = CtrlLF;
        localMap[ finjoints[RIGHT][i] ] = i;
        localMap[ finjoints[LEFT][i]  ] = i;
    }

    for(int i=0; i<AUX_JOINT_COUNT; i++)
    {
        ctrlMap[ auxjoints[i] ] = CtrlAX;
        localMap[ auxjoints[i] ] = i;
    }
    
}



double Hubo_Control::getTime() { return H_State.time; }

ctrl_flag_t Hubo_Control::update(bool stateWait, bool printError)
{
    int r1, r2;
    size_t fs;

    if( !stateWait )
    {
        r2 = ach_get( &chan_hubo_state, &H_State, sizeof(H_State), &fs, NULL, ACH_O_LAST );
        if( ACH_OK != r2 && printError )
            fprintf( stdout, "Ach report -- State Channel: %s at time=%f",
                ach_result_to_string((ach_status_t)r2), getTime() );
    }
    else
    {
        r2 = ach_get( &chan_hubo_state, &H_State, sizeof(H_State), &fs, NULL, ACH_O_WAIT );
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
    if(ctrlOn[CtrlAX])
        ach_put( &chan_hubo_aux_ctrl, &H_Aux_Ctrl, sizeof(H_Aux_Ctrl) ); 
/*    if( r != ACH_OK ) fprintf(stderr, "Problem sending control commands: (%d) %s\n",
                                r, ach_result_to_string((ach_status_t)r));
*/ //TODO: Maybe generate error messages or something
}
void Hubo_Control::sendCommands()
{
    int r = ach_put( &chan_hubo_board_cmd, &H_Cmd, sizeof(H_Cmd) );
    if( r != ACH_OK ) fprintf(stderr, "Problem sending board commands: (%d) %s\n",
                                r, ach_result_to_string((ach_status_t)r));
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
            case CtrlAX:
                H_Aux_Ctrl.joint[localMap[joint]].mode = CTRL_RESET;
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
            case CtrlAX: // Right Fingers
                H_Aux_Ctrl.joint[localMap[joint]].position = radians;
                H_Aux_Ctrl.joint[localMap[joint]].mode = CTRL_POS;
                H_Aux_Ctrl.active=1; ctrlOn[CtrlAX] = true; break;
                
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
//                if( H_Arm_Ctrl[RIGHT].joint[localMap[joint]].mode == CTRL_POS )
                    H_Arm_Ctrl[RIGHT].joint[localMap[joint]].speed = speed;
//                else
//                    return WRONG_MODE;
                break;
            case CtrlLA: // Left Arm
//                if( H_Arm_Ctrl[LEFT].joint[localMap[joint]].mode == CTRL_POS )
                    H_Arm_Ctrl[LEFT].joint[localMap[joint]].speed = speed;
//                else
//                    return WRONG_MODE;
                break;
            case CtrlRL: // Right Leg
//                if( H_Leg_Ctrl[RIGHT].joint[localMap[joint]].mode == CTRL_POS )
                    H_Leg_Ctrl[RIGHT].joint[localMap[joint]].speed = speed;
//                else
//                    return WRONG_MODE;
                break;
            case CtrlLL: // Left Leg
//                if( H_Leg_Ctrl[LEFT].joint[localMap[joint]].mode == CTRL_POS )
                    H_Leg_Ctrl[LEFT].joint[localMap[joint]].speed = speed;
//                else
//                    return WRONG_MODE;
                break;
            case CtrlRF: // Right Fingers
//                if( H_Fin_Ctrl[RIGHT].joint[localMap[joint]].mode == CTRL_POS )
                    H_Fin_Ctrl[RIGHT].joint[localMap[joint]].speed = speed;
//                else
//                    return WRONG_MODE;
                break;
            case CtrlLF: // Left Fingers
//                if( H_Fin_Ctrl[LEFT].joint[localMap[joint]].mode == CTRL_POS )
                    H_Fin_Ctrl[LEFT].joint[localMap[joint]].speed = speed;
//                else
//                    return WRONG_MODE;
                break;
            case CtrlAX: // Aux
//                if( H_Aux_Ctrl.joint[localMap[joint]].mode == CTRL_POS )
                    H_Aux_Ctrl.joint[localMap[joint]].speed = speed;
//                else
//                    return WRONG_MODE;
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
            case CtrlAX: // Aux
                H_Aux_Ctrl.joint[localMap[joint]].mode = CTRL_VEL;
                H_Aux_Ctrl.joint[localMap[joint]].velocity = 0;
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
            case CtrlAX: // Aux
                H_Aux_Ctrl.joint[localMap[joint]].velocity = vel;
                H_Aux_Ctrl.joint[localMap[joint]].mode = CTRL_VEL;
                H_Aux_Ctrl.active=1; ctrlOn[CtrlAX]=true; break;

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
            case CtrlAX:
                H_Aux_Ctrl.joint[localMap[joint]].acceleration = acc; break;
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
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            setPositionControl(armjoints[side][i]);
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setArmAngles(int side, Vector6d angles, bool send)
{
    if( angles.size() < ARM_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( angles.size() > ARM_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
       for(int i=0; i<ARM_JOINT_COUNT; i++)
            setJointAngle(armjoints[side][i], angles[i], false);

    else
        return BAD_SIDE;

    if(send)
        sendControls();

    return SUCCESS;
}

void Hubo_Control::setLeftArmPosCtrl() { setArmPosCtrl(LEFT); }

ctrl_flag_t Hubo_Control::setLeftArmAngles(Vector6d angles, bool send)
{ return setArmAngles( LEFT, angles, send ); }

void Hubo_Control::setRightArmPosCtrl() { setArmPosCtrl(RIGHT); }

ctrl_flag_t Hubo_Control::setRightArmAngles(Vector6d angles, bool send)
{ return setArmAngles( RIGHT, angles, send ); }


ctrl_flag_t Hubo_Control::setArmNomSpeeds(int side, Vector6d speeds)
{
    if( speeds.size() < ARM_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( speeds.size() > ARM_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
    {
//        for(int i=0; i<ARM_JOINT_COUNT; i++)
//            if( H_Arm_Ctrl[side].joint[armjoints[side][i]].mode != CTRL_POS )
//                return WRONG_MODE;

        for(int i=0; i<ARM_JOINT_COUNT; i++)
            setJointNominalSpeed( armjoints[side][i], speeds(i) );
    }
    else
        return BAD_SIDE;


    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLeftArmNomSpeeds(Vector6d speeds)
{ return setArmNomSpeeds(LEFT, speeds); }

ctrl_flag_t Hubo_Control::setRightArmNomSpeeds(Vector6d speeds)
{ return setArmNomSpeeds(RIGHT, speeds); }


// Velocity Control
ctrl_flag_t Hubo_Control::setArmVelCtrl(int side)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            setVelocityControl( armjoints[side][i] );
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setArmVels(int side, Vector6d vels, bool send)
{
    if( vels.size() < ARM_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( vels.size() > ARM_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            setJointVelocity(armjoints[side][i], vels(i), false);
    else
        return BAD_SIDE;

    if(send)
        sendControls();

    return SUCCESS;
}

void Hubo_Control::setLeftArmVelCtrl() { setArmVelCtrl(LEFT); }

ctrl_flag_t Hubo_Control::setLeftArmVels(Vector6d vels, bool send)
{ return setArmVels(LEFT, vels, send); }

void Hubo_Control::setRightArmVelCtrl() { setArmVelCtrl(RIGHT); }

ctrl_flag_t Hubo_Control::setRightArmVels(Vector6d vels, bool send)
{ return setArmVels(RIGHT, vels, send); }


// Acceleration settings
ctrl_flag_t Hubo_Control::setArmNomAcc(int side, Vector6d acc)
{
    if( acc.size() < ARM_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( acc.size() > ARM_JOINT_COUNT )
        return LONG_VECTOR;

    if( side==LEFT || side==RIGHT )
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            setJointNominalAcceleration( armjoints[side][i], acc(i) );
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLeftArmNomAcc(Vector6d acc)
{ return setArmNomAcc( LEFT, acc ); }

ctrl_flag_t Hubo_Control::setRightArmNomAcc(Vector6d acc)
{ return setArmNomAcc( RIGHT, acc ); }


// ~* Leg control sets
// Position control
ctrl_flag_t Hubo_Control::setLegPosCtrl(int side)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setPositionControl(legjoints[side][i]);
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLegAngles(int side, Vector6d angles, bool send)
{
    if( angles.size() < LEG_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( angles.size() > LEG_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setJointAngle(legjoints[side][i], angles[i], false);
    else
        return BAD_SIDE;


    if(send)
        sendControls();

    return SUCCESS;
}

void Hubo_Control::setLeftLegPosCtrl() { setLegPosCtrl(LEFT); }

ctrl_flag_t Hubo_Control::setLeftLegAngles(Vector6d angles, bool send)
{ return setLegAngles( LEFT, angles, send ); }

void Hubo_Control::setRightLegPosCtrl() { setLegPosCtrl(RIGHT); }

ctrl_flag_t Hubo_Control::setRightLegAngles(Vector6d angles, bool send)
{ return setLegAngles( RIGHT, angles, send ); }


ctrl_flag_t Hubo_Control::setLegNomSpeeds(int side, Vector6d speeds)
{
    if( speeds.size() < LEG_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( speeds.size() > LEG_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
    {
//        for(int i=0; i<LEG_JOINT_COUNT; i++)
//            if( H_Leg_Ctrl[side].joint[legjoints[side][i]].mode != CTRL_POS )
//                return WRONG_MODE;

        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setJointNominalSpeed( legjoints[side][i], speeds(i) );
    }
    else
        return BAD_SIDE;


    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLeftLegNomSpeeds(Vector6d speeds)
{ return setLegNomSpeeds(LEFT, speeds); }

ctrl_flag_t Hubo_Control::setRightLegNomSpeeds(Vector6d speeds)
{ return setLegNomSpeeds(RIGHT, speeds); }


// Velocity Control
ctrl_flag_t Hubo_Control::setLegVelCtrl(int side)
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setVelocityControl( legjoints[side][i] );
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLegVels(int side, Vector6d vels, bool send)
{
    if( vels.size() < LEG_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( vels.size() > LEG_JOINT_COUNT )
        return LONG_VECTOR;


    if( side==LEFT || side==RIGHT )
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setJointVelocity(legjoints[side][i], vels(i), false);
    else
        return BAD_SIDE;

    if(send)
        sendControls();

    return SUCCESS;
}

void Hubo_Control::setLeftLegVelCtrl() { setLegVelCtrl(LEFT); }

ctrl_flag_t Hubo_Control::setLeftLegVels(Vector6d vels, bool send)
{ return setLegVels(LEFT, vels, send); }

void Hubo_Control::setRightLegVelCtrl() { setLegVelCtrl(RIGHT); }

ctrl_flag_t Hubo_Control::setRightLegVels(Vector6d vels, bool send)
{ return setLegVels(RIGHT, vels, send); }


// Acceleration settings
ctrl_flag_t Hubo_Control::setLegNomAcc(int side, Vector6d acc)
{
    if( acc.size() < LEG_JOINT_COUNT )
        return SHORT_VECTOR;
    else if( acc.size() > LEG_JOINT_COUNT )
        return LONG_VECTOR;

    if( side==LEFT || side==RIGHT )
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            setJointNominalAcceleration( legjoints[side][i], acc(i) );
    else
        return BAD_SIDE;

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::setLeftLegNomAcc(Vector6d acc)
{ return setLegNomAcc( LEFT, acc ); }

ctrl_flag_t Hubo_Control::setRightLegNomAcc(Vector6d acc)
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
            case CtrlAX:
                H_Aux_Ctrl.joint[localMap[joint]].pos_min = radians; break;
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
            case CtrlAX:
                H_Aux_Ctrl.joint[localMap[joint]].pos_max = radians; break;
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
            case CtrlAX:
                H_Aux_Ctrl.joint[localMap[joint]].error_limit = speed; break;
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
            case CtrlAX:
                return H_Aux_Ctrl.joint[localMap[joint]].mode; break;

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
            case CtrlAX:
                return H_Aux_Ctrl.joint[localMap[joint]].position; break;
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
            case CtrlAX:
                return H_Aux_Ctrl.joint[localMap[joint]].speed; break;
        }
    }
    else
        return 0;
}

double Hubo_Control::getJointVelocity(int joint)
{
    if( joint < HUBO_JOINT_COUNT )
        return C_State.velocity[joint];
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
            case CtrlAX:
                return H_Aux_Ctrl.joint[localMap[joint]].velocity; break;
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
            case CtrlAX:
                return H_Aux_Ctrl.joint[localMap[joint]].acceleration; break;
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
ctrl_flag_t Hubo_Control::getArmAngles(int side, Vector6d &angles)
{
    if( side==LEFT || side==RIGHT )
    {
        if(angles.size() != ARM_JOINT_COUNT)
            angles.resize(ARM_JOINT_COUNT);
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            angles[i] = getJointAngle(armjoints[side][i]);
    } // TODO: make getArmAngleCtrls
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftArmAngles(Vector6d &angles)
{ getArmAngles(LEFT, angles); }
void Hubo_Control::getRightArmAngles(Vector6d &angles)
{ getArmAngles(RIGHT, angles); }

ctrl_flag_t Hubo_Control::getArmNomSpeeds(int side, Vector6d &speeds)
{
    if( side==LEFT || side==RIGHT )
    {
        if(speeds.size() != ARM_JOINT_COUNT)
            speeds.resize(ARM_JOINT_COUNT);

        for(int i=0; i<ARM_JOINT_COUNT; i++)
            speeds[i] = getJointNominalSpeed(armjoints[side][i]);
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftArmNomSpeeds(Vector6d &speeds)
{ getArmNomSpeeds(LEFT, speeds); }
void Hubo_Control::getRightArmNomSpeeds(Vector6d &speeds)
{ getArmNomSpeeds(RIGHT, speeds); }

ctrl_flag_t Hubo_Control::getArmVels(int side, Vector6d &vels)
{
    if( side==LEFT || side==RIGHT )
    {
        if(vels.size() != ARM_JOINT_COUNT)
            vels.resize(ARM_JOINT_COUNT);
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            vels[i] = getJointVelocity(armjoints[side][i]);
    } // TODO: make getArmAngleCtrls
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftArmVels(Vector6d &vels)
{ getArmVels(LEFT, vels); }
void Hubo_Control::getRightArmVels(Vector6d &vels)
{ getArmVels(RIGHT, vels); }


// Velocity control
ctrl_flag_t Hubo_Control::getArmVelCtrls(int side, Vector6d &vels)
{
    if( side==LEFT || side==RIGHT )
    {
        if(vels.size() != ARM_JOINT_COUNT)
            vels.resize(ARM_JOINT_COUNT);

        for(int i=0; i<ARM_JOINT_COUNT; i++)
            vels[i] = getJointVelocityCtrl(armjoints[side][i]);
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftArmVelCtrls(Vector6d &vels)
{ getArmVelCtrls(LEFT, vels); }
void Hubo_Control::getRightArmVelCtrls(Vector6d &vels)
{ getArmVelCtrls(RIGHT, vels); }

// Acceleration settings
ctrl_flag_t Hubo_Control::getArmNomAcc(int side, Vector6d &acc)
{
    if( side==LEFT || side==RIGHT )
    {
        if(acc.size() != ARM_JOINT_COUNT)
            acc.resize(ARM_JOINT_COUNT);

        for(int i=0; i<ARM_JOINT_COUNT; i++)
            acc[i] = getJointNominalAcceleration(armjoints[side][i]);
    }
    else
        return BAD_SIDE;
}
void Hubo_Control::getLeftArmNomAcc(Vector6d &acc)
{ getArmNomAcc(LEFT, acc); }
void Hubo_Control::getRightArmNomAcc(Vector6d &acc)
{ getArmNomAcc(RIGHT, acc); }


// ~* Leg control gets
// Position control
ctrl_flag_t Hubo_Control::getLegAngles(int side, Vector6d &angles)
{
    if( side==LEFT || side==RIGHT )
    {
        if(angles.size() != LEG_JOINT_COUNT)
            angles.resize(LEG_JOINT_COUNT);
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            angles[i] = getJointAngle(legjoints[side][i]);
    } // TODO: getLegAngleCtrls
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftLegAngles(Vector6d &angles)
{ getLegAngles(LEFT, angles); }
void Hubo_Control::getRightLegAngles(Vector6d &angles)
{ getLegAngles(RIGHT, angles); }

ctrl_flag_t Hubo_Control::getLegNomSpeeds(int side, Vector6d &speeds)
{
    if( side==LEFT || side==RIGHT )
    {
        if(speeds.size() != LEG_JOINT_COUNT)
            speeds.resize(LEG_JOINT_COUNT);

        for(int i=0; i<LEG_JOINT_COUNT; i++)
            speeds[i] = getJointNominalSpeed(legjoints[side][i]);
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftLegNomSpeeds(Vector6d &speeds)
{ getLegNomSpeeds(LEFT, speeds); }
void Hubo_Control::getRightLegNomSpeeds(Vector6d &speeds)
{ getLegNomSpeeds(RIGHT, speeds); }

ctrl_flag_t Hubo_Control::getLegVels(int side, Vector6d &vels)
{
    if( side==LEFT || side==RIGHT )
    {
        if(vels.size() != LEG_JOINT_COUNT)
            vels.resize(LEG_JOINT_COUNT);
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            vels[i] = getJointVelocity(legjoints[side][i]);
    } // TODO: make getLegAngleCtrls
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftLegVels(Vector6d &vels)
{ getLegVels(LEFT, vels); }
void Hubo_Control::getRightLegVels(Vector6d &vels)
{ getLegVels(RIGHT, vels); }

// Velocity control
ctrl_flag_t Hubo_Control::getLegVelCtrls(int side, Vector6d &vels)
{
    if( side==LEFT || side==RIGHT )
    {
        if(vels.size() != LEG_JOINT_COUNT)
            vels.resize(LEG_JOINT_COUNT);

        for(int i=0; i<LEG_JOINT_COUNT; i++)
            vels[i] = getJointVelocityCtrl(legjoints[side][i]);
    }
    else
        return BAD_SIDE;

    return SUCCESS;
}
void Hubo_Control::getLeftLegVelCtrls(Vector6d &vels)
{ getLegVelCtrls(LEFT, vels); }
void Hubo_Control::getRightLegVelCtrls(Vector6d &vels)
{ getLegVelCtrls(RIGHT, vels); }

// Acceleration settings
ctrl_flag_t Hubo_Control::getLegNomAcc(int side, Vector6d &acc)
{
    if( side==LEFT || side==RIGHT )
    {
        if(acc.size() != LEG_JOINT_COUNT)
            acc.resize(LEG_JOINT_COUNT);

        for(int i=0; i<LEG_JOINT_COUNT; i++)
            acc[i] = getJointNominalAcceleration(legjoints[side][i]);
    }
    else
        return BAD_SIDE;
}
void Hubo_Control::getLeftLegNomAcc(Vector6d &acc)
{ getLegNomAcc(LEFT, acc); }
void Hubo_Control::getRightLegNomAcc(Vector6d &acc)
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
            case CtrlAX:
                return H_Aux_Ctrl.joint[localMap[joint]].pos_min; break;
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
            case CtrlAX:
                return H_Aux_Ctrl.joint[localMap[joint]].pos_max; break;
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
            case CtrlAX:
                return H_Aux_Ctrl.joint[localMap[joint]].error_limit; break;
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
        return H_State.joint[joint].pos - jointAngleCalibration[joint];
    else
        return 0;
}

ctrl_flag_t Hubo_Control::getArmAngleStates( int side, Vector6d &angles )
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<ARM_JOINT_COUNT; i++)
            angles[i] = getJointAngleState( armjoints[side][i] );
    else
        return BAD_SIDE;

    return SUCCESS;
} 
void Hubo_Control::getRightArmAngleStates( Vector6d &angles )
{ getArmAngleStates( RIGHT, angles ); }
void Hubo_Control::getLeftArmAngleStates( Vector6d &angles )
{ getArmAngleStates( LEFT, angles ); }

ctrl_flag_t Hubo_Control::getLegAngleStates( int side, Vector6d &angles )
{
    if( side==LEFT || side==RIGHT )
        for(int i=0; i<LEG_JOINT_COUNT; i++)
            angles[i] = getJointAngleState( legjoints[side][i] );
    else
        return BAD_SIDE;

    return SUCCESS;
} 
void Hubo_Control::getRightLegAngleStates( Vector6d &angles )
{ getLegAngleStates( RIGHT, angles ); }
void Hubo_Control::getLeftLegAngleStates( Vector6d &angles )
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
            case CtrlAX: // Right Fingers
                H_Aux_Ctrl.joint[localMap[joint]].position = radians;
                H_Aux_Ctrl.joint[localMap[joint]].mode = CTRL_PASS;
                H_Aux_Ctrl.active=1; ctrlOn[CtrlAX] = true; break;
                
        }

        if(send)
            sendControls();
    }
    else
        return JOINT_OOB;

    return SUCCESS;
}

// ~~~*** Board Commands ***~~~ //
ctrl_flag_t Hubo_Control::homeJoint( int joint, bool send )
{
    if( joint < HUBO_JOINT_COUNT )
    {
        switch( ctrlMap[joint] )
        { 
            case CtrlRA:
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].position = 0;
                H_Arm_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_HOME;
                H_Arm_Ctrl[RIGHT].active = 2; ctrlOn[ctrlMap[joint]]=true; break;
            case CtrlLA:
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].position = 0;
                H_Arm_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_HOME;
                H_Arm_Ctrl[LEFT].active = 2; ctrlOn[ctrlMap[joint]]=true; break;
            case CtrlRL:
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].position = 0;
                H_Leg_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_HOME;
                H_Leg_Ctrl[RIGHT].active = 2; ctrlOn[ctrlMap[joint]]=true; break;
            case CtrlLL:
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].position = 0;
                H_Leg_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_HOME;
                H_Leg_Ctrl[LEFT].active = 2; ctrlOn[ctrlMap[joint]]=true; break;
            case CtrlRF:
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].position = 0;
                H_Fin_Ctrl[RIGHT].joint[localMap[joint]].mode = CTRL_HOME;
                H_Fin_Ctrl[RIGHT].active = 2; ctrlOn[ctrlMap[joint]]=true; break;
            case CtrlLF:
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].position = 0;
                H_Fin_Ctrl[LEFT].joint[localMap[joint]].mode = CTRL_HOME;
                H_Fin_Ctrl[LEFT].active = 2; ctrlOn[ctrlMap[joint]]=true; break;
            case CtrlAX:
                H_Aux_Ctrl.joint[localMap[joint]].position = 0;
                H_Aux_Ctrl.joint[localMap[joint]].mode = CTRL_HOME;
                H_Aux_Ctrl.active = 2; ctrlOn[ctrlMap[joint]]=true; break;
        }
        H_Cmd.type = D_GOTO_HOME;
        H_Cmd.joint = joint;
    }
    else
        return JOINT_OOB;

    if(send)
    {
        sendControls();
//        while( C_State.paused==0 )
//            update();
        sendCommands();
        
        for(int i=0; i<8; i++)
            ctrlOn[i] = false;
    }

    return SUCCESS;
}

void Hubo_Control::homeAllJoints( bool send )
{
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        homeJoint( i, false );

    H_Cmd.type = D_GOTO_HOME_ALL;
    if(send)
    {
        sendControls();
//        while( C_State.paused==0 )
//            update();
        sendCommands();
        
        for(int i=0; i<8; i++)
            ctrlOn[i] = false;
    }
}

ctrl_flag_t Hubo_Control::jointBeep( int joint, double elapseTime, bool send )
{
    if( joint < HUBO_JOINT_COUNT )
    {
        H_Cmd.type = D_JMC_BEEP;
        H_Cmd.joint = joint;
        H_Cmd.dValues[0] = elapseTime;

    }
    else
        return JOINT_OOB;

    if(send)
        sendCommands();

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::resetJoint( int joint, bool send )
{
    if( joint < HUBO_JOINT_COUNT )
    {
        H_Cmd.type = D_ZERO_ENCODER;
        H_Cmd.joint = joint;
    }
    else
        return JOINT_OOB;

    if(send)
        sendCommands();

    return SUCCESS;
}

void Hubo_Control::startAllSensors( bool send )
{
    H_Cmd.type = D_NULL_SENSORS_ALL;

    if(send)
        sendCommands();
}

ctrl_flag_t Hubo_Control::startSensor( hubo_sensor_index_t sensor, bool send )
{
    if( sensor < SENSOR_INDEX_COUNT )
    {
        H_Cmd.type = D_NULL_SENSOR;
        switch( sensor )
        {
            case HUBO_FT_R_HAND: H_Cmd.param[0] = D_R_HAND_FT; break;
            case HUBO_FT_L_HAND: H_Cmd.param[0] = D_L_HAND_FT; break;
            case HUBO_FT_R_FOOT: H_Cmd.param[0] = D_R_FOOT_FT; break;
            case HUBO_FT_L_FOOT: H_Cmd.param[0] = D_L_FOOT_FT; break;
            case HUBO_IMU0: H_Cmd.param[0] = D_IMU_SENSOR_0; break;
            case HUBO_IMU1: H_Cmd.param[0] = D_IMU_SENSOR_1; break;
            case HUBO_IMU2: H_Cmd.param[0] = D_IMU_SENSOR_2; break;
            default: return SENSOR_OOB;
        }
    }
    else
        return SENSOR_OOB;

    if(send)
        sendCommands();

    return SUCCESS;
}

ctrl_flag_t Hubo_Control::zeroTilt( hubo_sensor_index_t sensor, bool send )
{
    if( sensor < SENSOR_INDEX_COUNT )
    {
        H_Cmd.type = D_NULL_SENSOR;
        switch( sensor )
        {
            case HUBO_FT_R_FOOT: H_Cmd.param[0] = D_R_FOOT_ACC; break;
            case HUBO_FT_L_FOOT: H_Cmd.param[0] = D_L_FOOT_ACC; break;
            case HUBO_IMU0: H_Cmd.param[0] = D_IMU_SENSOR_0; break;
            case HUBO_IMU1: H_Cmd.param[0] = D_IMU_SENSOR_1; break;
            case HUBO_IMU2: H_Cmd.param[0] = D_IMU_SENSOR_2; break;
            default: return SENSOR_OOB;
        }
    }
    else
        return SENSOR_OOB;

    if(send)
        sendCommands();
    
    return SUCCESS;

}

ctrl_flag_t Hubo_Control::initializeBoard( int joint, bool send )
{
    if( joint < HUBO_JOINT_COUNT )
    {
        H_Cmd.type = D_JMC_INITIALIZE;
        H_Cmd.joint = joint;
    }
    else
        return JOINT_OOB;

    if(send)
        sendCommands();

    return SUCCESS;
}

void Hubo_Control::initializeAll( bool send )
{
    H_Cmd.type = D_JMC_INITIALIZE_ALL;
    
    if(send)
        sendCommands();
}

ctrl_flag_t Hubo_Control::motorCtrlSwitch( int joint, bool on, bool send )
{
    if( joint < HUBO_JOINT_COUNT )
    {
        H_Cmd.type = D_CTRL_SWITCH;
        H_Cmd.joint = joint;
        if(on)
            H_Cmd.param[0] = D_ENABLE;
        else
            H_Cmd.param[0] = D_DISABLE;
    }
    else
        return JOINT_OOB;

    // TODO: Consider how to inform control daemon that the motor is turned off
    
    if(send)
        sendCommands();
    
    return SUCCESS;
}

ctrl_flag_t Hubo_Control::motorCtrlOn( int joint, bool send )
{ return motorCtrlSwitch( joint, true, send ); }

ctrl_flag_t Hubo_Control::motorCtrlOff( int joint, bool send )
{ return motorCtrlSwitch( joint, false, send ); }

ctrl_flag_t Hubo_Control::fetSwitch( int joint, bool on, bool send )
{
    if( joint < HUBO_JOINT_COUNT )
    {
        H_Cmd.type = D_FET_SWITCH;
        H_Cmd.joint = joint;
        if(on)
            H_Cmd.param[0] = D_ENABLE;
        else
            H_Cmd.param[0] = D_DISABLE;
    }
    else
        return JOINT_OOB;

    // TODO: Consider how to inform control daemon that the motor is turned off
    
    if(send)
        sendCommands();
    
    return SUCCESS;
}

ctrl_flag_t Hubo_Control::fetOn( int joint, bool send )
{ return fetSwitch( joint, true, send ); }

ctrl_flag_t Hubo_Control::fetOff( int joint, bool send )
{ return fetSwitch( joint, false, send ); }

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


void Hubo_Control::huboArmFK(Eigen::Isometry3d &B, Vector6d &q, int side) {
    Eigen::Isometry3d hand;
    hand(0,0) =  1; hand(0,1) =  0; hand(0,2) = 0; hand(0,3) =   0;
    hand(1,0) =  0; hand(1,1) =  0; hand(1,2) =-1; hand(1,3) =   0;
    hand(2,0) =  0; hand(2,1) =  1; hand(2,2) = 0; hand(2,3) =   0;
    hand(3,0) =  0; hand(3,1) =  0; hand(3,2) = 0; hand(3,3) =   1;
    
    huboArmFK(B, q, side, hand);
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
        
/*
        limits <<
        -2,   2,
        -2,  .3,
        -2,   2,
        -2,   0.01,
        -2,   2,
        -1.4, 1.2;
*/        
        // Set offsets
        offset(1) = limits(1,1); // Note: I think this might be backwards
//        offset(1) = -limits(1,1);
        
    } else {
        // Transformation from Neck frame to left shoulder pitch frame
        neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
        neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) =  l1;
        neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
        neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
/*        
        limits <<
        -2,   2,
        -.3,   2,
        -2,   2,
        -2,   0.01,
        -2,   2,
        -1.4, 1.2;
*/        
        // Set offsets
        offset(1) = limits(1,0); // Note: I think this might be backwards
//        offset(1) = -limits(1,0);
    }
     
    // Calculate forward kinematics
    B = neck;
    for (int i = 0; i < 6; i++) {
        DH2HG(T, t(i)+q(i)-offset(i), f(i), r(i), d(i));
        B = B*T;
    }
    B = B*endEffector;
    
}


void Hubo_Control::huboArmIK(Vector6d &q, const Eigen::Isometry3d B, Vector6d qPrev, int side) {
    // Hand	
    Eigen::Isometry3d hand;
    hand(0,0) =  1; hand(0,1) =  0; hand(0,2) = 0; hand(0,3) =   0;
    hand(1,0) =  0; hand(1,1) =  0; hand(1,2) =-1; hand(1,3) =   0;
    hand(2,0) =  0; hand(2,1) =  1; hand(2,2) = 0; hand(2,3) =   0;
    hand(3,0) =  0; hand(3,1) =  0; hand(3,2) = 0; hand(3,3) =   1;
    
    huboArmIK(q, B, qPrev, side, hand);
}
  
void Hubo_Control::huboArmIK(Vector6d &q, const Eigen::Isometry3d B, Vector6d qPrev, int side, const Eigen::Isometry3d &endEffector)
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
/*
        limits <<
        -2,   2,
        -2,  .3,
        -2,   2,
        -2,   0.01,
        -2,   2,
        -1.4, 1.2;
*/
        // Set offsets
        offset(1) = limits(1,1); 
        
    } else {
        // Transformation from Neck frame to left shoulder pitch frame
        neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
        neck(1,0) = 0; neck(1,1) =  0; neck(1,2) = 1; neck(1,3) =  l1;
        neck(2,0) = 0; neck(2,1) = -1; neck(2,2) = 0; neck(2,3) =   0;
        neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;
/*        
        limits <<
        -2,   2,
        -.3,   2,
        -2,   2,
        -2,   0.01,
        -2,   2,
        -1.4, 1.2;
*/        
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

void Hubo_Control::huboLegIK(Vector6d &q, const Eigen::Isometry3d B, Vector6d qPrev, int side) {
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
}

ctrl_flag_t Hubo_Control::footVelocityIK( Vector6d &qdot, Eigen::Vector3d &velocity, int side )
{
    Eigen::Vector3d rotVel; rotVel.setZero();
    return footVelocityIK( qdot, velocity, rotVel, side );
}

ctrl_flag_t Hubo_Control::footVelocityIK( Vector6d &qdot, Eigen::Vector3d &velocity,
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

ctrl_flag_t Hubo_Control::hipVelocityIK( Vector6d &qdot, Eigen::Vector3d &velocity, int side ) 
{
    Eigen::Vector3d rotVel; rotVel.setZero();
    return hipVelocityIK( qdot, velocity, rotVel, side );
}

ctrl_flag_t Hubo_Control::hipVelocityIK( Vector6d &qdot, Eigen::Vector3d &velocity, const Vector6d &q ) 
{
    Eigen::Vector3d rotVel; rotVel.setZero();
    return hipVelocityIK( qdot, velocity, rotVel, q );
}

ctrl_flag_t Hubo_Control::hipVelocityIK( Vector6d &qdot, Eigen::Vector3d &velocity,
        Eigen::Vector3d &angularVel, int side )
{
    Vector6d q;
    if( side!=RIGHT && side!=LEFT )
        side = RIGHT;
    getLegAngleStates( side, q );
    hipVelocityIK( qdot, velocity, angularVel, q );
}

ctrl_flag_t Hubo_Control::hipVelocityIK( Vector6d &qdot, Eigen::Vector3d &velocity,
        Eigen::Vector3d &angularVel, const Vector6d &q )
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





void Hubo_Control::HuboDrillFK(Eigen::Isometry3d &B, Vector6d &q) {
    Eigen::Isometry3d drill;
    
    double ld = 7*25.4/1000.0;
    double ad = M_PI/4;
    
    drill(0,0) = cos(ad);  drill(0,1) = 0; drill(0,2) = sin(ad); drill(0,3) = ld*cos(ad);
    drill(1,0) = 0;        drill(1,1) = 1; drill(1,2) = 0;       drill(1,3) = 0;
    drill(2,0) = -sin(ad); drill(2,1) = 0; drill(2,2) = cos(ad); drill(2,3) = -ld*sin(ad);
    drill(3,0) = 0;        drill(3,1) = 0; drill(3,2) = 0;       drill(3,3) = 1;
    
    huboArmFK(B, q, RIGHT, drill);
}

void Hubo_Control::HuboDrillIK(Vector6d &q, double y) {
    Vector6d qPrev; qPrev.setZero();
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


