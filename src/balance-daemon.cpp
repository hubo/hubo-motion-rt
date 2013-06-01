
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

#include "balance-daemon.h"
#include "Walker.h"
#include "Hubo_Control.h"
#include "manip.h"


ach_channel_t bal_cmd_chan;
ach_channel_t bal_state_chan;
ach_channel_t bal_param_chan;
ach_channel_t manip_override_chan;
ach_channel_t manip_state_chan;


void staticBalance(Hubo_Control &hubo, balance_cmd_t &cmd, balance_gains_t &gains, double dt);


int main(int argc, char **argv)
{
    Hubo_Control hubo("balance-daemon", 35);

    hubo.storeAllDefaults();

    ach_status_t r = ach_open( &bal_cmd_chan, BALANCE_CMD_CHAN, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );

    r = ach_open( &bal_state_chan, BALANCE_STATE_CHAN, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );

    r = ach_open( &bal_param_chan, BALANCE_PARAM_CHAN, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );

    r = ach_open( &manip_override_chan, CHAN_HUBO_MANIP_OVERRIDE, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );

    r= ach_open( &manip_state_chan, CHAN_HUBO_MANIP_STATE, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );

    Walker walk;

    balance_cmd_t cmd;
    balance_state_t state;
    balance_gains_t gains;
    nudge_state_t nudge;
    manip_override_t ovr;
    hubo_manip_state_t manip_state;

    memset( &cmd, 0, sizeof(cmd) );
    memset( &state, 0, sizeof(state) );
    memset( &gains, 0, sizeof(gains) );
    memset( &nudge, 0, sizeof(nudge) );
    memset( &ovr, 0, sizeof(ovr) );
    memset( &manip_state, 0, sizeof(manip_state) );
    
    hubo.update();
    double dt, time=hubo.getTime();

    size_t fs;
    while( !daemon_sig_quit )
    {
        hubo.update();
        dt = hubo.getTime() - time;
        time = hubo.getTime();
        if( dt <= 0 )
        {
            fprintf(stderr, "Something unnatural has happened... %f\n", dt);
            continue;
        }

        ach_get( &bal_cmd_chan, &cmd, sizeof(cmd), &fs, NULL, ACH_O_LAST );
        ach_get( &bal_param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );

        state.m_balance_mode = cmd.cmd_request;

        
        if( BAL_LEGS_ONLY == cmd.cmd_request )
        {
            staticBalance(hubo, cmd, gains, dt);
        }
        else if( BAL_ZMP_WALKING == cmd.cmd_request )
        {
            ach_get( &manip_state_chan, &manip_state, sizeof(manip_state),
                        &fs, NULL, ACH_O_LAST );

            if( OVR_SOVEREIGN == manip_state.override )
            {
                ovr.m_override = OVR_ACQUIESCENT;
                ach_put( &manip_override_chan, &ovr, sizeof(ovr) );

                staticBalance(hubo, cmd, gains, dt);
            }
            else if( OVR_ACQUIESCENT == manip_state.override )
            {
                walk.commenceWalking(state, nudge, gains);
                ovr.m_override = OVR_SOVEREIGN;
                ach_put( &manip_override_chan, &ovr, sizeof(ovr) );
                // Probably not necessary...
                hubo.releaseLeftArm();
                hubo.releaseRightArm();
                hubo.releaseBody();
                hubo.releaseNeck();
            }
        }

        ach_put( &bal_state_chan, &state, sizeof(state) );

    }

    return 0;
}





void staticBalance(Hubo_Control &hubo, balance_cmd_t &cmd, balance_gains_t &gains, double dt)
{
    hubo.setJointNominalAcceleration( LKN, 0.6 );
    hubo.setJointNominalAcceleration( LHP, 0.6 );
    hubo.setJointNominalAcceleration( LHR, 0.6 );
    hubo.setJointNominalAcceleration( LAR, 0.6 );
    hubo.setJointNominalAcceleration( LAP, 0.6 );

    hubo.setJointNominalAcceleration( RKN, 0.6 );
    hubo.setJointNominalAcceleration( RHP, 0.6 );
    hubo.setJointNominalAcceleration( RHR, 0.6 );
    hubo.setJointNominalAcceleration( RAR, 0.6 );
    hubo.setJointNominalAcceleration( RAP, 0.6 );

    hubo.setJointAngleMax( LHP, 0 );
    hubo.setJointAngleMax( RHP, 0 );


    double L1 = 2*0.3002;
    double L2 = 0.28947 + 0.0795;
    
    if( cmd.height-L2 > L1 )
        cmd.height = L1+L2;
    else if( cmd.height-L2 < 0.25 ) //TODO: Don't hard code this
        cmd.height = L1+0.2;

    double knee = acos( (cmd.height-L2)/L1 )*2;

    double kneeAngleErrorL = knee - hubo.getJointAngle( LKN );
    double kneeAngleErrorR = knee - hubo.getJointAngle( RKN );

    double kneeVelL = gains.spring_gain[LEFT]*kneeAngleErrorL;
    double kneeVelR = gains.spring_gain[RIGHT]*kneeAngleErrorR;

    double pitchL = gains.straightening_pitch_gain[LEFT]*hubo.getAngleY()
                    + gains.flattening_gain[LEFT]*hubo.getLeftFootMy()
                    - kneeVelL/2;
    double rollL  = gains.straightening_roll_gain[LEFT]*hubo.getAngleX()
                    + gains.flattening_gain[LEFT]*hubo.getLeftFootMx();
    
    double pitchR = gains.straightening_pitch_gain[RIGHT]*hubo.getAngleY()
                    + gains.flattening_gain[RIGHT]*hubo.getRightFootMy()
                    - kneeVelR/2;
    double rollR  = gains.straightening_roll_gain[RIGHT]*hubo.getAngleX()
                    + gains.flattening_gain[RIGHT]*hubo.getRightFootMx();

    
    hubo.setJointVelocity( LAP, pitchL );
    hubo.setJointVelocity( LAR, rollL );
    hubo.setJointVelocity( LKN, kneeVelL );
    hubo.setJointVelocity( LHP, -kneeVelL/2.0 );

    hubo.setJointVelocity( RAP, pitchR );
    hubo.setJointVelocity( RAR, rollR );
    hubo.setJointVelocity( RKN, kneeVelR );
    hubo.setJointVelocity( RHP, -kneeVelR/2.0 );

    hubo.sendControls();

}























