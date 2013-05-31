
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


#include <Hubo_Control.h>
#include "Walker.h"
#include "balance-daemon.h"


void Walker::flattenFoot( Hubo_Control &hubo, zmp_traj_element_t &elem,
			nudge_state_t &state, balance_gains_t &gains, double dt )
{
    
     
    state.ankle_roll_compliance[LEFT] -= gains.decay_gain[LEFT]*state.ankle_roll_compliance[LEFT];
    state.ankle_roll_compliance[RIGHT] -= gains.decay_gain[RIGHT]*state.ankle_roll_compliance[RIGHT];

    state.ankle_pitch_compliance[LEFT] -= gains.decay_gain[LEFT]*state.ankle_pitch_compliance[LEFT];
    state.ankle_pitch_compliance[RIGHT] -= gains.decay_gain[RIGHT]*state.ankle_pitch_compliance[RIGHT];

    if( gains.force_min_threshold[RIGHT] < hubo.getRightFootFz() 
     && hubo.getRightFootFz() < gains.force_max_threshold[RIGHT] )
    {
        state.ankle_roll_compliance[RIGHT] += dt*gains.flattening_gain[RIGHT]
                                                *( hubo.getRightFootMx() );
        state.ankle_pitch_compliance[RIGHT] += dt*gains.flattening_gain[RIGHT]
                                                 *( hubo.getRightFootMy() );
    }

    if( gains.force_min_threshold[LEFT] < hubo.getLeftFootFz()
     && hubo.getLeftFootFz() < gains.force_max_threshold[LEFT] )
    {
        state.ankle_roll_compliance[LEFT] += dt*gains.flattening_gain[LEFT]
                                               *( hubo.getLeftFootMx() );
        state.ankle_pitch_compliance[LEFT] += dt*gains.flattening_gain[LEFT]
                                                *( hubo.getLeftFootMy() );
    }


    elem.angles[RAR] += state.ankle_roll_compliance[RIGHT];
    elem.angles[RAP] += state.ankle_pitch_compliance[RIGHT];
    elem.angles[LAR] += state.ankle_roll_compliance[LEFT];
    elem.angles[LAP] += state.ankle_pitch_compliance[LEFT];

}

void Walker::straightenBack( Hubo_Control &hubo, zmp_traj_element_t &elem,
        nudge_state_t &state, balance_gains_t &gains, double dt )
{
    if( elem.stance == SINGLE_LEFT )
    {
        state.ankle_pitch_resistance[LEFT] += dt*gains.straightening_pitch_gain[LEFT]
                                                *( hubo.getAngleY() );
        state.ankle_roll_resistance[LEFT]  += dt*gains.straightening_roll_gain[LEFT]
                                                *( hubo.getAngleX() );
    }

    if( elem.stance == SINGLE_RIGHT )
    {
        state.ankle_pitch_resistance[RIGHT] += dt*gains.straightening_pitch_gain[RIGHT]
                                                 *( hubo.getAngleY() );
        state.ankle_roll_resistance[RIGHT]  += dt*gains.straightening_roll_gain[RIGHT]
                                                 *( hubo.getAngleX() );
    }
    
    elem.angles[RAR] += state.ankle_roll_resistance[RIGHT];
    elem.angles[RAP] += state.ankle_pitch_resistance[RIGHT];
    elem.angles[LAR] += state.ankle_roll_resistance[LEFT];
    elem.angles[LAP] += state.ankle_pitch_resistance[LEFT];

}

void Walker::complyKnee( Hubo_Control &hubo, zmp_traj_element_t &elem,
        nudge_state_t &state, balance_gains_t &gains, double dt )
{
    state.knee_velocity_offset[LEFT] =
                   gains.spring_gain[LEFT]*( elem.angles[LKN] - hubo.getJointAngle(LKN))
                 + gains.damping_gain[LEFT]*( -state.knee_velocity_offset[LEFT] )
                 + gains.fz_response[LEFT]*( hubo.getLeftFootFz() );

    state.knee_velocity_offset[RIGHT] =
                   gains.spring_gain[RIGHT]*( elem.angles[LKN] - hubo.getJointAngle(LKN))
                 + gains.damping_gain[RIGHT]*( -state.knee_velocity_offset[RIGHT] )
                 + gains.fz_response[RIGHT]*( hubo.getRightFootFz() );
   
    for(int i=0; i<2; i++)
        state.knee_offset[i] += dt*state.knee_velocity_offset[i];

    elem.angles[LAP] += -state.knee_offset[LEFT]/2.0;
    elem.angles[LKN] += state.knee_offset[LEFT]/2.0;
    elem.angles[LHP] += -state.knee_offset[LEFT]/2.0;

    elem.angles[RAP] += -state.knee_offset[RIGHT]/2.0;
    elem.angles[RKN] += state.knee_offset[RIGHT];
    elem.angles[RHP] += -state.knee_offset[RIGHT]/2.0;
}



Walker::Walker(double maxInitTime, double jointSpaceTolerance, double jointVelContinuityTolerance) :
        m_maxInitTime(maxInitTime),
        m_jointSpaceTolerance( jointSpaceTolerance ),
        m_jointVelContTol( jointVelContinuityTolerance ),
        keepWalking(true),
        hubo()
{
    ach_status_t r = ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                HUBO_CHAN_ZMP_TRAJ_NAME, ach_result_to_string(r), (int)r );
    
    r = ach_open( &param_chan, BALANCE_PARAM_CHAN, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                BALANCE_PARAM_CHAN, ach_result_to_string(r), (int)r );

    r = ach_open( &bal_cmd_chan, BALANCE_CMD_CHAN, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                BALANCE_CMD_CHAN, ach_result_to_string(r), (int)r );

    r = ach_open( &bal_state_chan, BALANCE_STATE_CHAN, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "Problem with channel %s: %s (%d)\n",
                BALANCE_STATE_CHAN, ach_result_to_string(r), (int)r );

    memset( &cmd, 0, sizeof(cmd) );
    memset( &bal_state, 0, sizeof(bal_state) );
} 

Walker::~Walker()
{
    ach_close( &zmp_chan );
    ach_close( &param_chan );
    ach_close( &bal_cmd_chan );
    ach_close( &bal_state_chan );
}

void Walker::commenceWalking(balance_state_t &parent_state, nudge_state_t &state, balance_gains_t &gains)
{
    int timeIndex=0, nextTimeIndex=0, prevTimeIndex=0;
    keepWalking = true;
    size_t fs;
 
    zmp_traj_t prevTrajectory, currentTrajectory, nextTrajectory;
    memset( &prevTrajectory, 0, sizeof(prevTrajectory) );
    memset( &currentTrajectory, 0, sizeof(currentTrajectory) );
    memset( &nextTrajectory, 0, sizeof(nextTrajectory) );
    
    // Note: Consider making these values persistent
    memset( &state, 0, sizeof(state) );


    memcpy( &bal_state, &parent_state, sizeof(bal_state) );

    bal_state.m_balance_mode = BAL_ZMP_WALKING; 
    bal_state.m_walk_mode = WALK_WAITING;
    bal_state.m_walk_error = NO_WALK_ERROR;
    sendState();

    fprintf(stdout, "Waiting for first trajectory\n"); fflush(stdout);
    ach_status_t r;
    do {
        struct timespec t;
        clock_gettime( ACH_DEFAULT_CLOCK, &t );
        t.tv_sec += 1;
        r = ach_get( &zmp_chan, &currentTrajectory, sizeof(currentTrajectory), &fs,
                    &t, ACH_O_WAIT | ACH_O_LAST );
    } while(!daemon_sig_quit && r==ACH_TIMEOUT);

    if(!daemon_sig_quit)
        fprintf(stdout, "First trajectory acquired\t");
        
    daemon_assert( ACH_TIMEOUT != r, __LINE__ );

    bal_state.m_walk_mode = WALK_INITIALIZING;
    sendState();

    ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );

    hubo.update(true);
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        hubo.setJointAngle( i, currentTrajectory.traj[0].angles[i] );
        hubo.setJointNominalSpeed( i, 0.4 );
        hubo.setJointNominalAcceleration( i, 0.4 );
    }
    
    hubo.setJointNominalSpeed( RKN, 0.8 );
    hubo.setJointNominalAcceleration( RKN, 0.8 );
    hubo.setJointNominalSpeed( LKN, 0.8 );
    hubo.setJointNominalAcceleration( LKN, 0.8 );

    hubo.setJointAngle( RSR, currentTrajectory.traj[0].angles[RSR] + hubo.getJointAngleMax(RSR) );
    hubo.setJointAngle( LSR, currentTrajectory.traj[0].angles[LSR] + hubo.getJointAngleMin(LSR) );

    hubo.sendControls();

    double m_maxInitTime = 10;
    double biggestErr = 0;
    int worstJoint=-1;
    
    double dt, time, stime; stime=hubo.getTime(); time=hubo.getTime();
    double norm = m_jointSpaceTolerance+1; // make sure this fails initially
    while( !daemon_sig_quit && (norm > m_jointSpaceTolerance && time-stime < m_maxInitTime)) {
        hubo.update(true);
        norm = 0;
        for(int i=0; i<HUBO_JOINT_COUNT; i++)
        {
            double err=0;
            if( LF1!=i && LF2!=i && LF3!=i && LF4!=i && LF5!=i
             && RF1!=i && RF2!=i && RF3!=i && RF4!=i && RF5!=i
             && NK1!=i && NK2!=i && NKY!=i )
                err = (hubo.getJointAngleState( i )-currentTrajectory.traj[0].angles[i]);
            if( LSR == i )
                err -= hubo.getJointAngleMin(i);
            if( RSR == i )
                err -= hubo.getJointAngleMax(i);

            norm += err*err;
            if( fabs(err) > fabs(biggestErr) )
            {
                biggestErr = err;
                worstJoint = i;
            }
        }
        time = hubo.getTime();
    }

    if( time-stime >= m_maxInitTime )
    {
        fprintf(stderr, "Warning: could not reach the starting Trajectory within %f seconds\n"
                        " -- Biggest error was %f radians in joint %s\n",
                        m_maxInitTime, biggestErr, jointNames[worstJoint] );

        keepWalking = false;
        
        bal_state.m_walk_error = WALK_INIT_FAILED;
    }

    timeIndex = 1;
    bool haveNewTrajectory = false;
    fprintf(stdout, "Beginning main walking loop\n"); fflush(stdout);
    while(keepWalking && !daemon_sig_quit)
    {
        haveNewTrajectory = checkForNewTrajectory(nextTrajectory, haveNewTrajectory);
        ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );
        hubo.update(true);

        bal_state.m_walk_mode = WALK_IN_PROGRESS;

        dt = hubo.getTime() - time;
        time = hubo.getTime();
        if( dt <= 0 )
        {
            fprintf(stderr, "Something unnatural has happened... %f\n", dt);
            continue;
        }

        if( timeIndex==0 )
        {
            bal_state.m_walk_error = NO_WALK_ERROR;
            nextTimeIndex = timeIndex+1;
            executeTimeStep( hubo, prevTrajectory.traj[prevTimeIndex],
                                   currentTrajectory.traj[timeIndex],
                                   currentTrajectory.traj[nextTimeIndex],
                                   state, gains, dt );
            
        }
        else if( timeIndex == currentTrajectory.periodEndTick && haveNewTrajectory )
        {
            if( validateNextTrajectory( currentTrajectory.traj[timeIndex],
                                        nextTrajectory.traj[0], dt ) )
            {
                nextTimeIndex = 0;
                executeTimeStep( hubo, currentTrajectory.traj[prevTimeIndex],
                                       currentTrajectory.traj[timeIndex],
                                       nextTrajectory.traj[nextTimeIndex],
                                       state, gains, dt );
                
                memcpy( &prevTrajectory, &currentTrajectory, sizeof(prevTrajectory) );
                memcpy( &currentTrajectory, &nextTrajectory, sizeof(nextTrajectory) );
                fprintf(stderr, "Notice: Swapping in new trajectory\n");
            }
            else
            {
                fprintf(stderr, "WARNING: Discontinuous trajectory passed in. Walking to a stop.\n");
                bal_state.m_walk_error = WALK_FAILED_SWAP;

                nextTimeIndex = timeIndex+1;
                executeTimeStep( hubo, currentTrajectory.traj[prevTimeIndex],
                                       currentTrajectory.traj[timeIndex],
                                       currentTrajectory.traj[nextTimeIndex],
                                       state, gains, dt );
            }
            haveNewTrajectory = false;
        }
        else if( timeIndex == currentTrajectory.periodEndTick && currentTrajectory.reuse )
        {
            checkCommands();
            if( cmd.cmd_request != BAL_ZMP_WALKING )
                currentTrajectory.reuse = false;

            if( currentTrajectory.reuse == true )
                nextTimeIndex = currentTrajectory.periodStartTick;
            else
                nextTimeIndex = timeIndex+1;

            executeTimeStep( hubo, currentTrajectory.traj[prevTimeIndex],
                                   currentTrajectory.traj[timeIndex],
                                   currentTrajectory.traj[nextTimeIndex],
                                   state, gains, dt );
        }
        else if( timeIndex < currentTrajectory.count-1 )
        {
            nextTimeIndex = timeIndex+1;
            executeTimeStep( hubo, currentTrajectory.traj[prevTimeIndex],
                                   currentTrajectory.traj[timeIndex],
                                   currentTrajectory.traj[nextTimeIndex],
                                   state, gains, dt );
        }
        else if( timeIndex == currentTrajectory.count-1 && haveNewTrajectory )
        {
            checkCommands();
            if( cmd.cmd_request != BAL_ZMP_WALKING )
                keepWalking = false;

            if( keepWalking )
            {
                if( validateNextTrajectory( currentTrajectory.traj[timeIndex],
                                            nextTrajectory.traj[0], dt ) )
                {
                    bal_state.m_walk_error = NO_WALK_ERROR;
                    nextTimeIndex = 0;
                    executeTimeStep( hubo, currentTrajectory.traj[prevTimeIndex],
                                           currentTrajectory.traj[timeIndex],
                                           nextTrajectory.traj[nextTimeIndex],
                                           state, gains, dt );
                    
                    memcpy( &prevTrajectory, &currentTrajectory, sizeof(prevTrajectory) );
                    memcpy( &currentTrajectory, &nextTrajectory, sizeof(nextTrajectory) );
                }
                else
                {
                    bal_state.m_walk_mode = WALK_WAITING;
                    bal_state.m_walk_error = WALK_FAILED_SWAP;
                    fprintf(stderr, "WARNING: Discontinuous trajectory passed in. Discarding it.\n");
                }
                haveNewTrajectory = false;
            }
        }
        else
        {
            checkCommands();
            if( cmd.cmd_request != BAL_ZMP_WALKING )
                keepWalking = false;
        }

        prevTimeIndex = timeIndex;
        timeIndex = nextTimeIndex;
        sendState();
    }

    bal_state.m_walk_mode = WALK_INACTIVE;
    sendState();
}




bool Walker::checkForNewTrajectory(zmp_traj_t &newTrajectory, bool haveNewTrajAlready)
{
    size_t fs;
    
    ach_status_t r = ach_get( &zmp_chan, &newTrajectory, sizeof(newTrajectory), &fs, NULL, ACH_O_LAST );

    if( ACH_OK==r || ACH_MISSED_FRAME==r )
    {
        fprintf(stdout, "Noticed new trajectory: ID #%d\n", (int)newTrajectory.trajNumber);
        return true;
    }
    else
        return haveNewTrajAlready || false;

}

bool Walker::validateNextTrajectory( zmp_traj_element_t &current, zmp_traj_element_t &next, double dt )
{
    bool valid = true;
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        if( fabs(next.angles[i]-current.angles[i])/fabs(dt) > fabs(m_jointVelContTol) )
            valid = false;

    return valid;
}


void Walker::executeTimeStep( Hubo_Control &hubo, zmp_traj_element_t &prevElem,
            zmp_traj_element_t &currentElem, zmp_traj_element &nextElem,
            nudge_state_t &state, balance_gains_t &gains, double dt )
{
    flattenFoot( hubo, currentElem, state, gains, dt );
    //straightenBack( hubo, currentElem, state, gains, dt );
    //complyKnee( hubo, currentElem, state, gains, dt );
    //nudgeRefs( hubo, currentElem, state, dt, hkin ); //vprev, verr, dt );

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        hubo.setJointAngle( i, currentElem.angles[i] );
        hubo.setJointNominalSpeed( i,
               (currentElem.angles[i]-prevElem.angles[i])*ZMP_TRAJ_FREQ_HZ );
        double accel = ZMP_TRAJ_FREQ_HZ*ZMP_TRAJ_FREQ_HZ*(
                            prevElem.angles[i]
                        - 2*currentElem.angles[i]
                        +   nextElem.angles[i] );
        hubo.setJointNominalAcceleration( i, 10*accel );
    }


    hubo.setJointAngle( RSR, currentElem.angles[RSR] + hubo.getJointAngleMax(RSR) );
    hubo.setJointAngle( LSR, currentElem.angles[LSR] + hubo.getJointAngleMin(LSR) );

    hubo.setJointAngleMin( LHR, currentElem.angles[RHR] );
    hubo.setJointAngleMax( RHR, currentElem.angles[LHR] );
    hubo.sendControls();
}


void Walker::sendState()
{
    ach_put( &bal_state_chan, &bal_state, sizeof(bal_state) );
}


void Walker::checkCommands()
{
    size_t fs;
    ach_get( &bal_cmd_chan, &cmd, sizeof(cmd), &fs, NULL, ACH_O_LAST );
}

