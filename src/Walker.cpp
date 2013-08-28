
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


//#include "Hubo_Control.h"
#include "DrcHuboKin.h"
#include "balance-daemon.h"
#include "Walker.h"

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
    if( elem.effector_frame == EFFECTOR_L_FOOT )
    {
        state.ankle_pitch_resistance[LEFT] += dt*gains.straightening_pitch_gain[LEFT]
                                                *( hubo.getAngleY() );
        state.ankle_roll_resistance[LEFT]  += dt*gains.straightening_roll_gain[LEFT]
                                                *( hubo.getAngleX() );
    }

    if( elem.effector_frame == EFFECTOR_R_FOOT )
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
/*
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
*/

void Walker::complyKnee( Hubo_Control &hubo, zmp_traj_element_t &elem,
        nudge_state_t &state, balance_gains_t &gains, double dt )
{
    counter++;
    //-------------------------
    //      STANCE TYPE
    //-------------------------
    // Figure out if we're in single or double support stance and which leg
    int side;    //!< variable for stance leg
    if((unsigned char*)0x8 == elem.supporting)
        side = LEFT;
    else if((unsigned char*)"0100" == elem.supporting)
        side = RIGHT;
    else
        side = 100;

    //-------------------------
    //          GAINS
    //-------------------------
    Eigen::Vector3d spring_gain, damping_gain;
    spring_gain.setZero(); damping_gain.setZero();

    spring_gain.z() = gains.spring_gain[LEFT];
    damping_gain.z() = gains.damping_gain[LEFT];

    //-------------------------
    //    COPY JOINT ANGLES
    //-------------------------
    // Store leg joint angels for current trajectory timestep
    Vector6d qPrev[2];
    qPrev[LEFT](HY) = elem.angles[LHY],
    qPrev[LEFT](HR) = elem.angles[LHR],
    qPrev[LEFT](HP) = elem.angles[LHP],
    qPrev[LEFT](KN) = elem.angles[LKN],
    qPrev[LEFT](AP) = elem.angles[LAP],
    qPrev[LEFT](AR) = elem.angles[LAR];

    qPrev[RIGHT](HY) = elem.angles[RHY],
    qPrev[RIGHT](HR) = elem.angles[RHR],
    qPrev[RIGHT](HP) = elem.angles[RHP],
    qPrev[RIGHT](KN) = elem.angles[RKN],
    qPrev[RIGHT](AP) = elem.angles[RAP],
    qPrev[RIGHT](AR) = elem.angles[RAR];


    //-------------------------
    //        HIP YAWS
    //-------------------------
    // Get rotation matrix for each hip yaw
    Eigen::Matrix3d yawRot[2];
    yawRot[LEFT] = Eigen::AngleAxisd(hubo.getJointAngle(LHY), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    yawRot[RIGHT]= Eigen::AngleAxisd(hubo.getJointAngle(RHY), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    //-------------------------
    //        FOOT TFs
    //-------------------------
    // Determine how much we need to nudge to hips over to account for
    // error in ankle torques about the x- and y- axes.
    // If Roll torque is positive (ie. leaning left) we want hips to go right (ie. negative y-direction)
    // If Pitch torque is positive (ie. leaning back) we want hips to go forward (ie. positive x-direction)
    // Get TFs for feet
    Eigen::Isometry3d footTF[2];
    hubo.huboLegFK( footTF[LEFT], qPrev[LEFT], LEFT ); 
    hubo.huboLegFK( footTF[RIGHT], qPrev[RIGHT], RIGHT );

    if(counter > 40)
        std::cout << " now " << footTF[LEFT](2,3);

    //-------------------------
    //   FORCE/TORQUE ERROR
    //-------------------------
    // Averaged torque error in ankles (roll and pitch) (yaw is always zero)
    //FIXME The version below is has elem.torques negative b/c hubomz computes reaction torque at ankle
    // instead of torque at F/T sensor
    Eigen::Vector3d forceTorqueErr[2];

    forceTorqueErr[LEFT](0) = (-elem.torque[LEFT][0] - hubo.getLeftFootMx());
    forceTorqueErr[LEFT](1) = (-elem.torque[LEFT][1] - hubo.getLeftFootMy());
    forceTorqueErr[LEFT](2) = (-elem.forces[LEFT][2] - hubo.getLeftFootFz()); //FIXME should be positive
    
    forceTorqueErr[RIGHT](0) = (-elem.torque[RIGHT][0] - hubo.getRightFootMx());
    forceTorqueErr[RIGHT](1) = (-elem.torque[RIGHT][1] - hubo.getRightFootMy());
    forceTorqueErr[RIGHT](2) = (-elem.forces[RIGHT][2] - hubo.getRightFootFz()); //FIXME should be positive

    // Skew matrix for torque reaction logic
    Eigen::Matrix3d skew; 
    skew << 0, 1, 0,
           -1, 0, 0,
            0, 0, 1; //FIXME should be negative
    skew(0,1) = 0;
    skew(1,0) = 0;
    //------------------------
    //  IMPEDANCE CONTROLLER
    //------------------------
    // Check if we're on the ground, if not set instantaneous feet offset
    // to zero so integrated feet offset doesn't change, but we still apply it.
    const double forceThreshold = 0;//20; // Newtons
    if(hubo.getLeftFootFz() + hubo.getRightFootFz() > forceThreshold)
    {
        if(LEFT == side || RIGHT == side)
            impCtrl.run(state.dFeetOffset, yawRot[side]*skew*forceTorqueErr[side], dt);
        else
            impCtrl.run(state.dFeetOffset, (yawRot[LEFT]*skew*forceTorqueErr[LEFT] + yawRot[RIGHT]*skew*forceTorqueErr[RIGHT])/2, dt);
    }
    else
    {
        // Don't add to the dFeetOffset
    }

    // Decay the dFeetOffset
//    state.dFeetOffset -= gains.decay_gain[LEFT]*state.dFeetOffset;

    //------------------------
    //    CAP BODY OFFSET
    //------------------------
    const double dFeetOffsetTol = 0.06;
    double n = state.dFeetOffset.norm();
    if (n > dFeetOffsetTol) {
      state.dFeetOffset *= dFeetOffsetTol/n;
    }

    //------------------------
    //    ADJUST FEET TFs
    //------------------------
    // Pretranslate feet TF by integrated feet error translation vector
    Eigen::Isometry3d tempFootTF[2];
    tempFootTF[LEFT] = footTF[LEFT].pretranslate(state.dFeetOffset.block<3,1>(0,0));
    tempFootTF[RIGHT] = footTF[RIGHT].pretranslate(state.dFeetOffset.block<3,1>(0,0));

    //------------------------
    //   GET NEW LEG ANGLES
    //------------------------
    // Run IK on the adjusted feet TF to get new joint angles
    bool ok = false;
    // Check IK for each new foot TF. If either fails, use previous feet TF
    // New joint angles for both legs
    Vector6d qNew[2];
    ok = hubo.huboLegIK(qNew[LEFT], tempFootTF[LEFT], qPrev[LEFT], LEFT);
    if(ok)
    {
        ok = hubo.huboLegIK(qNew[RIGHT], tempFootTF[RIGHT], qPrev[RIGHT], RIGHT);
        state.prevdFeetOffset = state.dFeetOffset;
    }
    else // use previous integrated feet offset to get joint angles
    {
        std::cout << "IK Failed in impedance controller. Using previous feet TF.\n";
        // Pretranslate feet TF by integrated feet error translation vector
        footTF[LEFT].pretranslate(state.prevdFeetOffset.block<3,1>(0,0));
        footTF[RIGHT].pretranslate(state.prevdFeetOffset.block<3,1>(0,0));
        hubo.huboLegIK(qNew[LEFT], footTF[LEFT], qPrev[LEFT], LEFT);
        hubo.huboLegIK(qNew[RIGHT], footTF[RIGHT], qPrev[RIGHT], RIGHT);
    }

    hubo.huboLegFK( footTF[LEFT], qNew[LEFT], LEFT ); 
    if(counter > 40)
        std::cout << " aft " << footTF[LEFT](2,3);

    //----------------------
    //   DEBUG PRINT OUT
    //----------------------
    if(counter > 40)
    {
    if(true)
    {
        std::cout //<< " K: " << kP
                  //<< " TdL: " << -elem.torque[LEFT][0] << ", " << -elem.torque[LEFT][1]
                  //<< " TdR: " << -elem.torque[RIGHT][0] << ", " << -elem.torque[RIGHT][1]
                  //<< " MyLR: " << hubo.getLeftFootMy() << ", " << hubo.getRightFootMy()
                  //<< " MxLR: " << hubo.getLeftFootMx() << ", " << hubo.getRightFootMx()
                  << " mFz: " << hubo.getLeftFootFz()
                  << " dFz: " << -elem.forces[LEFT][2]
                  << " FTe: " << forceTorqueErr[LEFT].z()
                  //<< " Fte: " << instantaneousFeetOffset.transpose()
                  << " FeetE: " << state.dFeetOffset(2)
                  << " qDfL: " << (qNew[LEFT] - qPrev[LEFT]).transpose()
                  << "\n";
    }
    }
    //-----------------------
    //   SET JOINT ANGLES
    //-----------------------
    // Set leg joint angles for current timestep of trajectory
    {
        elem.angles[LHY] = qNew[LEFT](HY);
        elem.angles[LHR] = qNew[LEFT](HR);
        elem.angles[LHP] = qNew[LEFT](HP);
        elem.angles[LKN] = qNew[LEFT](KN);
        elem.angles[LAP] = qNew[LEFT](AP);
        elem.angles[LAR] = qNew[LEFT](AR);

        elem.angles[RHY] = qNew[RIGHT](HY);
        elem.angles[RHR] = qNew[RIGHT](HR);
        elem.angles[RHP] = qNew[RIGHT](HP);
        elem.angles[RKN] = qNew[RIGHT](KN);
        elem.angles[RAP] = qNew[RIGHT](AP);
        elem.angles[RAR] = qNew[RIGHT](AR);
    }
    if(counter > 40)
        counter = 0;
}




Walker::Walker(double maxInitTime, double jointSpaceTolerance, double jointVelContinuityTolerance) :
        m_maxInitTime(maxInitTime),
        m_jointSpaceTolerance( jointSpaceTolerance ),
        m_jointVelContTol( jointVelContinuityTolerance ),
        keepWalking(true),
        hubo(),
        kin(),
        impCtrl(kin.mass()),
        counter(0)
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
 
    zmp_traj_t *prevTrajectory, *currentTrajectory, *nextTrajectory;
    prevTrajectory = new zmp_traj_t;
    currentTrajectory = new zmp_traj_t;
    nextTrajectory = new zmp_traj_t;

    memset( prevTrajectory, 0, sizeof(*prevTrajectory) );
    memset( currentTrajectory, 0, sizeof(*currentTrajectory) );
    memset( nextTrajectory, 0, sizeof(*nextTrajectory) );
    
    // TODO: Consider making these values persistent
    memset( &state, 0, sizeof(state) );


    memcpy( &bal_state, &parent_state, sizeof(bal_state) );

    bal_state.m_balance_mode = BAL_ZMP_WALKING; 
    bal_state.m_walk_mode = WALK_WAITING;
    bal_state.m_walk_error = NO_WALK_ERROR;
    sendState();

    currentTrajectory->reuse = true;
    fprintf(stdout, "Waiting for first trajectory\n"); fflush(stdout);
    ach_status_t r;
    do {
        struct timespec t;
        clock_gettime( ACH_DEFAULT_CLOCK, &t );
        t.tv_sec += 1;
        r = ach_get( &zmp_chan, currentTrajectory, sizeof(*currentTrajectory), &fs,
                    &t, ACH_O_WAIT | ACH_O_LAST );

        checkCommands();
        if( cmd.cmd_request != BAL_ZMP_WALKING )
            keepWalking = false;
    } while(!daemon_sig_quit && keepWalking && r==ACH_TIMEOUT);

    if(!keepWalking) // TODO: Take out the reuse condition here
    {
        bal_state.m_walk_mode = WALK_INACTIVE;
        sendState();
        return;
    }

    if(!daemon_sig_quit)
        fprintf(stdout, "First trajectory acquired\n");
    
        
    daemon_assert( !daemon_sig_quit, __LINE__ );

    bal_state.m_walk_mode = WALK_INITIALIZING;
    sendState();

    // Get the balancing gains from the ach channel
    ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );

    hubo.update(true);

    // Set all the joints to the initial posiiton in the trajectory
    // using the control daemon to interpolate in joint space.
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        // Don't worry about where these joint are
        if( LF1!=i && LF2!=i && LF3!=i && LF4!=i && LF5!=i
         && RF1!=i && RF2!=i && RF3!=i && RF4!=i && RF5!=i
         && NK1!=i && NK2!=i && NKY!=i && LWR!=i && RWR!=i && RWY!=i && RWP!=i) //FIXME
        {
            hubo.setJointAngle( i, currentTrajectory->traj[0].angles[i] );
            hubo.setJointNominalSpeed( i, 0.4 );
            hubo.setJointNominalAcceleration( i, 0.4 );
        }
    }
    
    hubo.setJointNominalSpeed( RKN, 0.8 );
    hubo.setJointNominalAcceleration( RKN, 0.8 );
    hubo.setJointNominalSpeed( LKN, 0.8 );
    hubo.setJointNominalAcceleration( LKN, 0.8 );

    hubo.sendControls();

    // Wait specified time for joints to get into initial configuration,
    // otherwise time out and alert user.
    double m_maxInitTime = 10;
    double biggestErr = 0;
    int worstJoint=-1;
    
    double dt, time, stime; stime=hubo.getTime(); time=hubo.getTime();
    double norm = m_jointSpaceTolerance+1; // make sure this fails initially
    while( !daemon_sig_quit && (norm > m_jointSpaceTolerance && time-stime < m_maxInitTime)) {
//    while(false) { // FIXME TODO: SWITCH THIS BACK!!!
        hubo.update(true);
        norm = 0;
        for(int i=0; i<HUBO_JOINT_COUNT; i++)
        {
            double err=0;
            // Don't worry about waiting for these joints to get into position.
            if( LF1!=i && LF2!=i && LF3!=i && LF4!=i && LF5!=i
             && RF1!=i && RF2!=i && RF3!=i && RF4!=i && RF5!=i
             && NK1!=i && NK2!=i && NKY!=i && LWR!=i && RWR!=i && RWY!=i && RWP!=i) //FIXME
                err = (hubo.getJointAngleState( i )-currentTrajectory->traj[0].angles[i]);

            norm += err*err;
            if( fabs(err) > fabs(biggestErr) )
            {
                biggestErr = err;
                worstJoint = i;
            }
        }
        time = hubo.getTime();
    }
    // Print timeout error if joints don't get to initial positions in time
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
    if( keepWalking )
        fprintf(stdout, "Beginning main walking loop\n"); fflush(stdout);
    while(keepWalking && !daemon_sig_quit)
    {
        haveNewTrajectory = checkForNewTrajectory(*nextTrajectory, haveNewTrajectory);
        ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );
        hubo.update(true);

        bal_state.m_walk_mode = WALK_IN_PROGRESS;

        dt = hubo.getTime() - time;
        time = hubo.getTime();
        if( dt <= 0 )
        {
            fprintf(stderr, "Something unnatural has happened in the Walker... %f\n", dt);
            continue;
        }

        if( timeIndex==0 )
        {
            bal_state.m_walk_error = NO_WALK_ERROR;
            nextTimeIndex = timeIndex+1;
            executeTimeStep( hubo, prevTrajectory->traj[prevTimeIndex],
                                   currentTrajectory->traj[timeIndex],
                                   currentTrajectory->traj[nextTimeIndex],
                                   state, gains, dt );
            
        }
        else if( timeIndex == currentTrajectory->periodEndTick && haveNewTrajectory )
        {
            if( validateNextTrajectory( currentTrajectory->traj[timeIndex],
                                        nextTrajectory->traj[0], dt ) )
            {
                nextTimeIndex = 0;
                executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
                                       currentTrajectory->traj[timeIndex],
                                       nextTrajectory->traj[nextTimeIndex],
                                       state, gains, dt );
                
                memcpy( prevTrajectory, currentTrajectory, sizeof(*prevTrajectory) );
                memcpy( currentTrajectory, nextTrajectory, sizeof(*nextTrajectory) );
                fprintf(stderr, "Notice: Swapping in new trajectory\n");
            }
            else
            {
                fprintf(stderr, "WARNING: Discontinuous trajectory passed in. Walking to a stop.\n");
                bal_state.m_walk_error = WALK_FAILED_SWAP;

                nextTimeIndex = timeIndex+1;
                executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
                                       currentTrajectory->traj[timeIndex],
                                       currentTrajectory->traj[nextTimeIndex],
                                       state, gains, dt );
            }
            haveNewTrajectory = false;
        }
        else if( timeIndex == currentTrajectory->periodEndTick && currentTrajectory->reuse )
        {
            checkCommands();
            if( cmd.cmd_request != BAL_ZMP_WALKING )
                currentTrajectory->reuse = false;

            if( currentTrajectory->reuse == true )
                nextTimeIndex = currentTrajectory->periodStartTick;
            else
                nextTimeIndex = timeIndex+1;

            executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
                                   currentTrajectory->traj[timeIndex],
                                   currentTrajectory->traj[nextTimeIndex],
                                   state, gains, dt );
        }
        else if( timeIndex < currentTrajectory->count-1 )
        {
            nextTimeIndex = timeIndex+1;
            executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
                                   currentTrajectory->traj[timeIndex],
                                   currentTrajectory->traj[nextTimeIndex],
                                   state, gains, dt );
        }
        else if( timeIndex == currentTrajectory->count-1 && haveNewTrajectory )
        {
            checkCommands();
            if( cmd.cmd_request != BAL_ZMP_WALKING )
                keepWalking = false;

            if( keepWalking )
            {
                if( validateNextTrajectory( currentTrajectory->traj[timeIndex],
                                            nextTrajectory->traj[0], dt ) )
                {
                    bal_state.m_walk_error = NO_WALK_ERROR;
                    nextTimeIndex = 0;
                    executeTimeStep( hubo, currentTrajectory->traj[prevTimeIndex],
                                           currentTrajectory->traj[timeIndex],
                                           nextTrajectory->traj[nextTimeIndex],
                                           state, gains, dt );
                    
                    memcpy( prevTrajectory, currentTrajectory, sizeof(*prevTrajectory) );
                    memcpy( currentTrajectory, nextTrajectory, sizeof(*nextTrajectory) );
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
    // Make copy of zmp_traj_element so we don't effect the trajectory that's
    // being recycled or it will be like recycling a changing trajectory. Not Good!
    zmp_traj_element_t tempNextElem;
    memcpy(&tempNextElem, &nextElem, sizeof(zmp_traj_element_t));

//    int legidx[6] = { LHY, LHR, LHP, LKN, LAP, LAR };
    
//    std::cout << "before: ";
//    for (int i=0; i<6; ++i) { std::cout << tempNextElem.angles[legidx[i]] << " "; }
//    std::cout << "\n";

    //flattenFoot( hubo, nextElem, state, gains, dt );
    //straightenBack( hubo, nextElem, state, gains, dt );
    //complyKnee( hubo, tempNextElem, state, gains, dt );
    //nudgeRefs( hubo, nextElem, state, dt, hkin ); //vprev, verr, dt );

//    std::cout << "after: ";
//    for (int i=0; i<6; ++i) { std::cout << tempNextElem.angles[legidx[i]] << " "; }
//    std::cout << "\n";

    // For each joint set it's position to that in the trajectory for the
    // current timestep, which has been adjusted based on feedback.
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        hubo.passJointAngle( i, tempNextElem.angles[i] );

    }

    hubo.setJointAngleMin( LHR, currentElem.angles[RHR]-M_PI/2.0 );
    hubo.setJointAngleMax( RHR, currentElem.angles[LHR]+M_PI/2.0 );

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

