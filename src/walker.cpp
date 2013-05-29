
#include <Hubo_Control.h>
#include "walker.h"
#include "balance-daemon.h"

#define HAVE_HUBO_ACH
#include <hubo-zmp.h>

//#include "HuboKin.h"


ach_channel_t zmp_chan;
ach_channel_t param_chan;

const double jointSpaceTolerance = 0.02;
const double jointVelContTol = 6.0; // Joint trajectory velocity-based continuity tolerance

const double tau_dead_band = 1;

/*
const double nudgePGain = 0.04;
const double nudgeIGain = 0.2;
const double nudgeDGain = 0.0;
*/


/* Set by balance param struct now

const double hipDistance = 0.08843*2.0; // Distance between hip joints

// gain for ankle flat
const double k_tau = 0.0010; // Grey likes this value

// gain for moment reducing
const double k_pos = 0.0030; // Crane stance resistance to torque

// gain for IMU rotation
const double k_theta_x = 0;
const double k_theta_y = 0.5*M_PI/180.0; // IMU feedback gain

const double weight_thresh_N = -1e5; // TESTING k_tau 30; // weight to turn on/off integrators
const double nudge_max_norm = 0.05; // m
const double spin_max_angle = 30 * M_PI/180;
const double comp_max_angle = 30 * M_PI/180;



const double fzMin = 10;
const double fzMax = 50;

const double flatteningGain = 0.02;
*/



double applyDeadband( double x ) {

  if (x > tau_dead_band) {
    return x - tau_dead_band;
  } else if (x < -tau_dead_band) {
    return x + tau_dead_band;
  } else {
    return 0;
  }

}

static inline void clamp(double& x, double cap) {
  x = std::max(-cap, std::min(x, cap));
}


void executeTimeStep( Hubo_Control &hubo, zmp_traj_element &prevElem,
            zmp_traj_element_t &currentElem, zmp_traj_element &nextElem,
            nudge_state_t &state, balance_gains_t &gains, double dt );


bool checkForNewTrajectory(zmp_traj_t &newTrajectory, bool haveNewTrajAlready);
bool validateNextTrajectory( zmp_traj_element_t &current, zmp_traj_element_t &next, double dt );


void flattenFoot( Hubo_Control &hubo, zmp_traj_element_t &elem,
			nudge_state_t &state, balance_gains_t &gains, double dt )
{
    
//    std::cout << "RFz:" << hubo.getRightFootFz() << "RAP:" << state.ankle_pitch_compliance[RIGHT] << "\tLFz:" << hubo.getLeftFootFz() << "\tLAP:" << state.ankle_pitch_compliance[LEFT] << std::endl;

    if( gains.force_min_threshold[RIGHT] < hubo.getRightFootFz() 
     && hubo.getRightFootFz() < gains.force_max_threshold[RIGHT] )
    {
        state.ankle_roll_compliance[RIGHT] += dt*gains.flattening_gain[RIGHT]
                                                *( hubo.getRightFootMx() );
        state.ankle_pitch_compliance[RIGHT] += dt*gains.flattening_gain[RIGHT]
                                                 *( hubo.getRightFootMy() );
        std::cout << "Flattening Right Foot" << "\troll:" << state.ankle_roll_compliance[RIGHT]
                  << "\tpitch:" << state.ankle_pitch_compliance[RIGHT] << std::endl;
    }

    if( gains.force_min_threshold[LEFT] < hubo.getLeftFootFz()
     && hubo.getLeftFootFz() < gains.force_max_threshold[LEFT] )
    {
        state.ankle_roll_compliance[LEFT] += dt*gains.flattening_gain[LEFT]
                                               *( hubo.getLeftFootMx() );
        state.ankle_pitch_compliance[LEFT] += dt*gains.flattening_gain[LEFT]
                                                *( hubo.getLeftFootMy() );
        std::cout<< "Flattening Left Foot" << "\troll:" << state.ankle_roll_compliance[LEFT]
                  << "\tpitch:" << state.ankle_pitch_compliance[LEFT] << std::endl;
    }

    elem.angles[RAR] += state.ankle_roll_compliance[RIGHT];
    elem.angles[RAP] += state.ankle_pitch_compliance[RIGHT];
    elem.angles[LAR] += state.ankle_roll_compliance[LEFT];
    elem.angles[LAP] += state.ankle_pitch_compliance[LEFT];

}

void straightenBack( Hubo_Control &hubo, zmp_traj_element_t &elem,
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

void complyKnee( Hubo_Control &hubo, zmp_traj_element_t &elem,
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





int main(int argc, char **argv)
{
//    Hubo_Control hubo("walker");
    Hubo_Control hubo;

    int timeIndex=0, nextTimeIndex=0, prevTimeIndex=0;

    nudge_state_t state;
    balance_gains_t gains;
    memset( &state, 0, sizeof(nudge_state_t) );
    memset( &gains, 0, sizeof(balance_gains_t) );

    ach_status_t r = ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "%s (%d)\n", ach_result_to_string(r), (int)r );
    
    r = ach_open( &param_chan, BALANCE_PARAM_CHAN, NULL );
    if( r != ACH_OK )
        fprintf( stderr, "%s (%d)\n", ach_result_to_string(r), (int)r );
    
    size_t fs;
    zmp_traj_t prevTrajectory, currentTrajectory, nextTrajectory;
    memset( &prevTrajectory, 0, sizeof(prevTrajectory) );
    memset( &currentTrajectory, 0, sizeof(currentTrajectory) );
    memset( &nextTrajectory, 0, sizeof(nextTrajectory) );

    fprintf(stdout, "Waiting for first trajectory\n"); fflush(stdout);
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

    fprintf(stdout, "Count: %d\n", (int)currentTrajectory.count); fflush(stdout);

    ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );

    hubo.update(true);
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        hubo.setJointAngle( i, currentTrajectory.traj[0].angles[i] );
        hubo.setJointNominalSpeed( i, 0.4 );
        hubo.setJointNominalAcceleration( i, 0.4 );
    }
    
    hubo.setJointAngle(RSP, 0.34);
    hubo.setJointNominalSpeed( RKN, 0.8 );
    hubo.setJointNominalAcceleration( RKN, 0.8 );
    hubo.setJointNominalSpeed( LKN, 0.8 );
    hubo.setJointNominalAcceleration( LKN, 0.8 );

    hubo.setJointAngle( RSR, currentTrajectory.traj[0].angles[RSR] + hubo.getJointAngleMax(RSR) );
    hubo.setJointAngle( LSR, currentTrajectory.traj[0].angles[LSR] + hubo.getJointAngleMin(LSR) );

    hubo.sendControls();

    double maxWait = 10;
    double biggestErr = 0;
    int worstJoint=-1;
    
    double dt, time, stime; stime=hubo.getTime(); time=hubo.getTime();
    double norm = jointSpaceTolerance+1; // make sure this fails initially
    while( !daemon_sig_quit && (norm > jointSpaceTolerance && time-stime < maxWait)) {
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
        norm = sqrt(norm); std::cout << "norm:" << norm << "\tbiggest error:" << biggestErr << std::endl;
        time = hubo.getTime();
    }

    if( time-stime >= maxWait )
        fprintf(stderr, "Warning: could not reach the starting Trajectory within 15 seconds\n"
                        " -- Biggest error was %f radians in joint %s\n", biggestErr, jointNames[worstJoint] );
    daemon_assert( time-stime < maxWait, __LINE__ );

    timeIndex = 1;
    fprintf(stdout, "Beginning main loop\n"); fflush(stdout);
    while(!daemon_sig_quit)
    {
        bool haveNewTrajectory = checkForNewTrajectory(nextTrajectory, haveNewTrajectory);
        ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );
        hubo.update(true);
        dt = hubo.getTime() - time;
        time = hubo.getTime();

        if( timeIndex==0 )
        {
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
            nextTimeIndex = currentTrajectory.periodStartTick;
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
            }
            else
                fprintf(stderr, "WARNING: Discontinuous trajectory passed in. Discarding it.\n");

            haveNewTrajectory = false;
        }


        prevTimeIndex = timeIndex;
        timeIndex = nextTimeIndex;
    }

}




bool checkForNewTrajectory(zmp_traj_t &newTrajectory, bool haveNewTrajAlready)
{
    size_t fs;
    
    ach_status_t r = ach_get( &zmp_chan, &newTrajectory, sizeof(newTrajectory), &fs, NULL, ACH_O_LAST );

    if( ACH_OK==r || ACH_MISSED_FRAME==r )
        return true;
    else
        return haveNewTrajAlready || false;

}

bool validateNextTrajectory( zmp_traj_element_t &current, zmp_traj_element_t &next, double dt )
{
    bool valid = true;
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        if( fabs(next.angles[i]-current.angles[i])/fabs(dt) > fabs(jointVelContTol) )
            valid = false;
    return valid;
}


void executeTimeStep( Hubo_Control &hubo, zmp_traj_element_t &prevElem,
            zmp_traj_element_t &currentElem, zmp_traj_element &nextElem,
            nudge_state_t &state, balance_gains_t &gains, double dt )
{
    flattenFoot( hubo, currentElem, state, gains, dt );
    straightenBack( hubo, currentElem, state, gains, dt );
    complyKnee( hubo, currentElem, state, gains, dt );
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
