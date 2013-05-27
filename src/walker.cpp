
#include <Hubo_Control.h>
#include "walker.h"
#include "balance-daemon.h"

#define HAVE_HUBO_ACH
#include <hubo-zmp.h>

//#include "HuboKin.h"


ach_channel_t zmp_chan;
ach_channel_t param_chan;



/*
const double nudgePGain = 0.04;
const double nudgeIGain = 0.2;
const double nudgeDGain = 0.0;
*/

const double jointSpaceTolerance = 0.075;

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

const double tau_dead_band = 1;


const double fzMin = 10;
const double fzMax = 50;

const double flatteningGain = 0.02;

// Stance Controllers
void calibrateBoth( Hubo_Control &hubo );
void horseStance( Hubo_Control &hubo );
void craneStance( int side, Hubo_Control &hubo, double dt );
void craneStance( int side, Vector6d swingVels, Hubo_Control &hubo, double dt );


// Horse Stance Quasi-Statics
bool shiftToDistribution( int side, double distro, Hubo_Control &hubo, Balance_Monitor &trans, double dt );
bool shiftToSide( int side, Hubo_Control &hubo, Balance_Monitor &trans, double dt );
bool crouch( double height, Hubo_Control &hubo, double dt );


// Crane Stance Quasi-Statics
bool placeSwingFoot( int side, Eigen::Vector3d footPose, Hubo_Control &hubo, double dt );


//const double compPitchGain = 0.0015;
//const double shiftCompMultiplier = 1.0;
//const double craneCompMultiplier = 1.0;


// TODO: resurrect this later when roll ankle limits give us trouble
//
//const double craneHipPitchGain = 0.0*M_PI/180.0; // Not used
//const double craneHipRollGain  = 0.05*M_PI/180.0; // Used when ankle roll reaches joint limits
//const double craneAnkleRollThresh = 2.0*M_PI/180.0;

//const double craneHipSafetyMargin = 2.5*M_PI/180.0; // Buffer to prevent leg collisions during crane stance
//const double craneHipSafetyGain = 1.5;

//const double swingVelLimit = 0.05;

//const double rollAngleGain = 0.3*M_PI/180.0; // IMU feedback gain
//const double craneAngleMultiplier = 1.0;

//const double shiftGain = 0.0002; // Shift-to-side gain
//const double shiftBalanceGain = 0.0015; // Gain to maintain balance while shifting
//const double craneTransitionPercent = 98.0; // Percent of weight on swing foot before entering crane stance

//const double crouchGain = 0.30;  // Leg-lifting gain

//const double pcAccel = 0.4; // Nominal acceleration used for position control
//const double vcAccel = 1.5; // Nominal acceleration used for velocity control

//const double calKneeGain = 1.0;
//const double calWeightGain = 0.001; // rad/sec per N

double applyDeadband( double x ) {

  if (x > tau_dead_band) {
    return x - tau_dead_band;
  } else if (x < -tau_dead_band) {
    return x + tau_dead_band;
  } else {
    return 0;
  }

}


void getLegNudge( Hubo_Control &hubo, Vector6d q, 
		  zmp_traj_element_t elem, nudge_state_t &state, 
		  int side, double dt )
{
    Eigen::Vector3d vel; vel.setZero();

    double refAngleX=0, refAngleY=0;
    // TODO: Calculate desired IMU angle

    Eigen::Matrix3d R = ( Eigen::AngleAxisd(q(HY), Vector3d::UnitZ()) *
			  Eigen::AngleAxisd(q(HR), Vector3d::UnitX()) *
			  Eigen::AngleAxisd(q(HP), Vector3d::UnitY()) *
			  Eigen::AngleAxisd(q(KN), Vector3d::UnitY()) *
			  Eigen::AngleAxisd(q(AP), Vector3d::UnitY()) *
			  Eigen::AngleAxisd(q(AR), Vector3d::UnitX()) ).toRotationMatrix();

    /*
    std::cout << "left MX: " << hubo.getLeftFootMx() << ", "
	      << "desired: " << elem.torque[LEFT][0] << ", "
	      << "diff: " << (hubo.getLeftFootMx() - elem.torque[LEFT][0]) << "\n";

    std::cout << "left MY: " << hubo.getLeftFootMy() << ", "
	      << "desired: " << elem.torque[LEFT][1] << ", "
	      << "diff: " << (hubo.getLeftFootMy() - elem.torque[LEFT][1]) << "\n";

    std::cout << "right MX: " << hubo.getRightFootMx() << ", "
	      << "desired: " << elem.torque[RIGHT][0] << ", "
	      << "diff: " << (hubo.getRightFootMx() - elem.torque[RIGHT][0]) << "\n";

    std::cout << "right MY: " << hubo.getRightFootMy() << ", "
	      << "desired: " << elem.torque[RIGHT][1] << ", "
	      << "diff: " << (hubo.getRightFootMy() - elem.torque[RIGHT][1]) << "\n\n";
    */

    if( side==RIGHT )
    {
      vel(0) =  k_pos*applyDeadband( hubo.getRightFootMy() - elem.torque[RIGHT][1] );
      vel(1) = -k_pos*applyDeadband( hubo.getRightFootMx() - elem.torque[RIGHT][0] );


        state.nudge += R*vel*dt;

        state.spin(0) += dt*k_theta_x*(hubo.getAngleX()-refAngleX);//-state.imu_offset(0));
        state.spin(1) += dt*k_theta_y*(hubo.getAngleY()-refAngleY);//-state.imu_offset(0));

        state.ankle_roll_compliance[RIGHT] += dt*k_tau*( hubo.getRightFootMx() );
        state.ankle_pitch_compliance[RIGHT] += dt*k_tau*( hubo.getRightFootMy() );
    }
    else
    {
        vel(0) =  k_pos*applyDeadband( hubo.getLeftFootMy() - elem.torque[LEFT][1] );
        vel(1) = -k_pos*applyDeadband( hubo.getLeftFootMx() - elem.torque[LEFT][0] );

        state.nudge += R*vel*dt;

        state.spin(0) += dt*k_theta_x*(hubo.getAngleX()-refAngleX);//-state.imu_offset(0));
        state.spin(1) += dt*k_theta_y*(hubo.getAngleY()-refAngleY);//-state.imu_offset(0));

        state.ankle_roll_compliance[LEFT] += dt*k_tau*( hubo.getLeftFootMx() );
        state.ankle_pitch_compliance[LEFT] += dt*k_tau*( hubo.getLeftFootMy() );
    }



}

static inline void clamp(double& x, double cap) {
  x = std::max(-cap, std::min(x, cap));
}




void flattenFoot( Hubo_Control &hubo, zmp_traj_element_t &elem,
			nudge_state_t &state, balance_gains_t &gains, double dt )
{
    
//    std::cout << "RFz:" << hubo.getRightFootFz() << "RAP:" << state.ankle_pitch_compliance[RIGHT] << "\tLFz:" << hubo.getLeftFootFz() << "\tLAP:" << state.ankle_pitch_compliance[LEFT] << std::endl;

    if( fzMin < hubo.getRightFootFz() && hubo.getRightFootFz() < fzMax )
    {
        std::cout << "Flattening Right Foot" << std::endl;
        state.ankle_roll_compliance[RIGHT] += dt*gains.flattening_gain[RIGHT]
                                                *( hubo.getRightFootMx() );
        state.ankle_pitch_compliance[RIGHT] += dt*gains.flattening_gain[RIGHT]
                                                 *( hubo.getRightFootMy() );
    }

    if( fzMin < hubo.getLeftFootFz() && hubo.getLeftFootFz() < fzMax )
    {
        std::cout<< "Flattening Left Foot" << std::endl;
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

void straightenBack( Hubo_Control &hubo, zmp_traj_element_t &elem,
        nudge_state_t &state, balance_gains_t &gains, double dt )
{
//    if( 
    


}

void complyKnee( Hubo_Control &hubo, zmp_traj_element_t &elem,
        nudge_state_t &state, balance_gains_t &gains, double dt )
{



}


int main(int argc, char **argv)
{
    Hubo_Control hubo("walker");

//    fprintf(stdout, "Print something!!\n");

//    HK::HuboKin hkin;

//    hkin.kc.leg_l1 = 0; // eliminate neck -> waist Z distance
//    hkin.kc.leg_l3 = 0; // eliminate waist -> hip Z distance
//    hkin.kc.leg_l6 = 0; // eliminate ankle -> foot Z distance

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
    zmp_traj_t trajectory;
    memset( &trajectory, 0, sizeof(trajectory) );

    do {
        struct timespec t;
        clock_gettime( ACH_DEFAULT_CLOCK, &t );
        t.tv_sec += 1;
        ach_get( &zmp_chan, &trajectory, sizeof(trajectory), &fs, &t, ACH_O_WAIT | ACH_O_LAST );
    } while(!daemon_sig_quit && r==ACH_TIMEOUT);

    fprintf(stderr, "Count: %d\n", (int)trajectory.count);
    for(int i=0; i<trajectory.count; i++)
        fprintf(stdout, "%d: RHR %f\n", i, trajectory.traj[i].angles[RHR] );

    ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );

    hubo.update(true);
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        hubo.setJointAngle( i, trajectory.traj[0].angles[i] );
        hubo.setJointNominalSpeed( i, 0.4 );
        hubo.setJointNominalAcceleration( i, 0.4 );
    }

    hubo.setJointNominalSpeed( RKN, 0.8 );
    hubo.setJointNominalAcceleration( RKN, 0.8 );
    hubo.setJointNominalSpeed( LKN, 0.8 );
    hubo.setJointNominalAcceleration( LKN, 0.8 );

    hubo.setJointAngle( RSR, trajectory.traj[0].angles[RSR] + hubo.getJointAngleMax(RSR) );
    hubo.setJointAngle( LSR, trajectory.traj[0].angles[LSR] + hubo.getJointAngleMin(LSR) );

    hubo.sendControls();

    
    double dt, time, stime; stime=hubo.getTime(); time=hubo.getTime();
    double norm = jointSpaceTolerance+1; // make sure this fails initially
    while( !daemon_sig_quit && (norm > jointSpaceTolerance && time-stime < 15)) {
      hubo.update(true);
      norm = 0;
      for(int i=0; i<HUBO_JOINT_COUNT; i++)
          norm += (hubo.getJointAngleState( i )-trajectory.traj[0].angles[i])
                   *(hubo.getJointAngleState( i )-trajectory.traj[0].angles[i]);
      norm = sqrt(norm);
      time = hubo.getTime();
    }

    if( time-stime >= 15 )
        fprintf(stderr, "Warning: could not reach the starting trajectory within 15 seconds");


    while(!daemon_sig_quit)
    {


        fprintf(stdout, "%d\n", (int)trajectory.count);
        for(int t=1; t<trajectory.count-1; t++)
        {

            hubo.update(true);
            dt = hubo.getTime() - time;
            time = hubo.getTime();

            ach_get( &param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );

            flattenFoot( hubo, trajectory.traj[t], state, gains, dt );
            straightenBack( hubo, trajectory.traj[t], state, gains, dt );
            complyKnee( hubo, trajectory.traj[t], state, gains, dt );
            //nudgeRefs( hubo, trajectory.traj[t], state, dt, hkin ); //vprev, verr, dt );
            for(int i=0; i<HUBO_JOINT_COUNT; i++)
            {
                hubo.setJointAngle( i, trajectory.traj[t].angles[i] );
                hubo.setJointNominalSpeed( i,
                       (trajectory.traj[t].angles[i]-trajectory.traj[t-1].angles[i])*ZMP_TRAJ_FREQ_HZ );
                double accel = ZMP_TRAJ_FREQ_HZ*ZMP_TRAJ_FREQ_HZ*(
                                    trajectory.traj[t-1].angles[i]
                                - 2*trajectory.traj[t].angles[i]
                                +   trajectory.traj[t+1].angles[i] );
                hubo.setJointNominalAcceleration( i, 10*accel );
            }


            hubo.setJointAngle( RSR, trajectory.traj[t].angles[RSR] + hubo.getJointAngleMax(RSR) );
            hubo.setJointAngle( LSR, trajectory.traj[t].angles[LSR] + hubo.getJointAngleMin(LSR) );

            hubo.setJointAngleMin( LHR, trajectory.traj[t].angles[RHR] );
            hubo.setJointAngleMax( RHR, trajectory.traj[t].angles[LHR] );
            hubo.sendControls();
        }

        ach_status_t r = ACH_TIMEOUT;

        do {
            struct timespec t;
            clock_gettime( ACH_DEFAULT_CLOCK, &t );
            t.tv_sec += 1;
            r = ach_get( &zmp_chan, &trajectory, sizeof(trajectory), &fs,
                                      &t, ACH_O_WAIT | ACH_O_LAST );
        } while(!daemon_sig_quit && r==ACH_TIMEOUT);


    }









}
