
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

#include "DrcHuboKin.h"
#include "balance-daemon.h"
#include "Walker.h"
#include "Hubo_Control.h"
#include "manip.h"

ach_channel_t bal_cmd_chan;
ach_channel_t bal_state_chan;
ach_channel_t bal_param_chan;
ach_channel_t manip_override_chan;
ach_channel_t manip_state_chan;

void fillArmStates(Hubo_Control &hubo, balance_state_t &bal_state);

/**
 * \brief Balance while not moving the legs (in a static lower body pose)
 * using the Velocity mode in the control daemon
 * by resisting the roll and pith angles in the IMU in order to keep the
 * back upright, and complying with the roll and pitch torques in the ankles
 * in order to flatten the feet. There is also a term that adds velocity to
 * the hips in order to counter falling due to external forces.
 * \param hubo Hubo_Control object to get sensor values from.
 * \param cmd Balance command from rviz hubo_walk panel
 * \param gains Balance gains.
 * \param dt Cycle time.
 * \return void
*/
void staticBalance(Hubo_Control &hubo, DrcHuboKin &kin, balance_cmd_t &cmd, balance_gains_t &gains, double dt);

/**
 * \brief Takes the torque in the ankles and drives them to zero by
 * moving the hips at a velocity proportional to the torque error.
 * \param hubo Hubo_Control object to get sensor values from.
 * \param legJointVels Leg joint velocities to be filled in by reference
 * using the hip velocity IK function in Hubo_Control.
 * \param gains Balance gains.
 * \param dt Cycle time.
 * \return void
*/
void moveHips(Hubo_Control &hubo, DrcHuboKin &kin, std::vector<LegVector, Eigen::aligned_allocator<LegVector> > &legJointVels,
                balance_cmd_t &cmd, const balance_gains_t &gains, const double dt); 


int main(int argc, char **argv)
{
    Hubo_Control hubo("balance-daemon", 35);
    //Hubo_Control hubo;

    DrcHuboKin kin;

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
            fprintf(stderr, "Something unnatural has happened in the balance daemon... %f\n", dt);
            continue;
        }

        ach_get( &bal_cmd_chan, &cmd, sizeof(cmd), &fs, NULL, ACH_O_LAST );
        ach_get( &bal_param_chan, &gains, sizeof(gains), &fs, NULL, ACH_O_LAST );

        state.m_balance_mode = cmd.cmd_request;

        
        if( BAL_LEGS_ONLY == cmd.cmd_request )
        {
            staticBalance(hubo, kin, cmd, gains, dt);
        }
        else if( BAL_ZMP_WALKING == cmd.cmd_request )
        {
            ach_get( &manip_state_chan, &manip_state, sizeof(manip_state),
                        &fs, NULL, ACH_O_LAST );

            if( OVR_SOVEREIGN == manip_state.override )
            {
                ovr.m_override = OVR_ACQUIESCENT;
                ach_put( &manip_override_chan, &ovr, sizeof(ovr) );

                //staticBalance(hubo, kin, cmd, gains, dt);
            }
            else if( OVR_ACQUIESCENT == manip_state.override )
            {
                state.m_zmp_arm_states.should_use = cmd.use_cur_arm_positions;
                fillArmStates(hubo, state);
                // Put current arm angles on ach channel
                ach_put( &bal_state_chan, &state, sizeof(state) );
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


void fillArmStates(Hubo_Control &hubo, balance_state_t & bal_state)
{
    ArmVector armStates[2];

    for(int side=0; side<2; side++)
    {
        hubo.getArmAngles(side, armStates[side]);
        for(int joint=0; joint<ARM_JOINT_COUNT; joint++)
        {
            bal_state.m_zmp_arm_states.arm_joint_states[side][joint] = armStates[side](joint);
        }
    }

    bal_state.m_zmp_arm_states.num_arm_joints = ARM_JOINT_COUNT;
}


void staticBalance(Hubo_Control &hubo, DrcHuboKin &kin, balance_cmd_t &cmd, balance_gains_t &gains, double dt)
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


    double L1 = 0.33008 + 0.32995; //2*0.3002;
    double L2 = 0.28947 + 0.0795;
    
    if( cmd.height-L2 > L1 )
        cmd.height = L1+L2;
    else if( cmd.height-L2 < 0.25 ) //TODO: Don't hard code this
        cmd.height = L1+0.2;

    double knee = acos( (cmd.height-L2)/L1 )*2;

    // Initialize LegVectors for leg joint velocities,
    // which get passed into the hipVelocityIK() function
    std::vector<LegVector, Eigen::aligned_allocator<LegVector> > legJointVels(2);
    for(int side=0; side<2; side++)
        legJointVels[side].setZero();

    // Get leg joint velocities based that move the 
    // hips in the x-y plane to counter falling
    moveHips( hubo, kin, legJointVels, cmd, gains, dt );

    double kneeAngleErrorL = knee - hubo.getJointAngle( LKN );
    double kneeAngleErrorR = knee - hubo.getJointAngle( RKN );

    double kneeVelL = gains.spring_gain[LEFT]*kneeAngleErrorL + legJointVels[LEFT](KN);
    double kneeVelR = gains.spring_gain[RIGHT]*kneeAngleErrorR + legJointVels[RIGHT](KN);

    double pitchL = gains.straightening_pitch_gain[LEFT]*hubo.getAngleY()
                    + gains.flattening_gain[LEFT]*hubo.getLeftFootMy()    //FOR CORRECT F/T DIRECTIONS
                    - kneeVelL/2;
    double rollL  = gains.straightening_roll_gain[LEFT]*hubo.getAngleX()
                    + gains.flattening_gain[LEFT]*hubo.getLeftFootMx();    //FOR CORRECT F/T DIRECTIONS
    
    double pitchR = gains.straightening_pitch_gain[RIGHT]*hubo.getAngleY()
                    + gains.flattening_gain[RIGHT]*hubo.getRightFootMy()    //FOR CORRECT F/T DIRECTIONS
                    - kneeVelR/2;
    double rollR  = gains.straightening_roll_gain[RIGHT]*hubo.getAngleX()
                    + gains.flattening_gain[RIGHT]*hubo.getRightFootMx();    //FOR CORRECT F/T DIRECTIONS

    hubo.setJointVelocity( LAP, pitchL + legJointVels[LEFT](AP));
    hubo.setJointVelocity( LAR, rollL + legJointVels[LEFT](AR));
    hubo.setJointVelocity( LKN, kneeVelL );
    hubo.setJointVelocity( LHP, -kneeVelL/2.0 + legJointVels[LEFT](HP));
    hubo.setJointVelocity( LHR, legJointVels[LEFT](HR));

    hubo.setJointVelocity( RAP, pitchR + legJointVels[RIGHT](AP));
    hubo.setJointVelocity( RAR, rollR + legJointVels[RIGHT](AR));
    hubo.setJointVelocity( RKN, kneeVelR );
    hubo.setJointVelocity( RHP, -kneeVelR/2.0 + legJointVels[RIGHT](HP));
    hubo.setJointVelocity( RHR, legJointVels[RIGHT](HR));

    hubo.sendControls();
}


void moveHips( Hubo_Control &hubo, DrcHuboKin &kin, std::vector<LegVector, Eigen::aligned_allocator<LegVector> > &legJointVels,
                balance_cmd_t &cmd, const balance_gains_t &gains, const double dt )
{
    //---------------------------
    //       P & D Gains
    //---------------------------
    double kP = gains.double_support_hip_nudge_kp;
    double kD = gains.double_support_hip_nudge_kd;
    // Proportional gain matrix for ankle roll and pitch
    Eigen::Matrix3d shiftGainsKp;
    shiftGainsKp << kP,  0, 0,
                     0, kP, 0,
                     0,  0, 0;

    // Derivative gain matrix for ankle roll and pitch
    Eigen::Matrix3d shiftGainsKd;
    shiftGainsKd << kD,  0, 0,
                     0, kD, 0,
                     0,  0, 0;

    //---------------------------
    //    HIP YAW ROTATIONS
    //---------------------------
    // Get rotation matrix for each hip yaw
    Eigen::Matrix3d yawRot[2];
    yawRot[LEFT] = Eigen::AngleAxisd(hubo.getJointAngle(LHY), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    yawRot[RIGHT]= Eigen::AngleAxisd(hubo.getJointAngle(RHY), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    //---------------------------
    //       TORQUE ERROR
    //---------------------------
    // Averaged torque error in ankles (roll and pitch) (yaw is always zero)
    Eigen::Vector3d desiredForceTorque[2];
    for(int side; side<2; side++)
    {
        desiredForceTorque[side].setZero();
        desiredForceTorque[side].y() = -cmd.com_x_offset*kin.mass()*9.81; // torque about y-axis = com_x_offset * weight.
    }

    // Ankle torque error XYZ (ie. Roll/Pitch/Yaw), but just setting Z to zero.
    Vector3d torqueErr[2];

    // So if we are leaning backwards, causing a positive torque about y, and our desired torque is zero,
    // +M - 0 = +M, so we have a positive hip velocity term, which will nudge our hips forward and reduce
    // the torque.
    // If we are leaning left, causing a positive torque about x, and our desired torque is zero,
    // 0 - M = -M, so we have a negative hip velocity term, which will nudge our hips to the right and
    // reduce the torque to zero.
    torqueErr[LEFT].x() = desiredForceTorque[LEFT].x() - hubo.getLeftFootMx();
    torqueErr[LEFT].y() = desiredForceTorque[LEFT].y() - hubo.getLeftFootMy();
    torqueErr[LEFT].z() = desiredForceTorque[LEFT].z();
    
    torqueErr[RIGHT].x() = desiredForceTorque[RIGHT].x() - hubo.getRightFootMx();
    torqueErr[RIGHT].y() = desiredForceTorque[RIGHT].y() - hubo.getRightFootMy();
    torqueErr[RIGHT].z() = desiredForceTorque[RIGHT].z();

    //---------------------------
    //       HIP VELOCITY
    //---------------------------
    Eigen::Vector3d hipVelocity;//!< New hip velocity to counter falling 

    // Skew matrix to convert torque rotation to hip velocity direction.
    // Positive torque about x-axis should translate to negative y velocity
    // Postiive torque about y-axis should translate to positive x velocity
    Eigen::Matrix3d skew; 
    skew << 0,-1, 0,
            1, 0, 0,
            0, 0, 0;

    // Check if we're on the ground, if not set hipVelocity to zero.
    // If we are, apply torque error time a gain to our hip velocity.
//    const double forceThreshold = 20; // Newtons
//    if(hubo.getLeftFootFz() + hubo.getRightFootFz() > forceThreshold)
//    {
        hipVelocity = (shiftGainsKp * (yawRot[LEFT]*skew*torqueErr[LEFT] + yawRot[RIGHT]*skew*torqueErr[RIGHT])/2);
//                                 - (shiftGainsKd * (yawRot[LEFT]*(torqueErr[LEFT] - state.prevTorqueErr[LEFT])
//                                    + yawRot[RIGHT]*(torqueErr[RIGHT] - state.prevTorqueErr[RIGHT]))/2/dt);
//    }
//    else
//        hipVelocity.setZero();

    //---------------------------
    //   LEG JOINT VELOCITIES
    //---------------------------
    for(int side=0; side<2; side++)
    {
        hubo.hipVelocityIK(legJointVels[side], hipVelocity, side);
    }

    //---------------------------
    //       PRINT OUT
    //---------------------------
    if(false)
    {
        std::cout 
//                  << "MyLR: " << hubo.getLeftFootMy() << ", " << hubo.getRightFootMy()
//                  << "\tMxLR: " << hubo.getLeftFootMx() << ", " << hubo.getRightFootMx()
//                  << "Te: " << torqueErr[LEFT].transpose() << ", " << torqueErr[RIGHT].transpose()
                  << "hipVel: " << hipVelocity.transpose()
                  << "\nqVel[L]: " << legJointVels[LEFT].transpose()
                  << "\nqVel[R]: " << legJointVels[RIGHT].transpose()
                  << "\n";
    }
}
