
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
#include "DrcHuboKin.h"
#include "Walker.h"
#include "Hubo_Control.h"
#include "manip.h"

using namespace Eigen;

ach_channel_t bal_cmd_chan;
ach_channel_t bal_state_chan;
ach_channel_t bal_param_chan;
ach_channel_t manip_override_chan;
ach_channel_t manip_state_chan;
ach_channel_t crpc_param_chan;
ach_channel_t crpc_state_chan;

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


/**
 * \brief Adjusts posture based on force/torque and IMU sensors.
 * \param hubo Hubo_Control object to get sensor values from.
 * \param kin DrcHuboKin kinematics objects
 * \param cmd Balance command struct
 * \param crpc Crpc parameters struct
 * \param crpc_state Crpc state struct
 * \param offset Balance offsets for posture, balancing and walking
*/
void crpcPostureController(Hubo_Control &hubo, DrcHuboKin &kin, balance_cmd_t &cmd, crpc_params_t &crpc,
                           crpc_state_t &crpc_state, BalanceOffsets &offsets);


int main(int argc, char **argv)
{
    //Hubo_Control hubo("balance-daemon", 35);
    Hubo_Control hubo;

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
    
    r = ach_open( &crpc_param_chan, CRPC_PARAM_CHAN, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );

    r = ach_open( &crpc_state_chan, CRPC_STATE_CHAN, NULL );
    daemon_assert( r==ACH_OK, __LINE__ );

    Walker walk;

    balance_cmd_t cmd;
    balance_state_t state;
    balance_params_t params;
    nudge_state_t nudge;
    manip_override_t ovr;
    hubo_manip_state_t manip_state;
    crpc_params_t crpc;
    crpc_state_t crpc_state;
    
    BalanceOffsets offsets;

    memset( &cmd, 0, sizeof(cmd) );
    memset( &state, 0, sizeof(state) );
    memset( &params, 0, sizeof(params) );
    memset( &nudge, 0, sizeof(nudge) );
    memset( &ovr, 0, sizeof(ovr) );
    memset( &manip_state, 0, sizeof(manip_state) );
    memset( &crpc, 0, sizeof(crpc) );
    memset( &crpc_state, 0, sizeof(crpc_state) );
    
    crpc.kp_upper_body = 1e-2;
    crpc.kp_mass_distrib = 2e-7;
    crpc.kp_zmp_diff = 4e-3;
    crpc.kp_zmp_com = 1e-3;
    crpc.zmp_ref_x = 0;
    crpc.zmp_ref_y = 0;
    crpc.hip_crouch = 0.05;
    crpc.from_current_ref = false;
    
    
    ach_put(&crpc_param_chan, &crpc, sizeof(crpc));

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
        ach_get( &bal_param_chan, &params, sizeof(balance_gains), &fs, NULL, ACH_O_LAST );

        state.m_balance_mode = cmd.cmd_request;

        // if just balancing
        if( BAL_LEGS_ONLY == cmd.cmd_request )
        {
            staticBalance(hubo, kin, cmd, params.balance_gains, dt);
        }
        // if zmp walking
        else if( BAL_ZMP_WALKING == cmd.cmd_request )
        {
            ach_get( &manip_state_chan, &manip_state, sizeof(manip_state),
                        &fs, NULL, ACH_O_LAST );

            if( OVR_SOVEREIGN == manip_state.override )
            {
                ovr.m_override = OVR_ACQUIESCENT;
                ach_put( &manip_override_chan, &ovr, sizeof(ovr) );

                //staticBalance(hubo, kin, cmd, params.balance_gains, dt);
            }
            else if( OVR_ACQUIESCENT == manip_state.override )
            {
                walk.commenceWalking(state, nudge, params, offsets);
                ovr.m_override = OVR_SOVEREIGN;
                ach_put( &manip_override_chan, &ovr, sizeof(ovr) );
                // Probably not necessary...
                hubo.releaseUpperBody();
            }
        }
        // if running posture controller
        else if( BAL_CRPC == cmd.cmd_request )
        {
            // I don't see why we need to take control of the arms
//            ach_get( &manip_state_chan, &manip_state, sizeof(manip_state),
//                     &fs, NULL, ACH_O_LAST );

//            if( OVR_SOVEREIGN == manip_state.override )
//            {
//                ovr.m_override = OVR_ACQUIESCENT;
//                ach_put( &manip_override_chan, &ovr, sizeof(ovr) );
//            }
//            else if( OVR_ACQUIESCENT == manip_state.override )
//            {
//                //crpc.run();
//                ovr.m_override = OVR_SOVEREIGN;
//                ach_put( &manip_override_chan, &ovr, sizeof(ovr) );
//                hubo.releaseUpperBody();
//            }
            ach_get(&crpc_param_chan, &crpc, sizeof(crpc), &fs, NULL, ACH_O_LAST);
            crpcPostureController(hubo, kin, cmd, crpc, crpc_state, offsets);
        }
        else if( LOAD_CRPC == cmd.cmd_request )
        {
            offsets.loadCRPCFromText(cmd.filename);
        }
        else if( SAVE_CRPC == cmd.cmd_request )
        {
            offsets.saveCRPCToText(cmd.filename);
        }

        ach_put( &bal_state_chan, &state, sizeof(state) );

    }

    return 0;
}


void crpcPostureController(Hubo_Control &hubo, DrcHuboKin &kin, balance_cmd_t &cmd, crpc_params_t &crpc, crpc_state_t &crpc_state, BalanceOffsets &offsets)
{
    std::cout << "Running CRPC" << std::endl;

    if(!crpc.from_current_ref)
    {
        LegVector legSpeed; legSpeed.setOnes();
        legSpeed *= 0.4;
        legSpeed(KN) *= 2;
        
        hubo.setLegNomSpeeds(LEFT, legSpeed);
        hubo.setLegNomAcc(LEFT, legSpeed);
        hubo.setLegNomSpeeds(RIGHT, legSpeed);
        hubo.setLegNomAcc(RIGHT, legSpeed);
        
        
        LegVector q[2]; q[LEFT].setZero(); q[RIGHT].setZero();
        
        for(int side=0; side<2; side++)
        {
            kin.updateLegJoints(side, q[side]);
            RobotKin::TRANSFORM B = kin.legFK(side);
            B.pretranslate(Vector3d(0,0,crpc.hip_crouch));
            kin.legIK(side, q[side], B);
            hubo.setLegAngles(side, q[side]);
        }
        
        hubo.sendControls();
        
        LegVector qreal[2]; qreal[LEFT].setZero(); qreal[RIGHT].setZero();
        double max_time = 10, stime, time, tolerance = 0.0075;
        stime = hubo.getTime();
        
        do {
            
            hubo.update();
            time = hubo.getTime();
            for(int side=0; side<2; side++)
                hubo.getLegAngles(side, qreal[side]);
            
        } while(!daemon_sig_quit && time-stime < max_time
                    && ( (q[LEFT]-qreal[LEFT]).norm() > tolerance
                         || (q[RIGHT]-qreal[RIGHT]).norm() > tolerance )
                );
        
        if( time-stime >= max_time )
        {
            fprintf(stderr, "Warning: could not reach the initial stance in within %f seconds\n", max_time);
            cmd.cmd_request = BAL_READY;
            return;
        }
    }

    LegVector qReal[2];
    hubo.getLegAngles(LEFT, qReal[LEFT]);
    hubo.getLegAngles(RIGHT, qReal[RIGHT]);
    
    Isometry3d Bfoot[2];
    kin.updateHubo(hubo);
    Bfoot[LEFT] = kin.legFK(LEFT);
    Bfoot[RIGHT] = kin.legFK(RIGHT);
    Isometry3d center = Isometry3d::Identity();
    center.translate((Bfoot[LEFT].translation()+Bfoot[RIGHT].translation())/2);
    // TODO: Account for rotations of the feet
    Bfoot[LEFT] = center.inverse() * Bfoot[LEFT];
    Bfoot[RIGHT] = center.inverse() * Bfoot[RIGHT];
    
    double tau_sign = (crpc.negate_moments == 1) ? -1 : 1;
    
    for(int phase=1; phase<4; phase++)
    {
        int active_leg;
        bool do_upper_body;
        bool do_zmp_diff;
        bool do_zmp_com;
        bool do_mass_distrib = (crpc.kp_mass_distrib != 0);

        crpc_state.phase = (crpc_phase_t)phase;
        ach_put( &crpc_state_chan, &crpc_state, sizeof(crpc_state) );
        std::cout << "Phase " << phase << std::endl;
        
        switch(phase)
        {
        case CRPC_PHASE_1:
            active_leg = LEFT;
            do_upper_body = (crpc.kp_upper_body != 0);
            do_zmp_diff = false;
            do_zmp_com = false;
            break;
        case CRPC_PHASE_2:
            active_leg = RIGHT;
            do_upper_body = (crpc.kp_upper_body != 0);
            do_zmp_diff = (crpc.kp_zmp_diff != 0);
            do_zmp_com = false;
            break;
        default:
            active_leg = LEFT;
            do_upper_body = false;
            do_zmp_diff = false;
            do_zmp_com = (crpc.kp_zmp_com != 0);
            break;
        }
        
        bool done = false;
        
        while(!done)
        {
            hubo.update();
            
            Vector2d foot_zmp[2];
            Vector2d total_zmp(0,0);
            
            hubo.computeZMPs(Bfoot, foot_zmp, total_zmp, tau_sign); // TODO: Account for fz threshold and tau sign
            
            Vector2d body_angle_meas(hubo.getAngleX(), hubo.getAngleY());
            Vector2d body_angle_ref(0,0);
            Vector2d upper_body_err = (body_angle_ref - body_angle_meas);
            
            double mass_distrib_err = hubo.getFootFz(RIGHT) - hubo.getFootFz(LEFT);
            
            Vector2d zmp_diff_err = foot_zmp[RIGHT] - foot_zmp[LEFT];
            
            Vector2d zmp_ref(crpc.zmp_ref_x, crpc.zmp_ref_y);
            
            Vector2d zmp_com_err = zmp_ref - total_zmp;
            
            done = true;
            
            if(do_upper_body)
            {
                if(upper_body_err.norm() > 0.02 * M_PI/180) {
                    done = false;
                }
                offsets.crpcOffsets.body_angle[0] += crpc.kp_upper_body * upper_body_err.x();
                offsets.crpcOffsets.body_angle[1] += crpc.kp_upper_body * upper_body_err.y();
            }
            
            if(do_mass_distrib)
            {
                if( fabs(mass_distrib_err) > 2) {
                    done = false;
                }
                double sign = (active_leg == LEFT) ? -1 : 1;
                offsets.crpcOffsets.leg_length[active_leg] += sign * crpc.kp_mass_distrib * mass_distrib_err;
            }
            
            if(do_zmp_diff)
            {
                if(zmp_diff_err.norm() > 0.2 * 1e-3) {
                    done = false;
                }
                double sign = (active_leg == LEFT) ? 1 : -1;
                offsets.crpcOffsets.foot_angle_x[active_leg] += crpc.kp_zmp_diff * sign * -zmp_diff_err[1];
                offsets.crpcOffsets.foot_angle_y[active_leg] += crpc.kp_zmp_diff * sign * zmp_diff_err[0];
            }
            
            if(do_zmp_com)
            {
                if(zmp_com_err.norm() > 0.001) {
                    done = false;
                }
                offsets.crpcOffsets.body_com[0] += crpc.kp_zmp_com * zmp_com_err.x();
                offsets.crpcOffsets.body_com[1] += crpc.kp_zmp_com * zmp_com_err.y();
            }

/*
            printf("\033[2J\033[1;1H");
            printf("************************************************************************\n");
            printf("****************************** IN PHASE %d ******************************\n", phase);
            printf("Left FT:           mx = %10f   my = %10f   fz = %10f\n", hubo.getFootMx(LEFT), hubo.getFootMy(LEFT), hubo.getFootFz(LEFT));
            printf("Right FT:          mx = %10f   my = %10f   fz = %10f\n", hubo.getFootMx(RIGHT), hubo.getFootMy(RIGHT), hubo.getFootFz(RIGHT));
            printf("------------------------------------------------------------------------\n");
            printf("Upper body ctrl: %s\n", do_upper_body ? "ACTIVE" : "inactive");
            printf("Body angle meas:    r = %10f    p = %10f\n", body_angle_meas.x(), body_angle_meas.y());
            printf("Body angle ref:     r = %10f    p = %10f\n", body_angle_ref.x(), body_angle_ref.y());
            printf("Upper body err:     r = %10f    p = %10f   norm = %10f\n",  upper_body_err.x(), upper_body_err.y(), upper_body_err.norm());
            printf("------------------------------------------------------------------------\n");
            printf("Mass distrib ctrl: %s\n", do_mass_distrib ? "ACTIVE" : "inactive");
            printf("Mass distrib:     err = %10f\n", mass_distrib_err);
            printf("------------------------------------------------------------------------\n");
            printf("ZMP diff ctrl: %s\n", do_zmp_diff ? "ACTIVE" : "inactive");
            printf("Left foot ZMP:      x = %10f    y = %10f\n", foot_zmp[LEFT].x(), foot_zmp[LEFT].y());
            printf("Right foot ZMP:     x = %10f    y = %10f\n", foot_zmp[RIGHT].x(), foot_zmp[RIGHT].y());
            printf("ZMP diff error:     x = %10f    y = %10f   norm = %10f\n", zmp_diff_err.x(), zmp_diff_err.y(), zmp_diff_err.norm());
            printf("------------------------------------------------------------------------\n");
            printf("ZMP COM ctrl: %s\n", do_zmp_com ? "ACTIVE" : "inactive");
            printf("ZMP total:          x = %10f    y = %10f\n", total_zmp.x(), total_zmp.y());
            printf("ZMP ref:            x = %10f    y = %10f\n", zmp_ref.x(), zmp_ref.y());
            printf("ZMP COM err:        x = %10f    y = %10f   norm = %10f\n", zmp_com_err.x(), zmp_com_err.y(), zmp_com_err.norm());
            printf("------------------------------------------------------------------------\n");
            printf("Body angle offset:  r = %10f    p = %10f\n", offsets.crpcOffsets.body_angle[0], offsets.crpcOffsets.body_angle[1]);
            printf("Body COM offset:    x = %10f    y = %10f\n", offsets.crpcOffsets.body_com[0], offsets.crpcOffsets.body_com[1]);
            printf("Left offsets:      ar = %10f   ap = %10f    len = %10f\n", offsets.crpcOffsets.foot_angle_x[LEFT], offsets.crpcOffsets.foot_angle_y[LEFT], offsets.crpcOffsets.leg_length[LEFT]);
            printf("Right offsets:     ar = %10f   ap = %10f    len = %10f\n", offsets.crpcOffsets.foot_angle_x[RIGHT], offsets.crpcOffsets.foot_angle_x[RIGHT], offsets.crpcOffsets.leg_length[RIGHT]);
            printf("************************************************************************\n");
*/

            
            for(int side=0; side<2; side++)
            {
                LegVector q = qReal[side];
                kin.applyBalanceOffsets(side, q, offsets);
                hubo.setLegAngles(side, q);
            }
            
            hubo.sendControls();
        }
    }

    crpc_state.phase = CRPC_DONE;
    ach_put( &crpc_state_chan, &crpc_state, sizeof(crpc_state) );
    fprintf(stdout, "Posture Controller -- All Phases Finished\n"); fflush(stdout);

    cmd.cmd_request = BAL_READY;
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

    double kneeVelL = gains.spring_gain*kneeAngleErrorL + legJointVels[LEFT](KN);
    double kneeVelR = gains.spring_gain*kneeAngleErrorR + legJointVels[RIGHT](KN);

    double pitchL = gains.straightening_pitch_gain*hubo.getAngleY()
                    + gains.flattening_gain*hubo.getLeftFootMy()    //FOR CORRECT F/T DIRECTIONS
                    - kneeVelL/2;
    double rollL  = gains.straightening_roll_gain*hubo.getAngleX()
                    + gains.flattening_gain*hubo.getLeftFootMx();    //FOR CORRECT F/T DIRECTIONS
    
    double pitchR = gains.straightening_pitch_gain*hubo.getAngleY()
                    + gains.flattening_gain*hubo.getRightFootMy()    //FOR CORRECT F/T DIRECTIONS
                    - kneeVelR/2;
    double rollR  = gains.straightening_roll_gain*hubo.getAngleX()
                    + gains.flattening_gain*hubo.getRightFootMx();    //FOR CORRECT F/T DIRECTIONS

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
