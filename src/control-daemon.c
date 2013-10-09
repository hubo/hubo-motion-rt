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

#include "control-daemon.h"

ach_channel_t chan_hubo_ref;
ach_channel_t chan_hubo_pwm;
ach_channel_t chan_hubo_board_cmd;
ach_channel_t chan_hubo_state;
ach_channel_t chan_hubo_ra_ctrl;
ach_channel_t chan_hubo_la_ctrl;
ach_channel_t chan_hubo_rl_ctrl;
ach_channel_t chan_hubo_ll_ctrl;
ach_channel_t chan_hubo_rf_ctrl;
ach_channel_t chan_hubo_lf_ctrl;
ach_channel_t chan_hubo_bod_ctrl;
ach_channel_t chan_hubo_nck_ctrl;
ach_channel_t chan_ctrl_state;

static char *ctrlFileLocation    = "/etc/hubo-ach/control.table";
static char *dutyFileLocation = "/etc/hubo-ach/duty.table";
static char *torqueFileLocation  = "/etc/hubo-ach/torque.table";

int simMode;


void controlLoop();
int setCtrlDefaults(hubo_param_t *H_param);
int setConversionTables( struct hubo_conversion_tables *conversion);
double getGearReduction(hubo_param_t *h, int jnt);

void sortJointControls( struct hubo_control *ctrl, struct hubo_arm_control *ractrl, struct hubo_arm_control *lactrl,
                                                   struct hubo_leg_control *rlctrl, struct hubo_leg_control *llctrl,
                                                   struct hubo_fin_control *rfctrl, struct hubo_fin_control *lfctrl,
                        struct hubo_bod_control *bodctrl, struct hubo_nck_control *nckctrl)
{
    for(int i=0; i < ractrl->count; i++)
        memcpy( &(ctrl->joint[ractrl->jointIndices[i]]), &(ractrl->joint[i]), sizeof(struct hubo_joint_control) );

    for(int i=0; i < lactrl->count; i++)
        memcpy( &(ctrl->joint[lactrl->jointIndices[i]]),  &(lactrl->joint[i]), sizeof(struct hubo_joint_control) );
    
    for(int i=0; i < rlctrl->count; i++)
        memcpy( &(ctrl->joint[rlctrl->jointIndices[i]]), &(rlctrl->joint[i]), sizeof(struct hubo_joint_control) );
    
    for(int i=0; i < llctrl->count; i++)
        memcpy( &(ctrl->joint[llctrl->jointIndices[i]]),  &(llctrl->joint[i]), sizeof(struct hubo_joint_control) );

    for(int i=0; i < rfctrl->count; i++)
        memcpy( &(ctrl->joint[rfctrl->jointIndices[i]]), &(rfctrl->joint[i]), sizeof(struct hubo_joint_control) );
    
    for(int i=0; i < lfctrl->count; i++)
        memcpy( &(ctrl->joint[lfctrl->jointIndices[i]]),  &(lfctrl->joint[i]), sizeof(struct hubo_joint_control) );
    
    for(int i=0; i < bodctrl->count; i++)
        memcpy( &(ctrl->joint[bodctrl->jointIndices[i]]), &(bodctrl->joint[i]), sizeof(struct hubo_joint_control) );

    for(int i=0; i < nckctrl->count; i++)
        memcpy( &(ctrl->joint[nckctrl->jointIndices[i]]), &(nckctrl->joint[i]), sizeof(struct hubo_joint_control) );

    
    if( ractrl->active==1 || lactrl->active==1 || rlctrl->active==1 || llctrl->active==1
        || rfctrl->active==1 || lfctrl->active==1 || bodctrl->active==1 || nckctrl->active==1 )
    {
        ctrl->active = 1;
    }
    else
        ctrl->active = 0;

}

double sign(double x)
{
    return (x < 0) ? -1 : (x > 0);
}

void controlLoop()
{
    struct hubo_ref H_ref, stored_ref;
    struct hubo_board_cmd H_cmd;
    struct hubo_state H_state;
    struct hubo_control ctrl;
    struct hubo_arm_control ractrl;
    struct hubo_arm_control lactrl;
    struct hubo_leg_control rlctrl;
    struct hubo_leg_control llctrl;
    struct hubo_fin_control rfctrl;
    struct hubo_fin_control lfctrl;
    struct hubo_bod_control bodctrl;
    struct hubo_nck_control nckctrl;
    struct hubo_ctrl_state C_state;
    struct hubo_param h;

    struct hubo_conversion_tables conversion;

    memset( &H_ref,   0, sizeof(H_ref)   );
    memset( &H_cmd,   0, sizeof(H_cmd)   );
    memset( &H_state, 0, sizeof(H_state) );
    memset( &ctrl,    0, sizeof(ctrl)    );
    memset( &ractrl,  0, sizeof(ractrl)  );
    memset( &lactrl,  0, sizeof(lactrl)  );
    memset( &rlctrl,  0, sizeof(rlctrl)  );
    memset( &llctrl,  0, sizeof(llctrl)  );
    memset( &rfctrl,  0, sizeof(rfctrl)  );
    memset( &lfctrl,  0, sizeof(lfctrl)  );
    memset( &bodctrl, 0, sizeof(bodctrl) );
    memset( &nckctrl, 0, sizeof(nckctrl) );
    memset( &C_state, 0, sizeof(C_state) );
    memset( &h,       0, sizeof(h)       );
    memset( &conversion,  0, sizeof(conversion)  );

    size_t fs;
    int result = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
    if( ACH_OK != result )
    {
        // TODO: Debug stuff
    }
    else
    {
        daemon_assert( sizeof(H_ref) == fs, __LINE__ );
    }

    hubo_pwm_gains_t gains;
    memset(&gains, 0, sizeof(gains));
    if(setCtrlDefaults( &h )==-1)
        return;

    if(setConversionTables(&conversion)==-1)
        return;

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        ctrl.joint[i].ctrl_mode = CTRL_HOME;

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        H_ref.mode[i] = HUBO_REF_MODE_REF; // Make sure that the values are not put through Dan's buffer/filter


    memcpy( &stored_ref, &H_ref, sizeof(H_ref) );

    fprintf(stdout, "Waiting for first state... "); fflush(stdout);
    if(simMode==0)
    {
        do {
            struct timespec timeoutCheck;
            clock_gettime( ACH_DEFAULT_CLOCK, &timeoutCheck );
            timeoutCheck.tv_sec += 1;
            result = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, &timeoutCheck, ACH_O_WAIT | ACH_O_LAST );
        } while( !daemon_sig_quit && result == ACH_TIMEOUT );
    }
    else
        ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
    fprintf(stdout, " Got it!\n"); fflush(stdout);

    if( ACH_OK != result )
    {
        // TODO: Print a debug message
    }
    else { daemon_assert( sizeof(H_state) == fs, __LINE__ ); }

    double dr[HUBO_JOINT_COUNT];
    double V[HUBO_JOINT_COUNT];
    double V0[HUBO_JOINT_COUNT];
    double dV[HUBO_JOINT_COUNT];
    double V0_actual[HUBO_JOINT_COUNT];
    double startWaypoint[HUBO_JOINT_COUNT];
    double adr;
    double dtMax = 0.1;
    double errorFactor = 100; // This makes the error checking basically meaningless
    double timeElapse[HUBO_JOINT_COUNT];
    double amp=0;
    double ampUpper;
    double ampLower;
    double torque=0;
    double torqueUpper;
    double torqueLower;
    double dutyUpper;
    double dutyLower;
    int tableType=0;
    double antifriction=0;
    double jointspaceDuty=0;
    double maxPWM=0;


    int torqueWarning[HUBO_JOINT_COUNT];


    int fail[HUBO_JOINT_COUNT];
    int reset[HUBO_JOINT_COUNT];
    ach_status_t cresult, rresult, sresult, presult=ACH_OK;
    int iter=0, maxi=200;

    // Initialize arrays
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        dr[i] = 0;  dV[i] = 0;
        V[i] = 0;   V0[i] = 0;
        V0_actual[i] = 0;
        fail[i] = 0; torqueWarning[i] = 0;
    }

    double minAccel = 0.1; // FIXME: Replace this with user-defined parameter

    double t0 = H_state.time;
    double t = H_state.time, dt, err;
    fprintf(stdout, "Start time:%f\n", H_state.time); fflush(stdout);
    dt = 1.0; // Arbitrary non-zero number to keep things from crashing

    fprintf(stdout, "Beginning control loop\n"); fflush(stdout);  fflush(stdout);

    // Main control loop
    while( !daemon_sig_quit )
    {
        struct timespec recheck;
        clock_gettime( ACH_DEFAULT_CLOCK, &recheck );
        long nanoWait = recheck.tv_nsec + (long)(dt/3.0*1E9);
        recheck.tv_sec += (long)(nanoWait/1E9);
        recheck.tv_nsec = (long)(nanoWait%((long)1E9));
        sresult = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs,
                             &recheck, ACH_O_WAIT | ACH_O_LAST );

        if( ACH_TIMEOUT == sresult || ACH_STALE_FRAMES == sresult )
        {
            memcpy( &H_ref, &stored_ref, sizeof(H_ref) );
        }
        else if( ACH_OK == sresult || ACH_MISSED_FRAME == sresult )
        {
            daemon_assert( sizeof(H_state) == fs, __LINE__ );

            if( H_state.time > t )
            {
                t = H_state.time;
                dt = t - t0;
                t0 = t;
                for(int i=0; i<HUBO_JOINT_COUNT; i++)
                {
                    C_state.actual_vel[i] = (H_state.joint[i].pos - C_state.actual_pos[i])/dt;
                    C_state.actual_acc[i] = (C_state.actual_vel[i] - V0_actual[i])/dt;
                    V0_actual[i] = C_state.actual_vel[i];
                    C_state.actual_pos[i] = H_state.joint[i].pos;
                }
                ach_put( &chan_ctrl_state, &C_state, sizeof(C_state) );

                // These are being updated now because they belong to ach_put of the next cycle
                for(int i=0; i<HUBO_JOINT_COUNT; i++)
                {
                    C_state.requested_pos[i] = H_ref.ref[i];
                    C_state.requested_vel[i] = V[i];
                    C_state.requested_acc[i] = (V[i]-V0[i])/dt;

                    V0[i] = V[i];
                    timeElapse[i] += dt;
                }

                memcpy( &stored_ref, &H_ref, sizeof(H_ref) );
            }
            else if( dt < 0 )
                fprintf(stderr, "You have traveled backwards through time by %f seconds!\n", -dt);
            else if( dt == 0 )
                fprintf(stderr, "Something unnatural has happened on 254 in the control-daemon...\n");
        }
        else
            fprintf( stderr, "Unexpected ach state: %s\n", ach_result_to_string(sresult) );


        cresult = ach_get( &chan_hubo_ra_ctrl, &ractrl, sizeof(ractrl), &fs, NULL, ACH_O_LAST );
        if( cresult==ACH_OK )
            for(int j=0; j<ARM_JOINT_COUNT; j++)
                timeElapse[ractrl.jointIndices[j]] = 0.0;

        cresult = ach_get( &chan_hubo_la_ctrl, &lactrl, sizeof(lactrl), &fs, NULL, ACH_O_LAST );
        if( cresult==ACH_OK )
            for(int j=0; j<ARM_JOINT_COUNT; j++)
                timeElapse[lactrl.jointIndices[j]] = 0.0;

        cresult = ach_get( &chan_hubo_rl_ctrl, &rlctrl, sizeof(rlctrl), &fs, NULL, ACH_O_LAST );
        if( cresult==ACH_OK )
            for(int j=0; j<LEG_JOINT_COUNT; j++)
                timeElapse[rlctrl.jointIndices[j]] = 0.0;

        cresult = ach_get( &chan_hubo_ll_ctrl, &llctrl, sizeof(llctrl), &fs, NULL, ACH_O_LAST );
        if( cresult==ACH_OK )
            for(int j=0; j<LEG_JOINT_COUNT; j++)
                timeElapse[llctrl.jointIndices[j]] = 0.0;

        cresult = ach_get( &chan_hubo_rf_ctrl, &rfctrl, sizeof(rfctrl), &fs, NULL, ACH_O_LAST );
        if( cresult==ACH_OK )
            for(int j=0; j<FIN_JOINT_COUNT; j++)
                timeElapse[rfctrl.jointIndices[j]] = 0.0;


        cresult = ach_get( &chan_hubo_lf_ctrl, &lfctrl, sizeof(lfctrl), &fs, NULL, ACH_O_LAST );
        if( cresult==ACH_OK )
            for(int j=0; j<FIN_JOINT_COUNT; j++)
                timeElapse[lfctrl.jointIndices[j]] = 0.0;
        

        cresult = ach_get( &chan_hubo_bod_ctrl, &bodctrl, sizeof(bodctrl), &fs, NULL, ACH_O_LAST );
        if( cresult==ACH_OK )
            for(int j=0; j<BOD_JOINT_COUNT; j++)
                timeElapse[bodctrl.jointIndices[j]] = 0.0;
        
        cresult = ach_get( &chan_hubo_nck_ctrl, &nckctrl, sizeof(nckctrl), &fs, NULL, ACH_O_LAST );
        if( cresult==ACH_OK )
            for(int j=0; j<NCK_JOINT_COUNT; j++)
                timeElapse[nckctrl.jointIndices[j]] = 0.0;

        sortJointControls( &ctrl, &ractrl, &lactrl,
                                  &rlctrl, &llctrl,
                                  &rfctrl, &lfctrl,
                                  &bodctrl, &nckctrl );

        
        if( ctrl.active == 2 || H_state.refWait==1 )
        {
            if(C_state.paused==0)
                fprintf(stdout, "Pausing control\n");
            C_state.paused=1;

            ach_put( &chan_ctrl_state, &C_state, sizeof(C_state) );
            for(int jnt=0; jnt<HUBO_JOINT_COUNT; jnt++)
            {
                H_ref.ref[jnt] = H_state.joint[jnt].ref;
                stored_ref.ref[jnt] = H_state.joint[jnt].ref;
                V[jnt] = 0; V0[jnt] = 0; dV[jnt] = 0;
                dr[jnt]=0;
/*
                if( ctrl.joint[jnt].ctrl_mode == CTRL_HOME )
                {
                    H_ref.ref[jnt] = 0; 
                    V[jnt]=0; V0[jnt]=0; dV[jnt]=0;
                    dr[jnt]=0;
                }
*/
            }
        }
        else if(ctrl.active==1 && H_state.refWait==0)
            C_state.paused=0;


        if( (0 < dt && dt < dtMax && H_state.refWait==0) || simMode==1 )
        {
            iter++; if(iter>maxi) iter=0;

            for(int jnt=0; jnt<HUBO_JOINT_COUNT; jnt++)
            {
                err = H_ref.ref[jnt] - H_state.joint[jnt].pos;


                if( ctrl.joint[jnt].ctrl_mode == CTRL_PASS )
                {
                    V[jnt] = (ctrl.joint[jnt].position-H_ref.ref[jnt])/dt;
                    H_ref.ref[jnt] = ctrl.joint[jnt].position;
                }
                else if( ctrl.joint[jnt].ctrl_mode == CTRL_PWM )
                {
                    H_ref.comply[jnt] = 1;
                    // FIXME: Remove this hack
                    gains.joint[jnt].maxPWM = 40;

                    gains.joint[jnt].Kp = 0;
                    gains.joint[jnt].Kd = 0;

                    gains.joint[jnt].pwmCommand = ctrl.joint[jnt].pwm;
                }
                else if( fabs(err) <=  fabs(errorFactor*ctrl.joint[jnt].error_limit*dtMax)  // TODO: Validate this condition
                    && fail[jnt]==0  )
                {
                    if( ctrl.joint[jnt].ctrl_mode != CTRL_OFF && ctrl.joint[jnt].ctrl_mode != CTRL_RESET )
                    {

                        if( ctrl.joint[jnt].ctrl_mode == CTRL_VEL )
                        {
                            if( timeElapse[jnt] > ctrl.joint[jnt].timeOut )
                                ctrl.joint[jnt].velocity = 0.0;

                            // FIXME: Remove this hack
                            if( fabs(ctrl.joint[jnt].acceleration) == 0 )
                                ctrl.joint[jnt].acceleration = fabs(minAccel);

//                            if( fabs(ctrl.joint[jnt].acceleration) < fabs(minAccel) );
//                                ctrl.joint[jnt].acceleration = fabs(minAccel);

/*                          // FIXME: Figure out a good way to handle joint limits
                            int inBounds = 0;
                            if( H_ref.ref[jnt] < ctrl.joint[jnt].pos_min )
                                ctrl.joint[jnt].velocity = fabs(ctrl.joint[jnt].velocity);
                            else if( H_ref.ref[jnt] > ctrl.joint[jnt].pos_max )
                                ctrl.joint[jnt].velocity = -fabs(ctrl.joint[jnt].velocity);
                            else
                                inBounds = 1;
*/                           
                            dV[jnt] = ctrl.joint[jnt].velocity - V0[jnt]; // Check how far we are from desired velocity
                            if( dV[jnt] > fabs(ctrl.joint[jnt].acceleration*dt) ) // Scale it down to be within bounds
                                dV[jnt] = fabs(ctrl.joint[jnt].acceleration*dt);
                            else if( dV[jnt] < -fabs(ctrl.joint[jnt].acceleration*dt) )
                                dV[jnt] = -fabs(ctrl.joint[jnt].acceleration*dt);

                            V[jnt] = V0[jnt] + dV[jnt]; // Step velocity forward

                            dr[jnt] = V[jnt]*dt;
/*
                            // FIXME: Figure out a good way to handle joints limits
                            if( H_ref.ref[jnt]+dr[jnt] < ctrl.joint[jnt].pos_min && inBounds==1 )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_min;
                            else if( H_ref.ref[jnt]+dr[jnt] > ctrl.joint[jnt].pos_max && inBounds==1 )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_max;
                            else
                                H_ref.ref[jnt] += dr[jnt];
*/
                            H_ref.ref[jnt] += dr[jnt];
                        }
                        else if( ctrl.joint[jnt].ctrl_mode == CTRL_TRAJ )
                        {
//                            if( timeElapse[jnt] > ctrl.joint[jnt].timeOut )
//                                ctrl.joint[jnt].velocity = 0.0;
                            if( timeElapse[jnt] == 0 )
                                startWaypoint[jnt] = H_ref.ref[jnt];

                            if( timeElapse[jnt] > ctrl.joint[jnt].timeOut )
                                ctrl.joint[jnt].velocity = 0.0;

                            if( ctrl.joint[jnt].position < ctrl.joint[jnt].pos_min )
                                ctrl.joint[jnt].position = ctrl.joint[jnt].pos_min;
                            else if( ctrl.joint[jnt].position > ctrl.joint[jnt].pos_max )
                                ctrl.joint[jnt].position = ctrl.joint[jnt].pos_max;

//                            dr[jnt] = (ctrl.joint[jnt].position - startWaypoint[jnt])
//                                        *timeElapse[jnt]*ctrl.joint[jnt].frequency; // Check how far we are from desired position


                            if( ctrl.joint[jnt].correctness > 1 )
                                ctrl.joint[jnt].correctness = 1;
                            else if( ctrl.joint[jnt].correctness < 0 )
                                ctrl.joint[jnt].correctness = 0;


                            dr[jnt] = (1-ctrl.joint[jnt].correctness)*ctrl.joint[jnt].velocity*dt
                                    + ctrl.joint[jnt].correctness*(
                                        (ctrl.joint[jnt].position - startWaypoint[jnt])
                                        *(timeElapse[jnt]+dt)*ctrl.joint[jnt].frequency
                                        - (H_ref.ref[jnt] - startWaypoint[jnt])
                                        );

                            // Probably come up with a more meaningful safety check
                            if( fabs(dr[jnt]) > fabs(ctrl.joint[jnt].maxSpeed*dt) )
                                dr[jnt] = sign(dr[jnt])*fabs(ctrl.joint[jnt].maxSpeed*dt);

                            H_ref.ref[jnt] += dr[jnt];

                            fprintf(stdout, "%f\n", H_ref.ref[jnt]);

                            V[jnt] = dr[jnt]/dt;
                        }
                        else if( ctrl.joint[jnt].ctrl_mode == CTRL_POS )
                        {
//if(jnt==RF1)
//fprintf(stdout, "Pos:%f\t", ctrl.joint[jnt].position);
//                            if( ctrl.joint[jnt].acceleration < minAccel )
//                                ctrl.joint[jnt].acceleration = fabs(minAccel);

                            // Constrain ourselves to the dynamic joint limits
                            if( ctrl.joint[jnt].position < ctrl.joint[jnt].pos_min )
                                ctrl.joint[jnt].position = ctrl.joint[jnt].pos_min;
                            else if( ctrl.joint[jnt].position > ctrl.joint[jnt].pos_max )
                                ctrl.joint[jnt].position = ctrl.joint[jnt].pos_max;

                            dr[jnt] = ctrl.joint[jnt].position - H_ref.ref[jnt]; // Check how far we are from desired position

                            ctrl.joint[jnt].velocity = sign(dr[jnt])*fabs(ctrl.joint[jnt].speed); // Set velocity into the correct direction


                            dV[jnt] = ctrl.joint[jnt].velocity - V0[jnt]; // Check how far we are from desired velocity

                            adr = sqrt(fabs(2.0*ctrl.joint[jnt].acceleration*dr[jnt])); // Slow down before reaching goal
                            if( fabs(V0[jnt]) >= adr ) 
                                dV[jnt] = sign(dr[jnt])*adr-V0[jnt];
                            else if( dV[jnt] > fabs(ctrl.joint[jnt].acceleration*dt) ) // Scale it down to be within bounds
                                dV[jnt] = fabs(ctrl.joint[jnt].acceleration*dt);
                            else if( dV[jnt] < -fabs(ctrl.joint[jnt].acceleration*dt) ) // Make sure the sign is correct
                                dV[jnt] = -fabs(ctrl.joint[jnt].acceleration*dt);

                            V[jnt] = V0[jnt] + dV[jnt]; // Step velocity forward

/*
                            if( fabs(dr[jnt]) > fabs(V[jnt]*dt) && V[jnt]*dr[jnt] >= 0 )
                                dr[jnt] = V[jnt]*dt;
                            else if( fabs(dr[jnt]) > fabs(V[jnt]*dt) && V[jnt]*dr[jnt] < 0 )
                                dr[jnt] = -V[jnt]*dt;
*/

                            if( fabs(dr[jnt]) > fabs(V[jnt]*dt) || V[jnt]*dr[jnt] < 0 )
                                dr[jnt] = V[jnt]*dt;

                            V[jnt] = dr[jnt]/dt;

/*                            if( H_ref.ref[jnt]+dr[jnt] < ctrl.joint[jnt].pos_min )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_min;
                            else if( H_ref.ref[jnt]+dr[jnt] > ctrl.joint[jnt].pos_max )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_max;
                            else
                                H_ref.ref[jnt] += dr[jnt];*/
                            H_ref.ref[jnt] += dr[jnt];
                        }
                        else if( ctrl.joint[jnt].ctrl_mode == CTRL_HOME )
                        {
                            V[jnt]=0; V0[jnt]=0; dV[jnt]=0;
                        }
                        else
                        {
                            fprintf(stderr, "Joint %d has invalid control mode: %d\n",
                                jnt, (int)ctrl.joint[jnt].ctrl_mode );
                        }

//                        timeElapse[jnt] += dt;
//                        V0[jnt] = V[jnt];
//                        C_state.velocity[jnt] = V[jnt];
                        reset[jnt]=0;
                    }

                }
                else if(fail[jnt]==0 && ctrl.joint[jnt].ctrl_mode != CTRL_HOME)
                {
                    fprintf(stderr, "JOINT %d FROZEN! Exceeded error limit(%g):%g, Ref:%f, State:%f\n", jnt,
                        fabs(errorFactor*ctrl.joint[jnt].error_limit*dtMax), err, H_ref.ref[jnt], H_state.joint[jnt].pos );
                    V[jnt]=0; V0[jnt]=0;
                    H_ref.ref[jnt] = H_state.joint[jnt].pos;
                    fail[jnt]=1;
                    C_state.status[jnt] = 1;
                }
                else if( ctrl.joint[jnt].ctrl_mode == CTRL_RESET && reset[jnt]==0 )
                {
                    fprintf(stdout, "Joint %d has been reset\n", jnt);
                    V[jnt]=0; V0[jnt]=0; dV[jnt]=0;
                    dr[jnt]=0;
                    fail[jnt]=0;
                    reset[jnt]=1;
                    C_state.status[jnt] = 0;
                }

                if( H_ref.ref[jnt] != H_ref.ref[jnt] )
                {
                    fprintf( stderr, "JOINT FROZEN! You have requested a NaN for joint #%d!\n", jnt );
                    H_ref.ref[jnt] = stored_ref.ref[jnt];
                    fail[jnt] = 1;
                    C_state.status[jnt] = 1;
                }                    


                
                if( ctrl.joint[jnt].comp_mode == CTRL_COMP_ON || ctrl.joint[jnt].torque_mode == CTRL_TORQUE_ON )
                {
                    H_ref.comply[jnt] = 1;
                    // FIXME: Remove this hack
                    gains.joint[jnt].maxPWM = 40;

                    if( ctrl.joint[jnt].torque_mode != CTRL_TORQUE_ON )
                    {
                        gains.joint[jnt].pwmCommand = 0;
                        gains.joint[jnt].pwmCommand = 0;
                    }
                    else
                    {
                        // These torque calculations are based off of
                        // Dr. Inhyeok Kim's RAINBOW code
//                        amp = fabs(ctrl.joint[jnt].torque)/conversion.Kt[jnt]/getGearReduction(&h, jnt);

                        tableType = conversion.joint[jnt].dutyType;
                        int c = 0;

                        torque = fabs(ctrl.joint[jnt].torque);

                        if(conversion.table[tableType].torque[conversion.table[tableType].count-1]
                                <= torque)
                            c = conversion.table[tableType].count-2;
                        else
                            for(c=0; c<conversion.table[tableType].count-1; c++)
                                if(conversion.table[tableType].torque[c] <= torque
                                        && torque < conversion.table[tableType].torque[c+1])
                                    break;

                        if(tableType != 0)
                        {

                            torqueLower = conversion.table[tableType].torque[c];
                            torqueUpper = conversion.table[tableType].torque[c+1];
                            dutyLower   = conversion.table[tableType].duty[c];
                            dutyUpper   = conversion.table[tableType].duty[c+1];

                            if(torqueLower == torqueUpper)
                            {
                                gains.joint[jnt].pwmCommand = 0;
                                fprintf(stderr, "ERROR IN DUTY TABLE! Torque entry %d is equal to entry %d!",
                                        c, c+1);
                            }
                            else
                                gains.joint[jnt].pwmCommand = sign(ctrl.joint[jnt].torque)*
                                    ((dutyUpper-dutyLower)/(torqueUpper-torqueLower)*(torque-torqueLower)+dutyLower
                                        + fabs(conversion.joint[jnt].Fmax*conversion.joint[jnt].deadbandScale) );

                            if(simMode==1 && iter==maxi)
                                fprintf(stdout, "%s | %06.3f : [%06.3f %06.3f] [%04.1f %04.1f] (%05.2f) : %04.1f ",
                                        jointNames[jnt], ctrl.joint[jnt].torque,
                                        torqueLower, torqueUpper, dutyLower, dutyUpper,
                                        fabs(conversion.joint[jnt].Fmax*conversion.joint[jnt].deadbandScale),
                                        gains.joint[jnt].pwmCommand);
                            
                        }
                        else
                        {
                            if(torqueWarning[jnt]==0)
                                fprintf(stderr, "You are requesting torque mode for a joint that does not support it! (%s)\n", jointNames[jnt]);
                            torqueWarning[jnt]=1;
                        }

                    }

                    if(ctrl.joint[jnt].friction_mode == CTRL_ANTIFRICTION_ON)
                    {
                        antifriction = conversion.joint[jnt].kF*H_state.joint[jnt].vel;
                        if(fabs(antifriction) > fabs(conversion.joint[jnt].Fmax))
                            antifriction = sign(antifriction)*fabs(conversion.joint[jnt].Fmax);

                        gains.joint[jnt].pwmCommand += antifriction;


                        if(simMode==1 && iter==maxi)
                            fprintf(stdout, "\t---> (%f) ---> %f : %f\t", H_state.joint[jnt].vel, antifriction, gains.joint[jnt].pwmCommand);
                    }

                    if( ctrl.joint[jnt].comp_mode != CTRL_COMP_ON )
                    {
                        gains.joint[jnt].Kp = 0;
                        gains.joint[jnt].Kd = 0;
                    }
                    else
                    {
                        gains.joint[jnt].Kp = 0;
                        gains.joint[jnt].Kd = 0;

                        jointspaceDuty = ctrl.joint[jnt].Kp*(H_ref.ref[jnt] - H_state.joint[jnt].pos)
                                       - ctrl.joint[jnt].Kd*H_state.joint[jnt].vel;

                        maxPWM = ctrl.joint[jnt].maxPWM;
                        if(maxPWM > 100)
                            maxPWM = 100;
                        else if(maxPWM < 0)
                            maxPWM = 0;

                        if(fabs(jointspaceDuty) > maxPWM)
                            jointspaceDuty = sign(jointspaceDuty)*maxPWM;

                        gains.joint[jnt].pwmCommand += jointspaceDuty;

                        if( ctrl.joint[jnt].friction_mode == CTRL_ANTIFRICTION_ON )
                        {

                            if( sign(H_state.joint[jnt].vel) != sign(H_ref.ref[jnt] - H_state.joint[jnt].pos) )
                            {
                                antifriction = -conversion.joint[jnt].kF*H_state.joint[jnt].vel;
                                if(fabs(antifriction) > fabs(conversion.joint[jnt].Fmax))
                                    antifriction = sign(antifriction)*fabs(conversion.joint[jnt].Fmax);

                                gains.joint[jnt].pwmCommand += antifriction;
                            }

                        }

                        if( ctrl.joint[jnt].torque_mode == CTRL_TORQUE_ON )
                        {

                            if( sign(ctrl.joint[jnt].torque) != sign(H_ref.ref[jnt] - H_state.joint[jnt].pos) )
                            {
                                gains.joint[jnt].pwmCommand += -0.0*sign(ctrl.joint[jnt].torque)
                                                               *fabs(conversion.joint[jnt].Fmax
                                                               *conversion.joint[jnt].deadbandScale);
                            }

                        }
/*
                        gains.joint[jnt].pwmCommand += fabs(conversion.joint[jnt].Fmax
                                                      *conversion.joint[jnt].deadbandScale)
                                                      *sign(H_ref.ref[jnt]-H_state.joint[jnt].pos);
*/
                    }

                    if(simMode==1 && iter==maxi)
                        fprintf(stdout, "\n");

                }
                else if( ctrl.joint[jnt].ctrl_mode != CTRL_PWM )
                    H_ref.comply[jnt] = 0;





            } // end: for loop


            if(ctrl.active == 1 && C_state.paused==0)
            {
                presult = ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref) );
                if(presult != ACH_OK)
                    fprintf(stderr, "Error sending ref command! (%d) %s\n",
                        presult, ach_result_to_string(presult));
                presult = ach_put( &chan_hubo_pwm, &gains, sizeof(gains) );
                if(presult != ACH_OK)
                    fprintf(stderr, "Error sending pwm command! (%d) %s\n",
                        presult, ach_result_to_string(presult));
            }
            
        }// end: time test
        else if( dt >= 0.1 && simMode==0 )
            fprintf(stderr, "Experiencing Delay of %f seconds\n", dt);
        else if( dt < 0 )
            fprintf(stderr, "Congratulations! You have traveled backwards"
                            " through time by %f seconds!", -dt);
        else if( dt == 0 )
            fprintf(stderr, "Something unnatural has happened...\n");

        fflush(stdout);
        fflush(stderr);
    } // End of Main Control Loop

}





int main(int argc, char **argv)
{
    // TODO: Parse runtime arguments
    simMode = 0;
    int eatTerminal = 0;

    for(int i=1; i<argc; i++)
    {
        if( 0 == strcmp(argv[i], "-s") )
            simMode = 1;

        if( 0 == strcmp(argv[i], "-t") )
            eatTerminal = 1;
    }

    if(eatTerminal != 1)
        daemonize( "control-daemon", 49 );
   
    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_pwm, HUBO_CHAN_PWM_GAINS_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_ra_ctrl, HUBO_CHAN_RA_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_la_ctrl, HUBO_CHAN_LA_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_rl_ctrl, HUBO_CHAN_RL_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_ll_ctrl, HUBO_CHAN_LL_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_rf_ctrl, HUBO_CHAN_RF_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_lf_ctrl, HUBO_CHAN_LF_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_bod_ctrl, HUBO_CHAN_BOD_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );
    
    r = ach_open(&chan_hubo_nck_ctrl, HUBO_CHAN_NCK_CTRL_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_hubo_board_cmd, HUBO_CHAN_BOARD_CMD_NAME, NULL);
    daemon_assert( ACH_OK == r, __LINE__ );

    r = ach_open(&chan_ctrl_state, CTRL_CHAN_STATE, NULL );
    daemon_assert( ACH_OK == r, __LINE__ );

    hubo_conversion_tables_t conversion;
    setConversionTables(&conversion);

    for(int i=0; i<MAX_DUTY_TABLE_TYPES; i++)
    {
        fprintf(stdout, "Table #%d\t Count:%lu\n", i, conversion.table[i].count);
        fprintf(stdout, "Duty:\t");
        for(int j=0; j<conversion.table[i].count; j++)
            fprintf(stdout, "\t%f", conversion.table[i].duty[j]);
        fprintf(stdout, "\n");
        fprintf(stdout, "Torque: \t");
        for(int j=0; j<conversion.table[i].count; j++)
            fprintf(stdout, "\t%f", conversion.table[i].torque[j]);
        fprintf(stdout, "\n\n");
    }

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        fprintf(stdout, "%s:\t%d\t%f\t%f\t%f\n",jointNames[i],
                conversion.joint[i].dutyType, conversion.joint[i].kF,
                conversion.joint[i].Fmax, conversion.joint[i].deadbandScale);


    controlLoop();

    daemon_close();
}


int setCtrlDefaults(hubo_param_t *H_param)
{
    struct hubo_arm_control ractrl;
    struct hubo_arm_control lactrl;
    struct hubo_leg_control rlctrl;
    struct hubo_leg_control llctrl;
    struct hubo_fin_control rfctrl;
    struct hubo_fin_control lfctrl;
    struct hubo_bod_control bodctrl;
    struct hubo_nck_control nckctrl;

	memset( &ractrl, 0, sizeof(struct hubo_arm_control) );
	memset( &lactrl, 0, sizeof(struct hubo_arm_control) );
	memset( &rlctrl, 0, sizeof(struct hubo_leg_control) );
	memset( &llctrl, 0, sizeof(struct hubo_leg_control) );
	memset( &rfctrl, 0, sizeof(struct hubo_fin_control) );
	memset( &lfctrl, 0, sizeof(struct hubo_fin_control) );
	memset( &bodctrl, 0, sizeof(struct hubo_bod_control) );
    memset( &nckctrl, 0, sizeof(struct hubo_nck_control) );



    struct hubo_state H_state;
    struct hubo_pwm_gains gains;
    memset( H_param, 0, sizeof(*H_param) );
    memset( &H_state, 0, sizeof(H_state) );
    memset( &gains,   0, sizeof(gains)   );
    setJointParams( H_param, &H_state, &gains);


	FILE *ptr_file;

	// open file for read access and if it fails, return -1
	if (!(ptr_file=fopen(ctrlFileLocation, "r")))
    {
        fprintf(stderr, "Unable to locate %s\n -- Try reinstalling hubo-motion-rt!\n",ctrlFileLocation);
		return -1;
    }
	// instantiate stucts for getting values from control.table
	// file and copying them to
	struct hubo_joint_control tempJC;
    memset( &tempJC, 0, sizeof(tempJC) );


    // TODO: Just use the hubo.h jointNames[] array instead
	// array of joint name values from header file hubo.h
	uint16_t jointNameValues[] =
			{WST, NKY, NK1, NK2,
			LSP, LSR, LSY, LEB, LWY, LWR, LWP,
			RSP, RSR, RSY, REB, RWY, RWR, RWP,
			LHY, LHR, LHP, LKN, LAP, LAR,
			RHY, RHR, RHP, RKN, RAP, RAR,
			RF1, RF2, RF3, RF4, RF5,
			LF1, LF2, LF3, LF4, LF5};

	// array of joint name strings (total of 40)
	char *jointNameStrings[] =
			{"WST", "NKY", "NK1", "NK2",
			 "LSP", "LSR", "LSY", "LEB", "LWY", "LWR", "LWP",
			 "RSP", "RSR", "RSY", "REB", "RWY", "RWR", "RWP",
			 "LHY", "LHR", "LHP", "LKN", "LAP", "LAR",
			 "RHY", "RHR", "RHP", "RKN", "RAP", "RAR",
			 "RF1", "RF2", "RF3", "RF4", "RF5",
			 "LF1", "LF2", "LF3", "LF4", "LF5"};


	char *charPointer;
	size_t jntNameCheck = 0;
	char buff[1024];
    char name[4];
    char type[3];

	// read in each non-commented line of the config file corresponding to each joint
	while (fgets(buff, sizeof(buff), ptr_file) != NULL)
    {
		// set first occurrence of comment character, '#' to the
		// null character, '\0'.
		charPointer = strchr(buff, '#');
		if (NULL != charPointer) {
			*charPointer = '\0';
        }

		// check if a line is longer than the buffer, 'buff', and return -1 if so.
		if ( strlen(buff) == sizeof(buff)-1 ) {
			fprintf(stderr, "Control Table Parser: Line length overflow");
			return -1; // parsing failed
		}

		// read in the buffered line from fgets, matching the following pattern
		// to get all the parameters for the joint on this line.
        if (11 == sscanf(buff, "%s%lf%lf%lf%lf%lf%lf%lf%lf%lf%s",
			name,
			&tempJC.speed,
			&tempJC.acceleration,
            &tempJC.maxSpeed,
            &tempJC.correctness,
			&tempJC.error_limit,
			&tempJC.pos_min,
			&tempJC.pos_max,
            &tempJC.timeOut,
            &tempJC.frequency,
            type ) ) // check that all values are found
		{

			// check to make sure jointName is valid
			size_t x; int i; jntNameCheck = 0;
			for (x = 0; x < sizeof(jointNameStrings)/sizeof(jointNameStrings[0]); x++) {
				if (0 == strcmp(name, jointNameStrings[x])) {
					i = jointNameValues[x];
					jntNameCheck = 1;
					break;
				}
			}

			// if joint name is invalid print error and return -1
			if (jntNameCheck != 1) {
				fprintf(stderr, "Joint name '%s' is incorrect\n", name);
				return -1; // parsing failed
			}
            else
            {
                if( strcmp(type, "LA") == 0 )
                {
                    memcpy( &lactrl.joint[lactrl.count], &tempJC, sizeof(tempJC) );
                    lactrl.jointIndices[lactrl.count] = i;

                    lactrl.joint[lactrl.count].Kp = gains.joint[i].Kp;
                    lactrl.joint[lactrl.count].Kd = gains.joint[i].Kd;
                    lactrl.joint[lactrl.count].maxPWM = gains.joint[i].maxPWM;

                    lactrl.count++;
                }
                
                else if( strcmp(type, "RA") == 0 )
                {
                    memcpy( &ractrl.joint[ractrl.count], &tempJC, sizeof(tempJC) );
                    ractrl.jointIndices[ractrl.count] = i;

                    ractrl.joint[ractrl.count].Kp = gains.joint[i].Kp;
                    ractrl.joint[ractrl.count].Kd = gains.joint[i].Kd;
                    ractrl.joint[ractrl.count].maxPWM = gains.joint[i].maxPWM;

                    ractrl.count++;
                }
                
                else if( strcmp(type, "LL") == 0 )
                {
                    memcpy( &llctrl.joint[llctrl.count], &tempJC, sizeof(tempJC) );
                    llctrl.jointIndices[llctrl.count] = i;

                    llctrl.joint[llctrl.count].Kp = gains.joint[i].Kp;
                    llctrl.joint[llctrl.count].Kd = gains.joint[i].Kd;
                    llctrl.joint[llctrl.count].maxPWM = gains.joint[i].maxPWM;

                    llctrl.count++;
                }
                
                else if( strcmp(type, "RL") == 0 )
                {
                    memcpy( &rlctrl.joint[rlctrl.count], &tempJC, sizeof(tempJC) );
                    rlctrl.jointIndices[rlctrl.count] = i;

                    rlctrl.joint[rlctrl.count].Kp = gains.joint[i].Kp;
                    rlctrl.joint[rlctrl.count].Kd = gains.joint[i].Kd;
                    rlctrl.joint[rlctrl.count].maxPWM = gains.joint[i].maxPWM;

                    rlctrl.count++;
                }
                
                else if( strcmp(type, "LF") == 0 )
                {
                    memcpy( &lfctrl.joint[lfctrl.count], &tempJC, sizeof(tempJC) );
                    lfctrl.jointIndices[lfctrl.count] = i;

                    lfctrl.joint[lfctrl.count].Kp = gains.joint[i].Kp;
                    lfctrl.joint[lfctrl.count].Kd = gains.joint[i].Kd;
                    lfctrl.joint[lfctrl.count].maxPWM = gains.joint[i].maxPWM;

                    lfctrl.count++;
                }
                
                else if( strcmp(type, "RF") == 0 )
                {
                    memcpy( &rfctrl.joint[rfctrl.count], &tempJC, sizeof(tempJC) );
                    rfctrl.jointIndices[rfctrl.count] = i;

                    rfctrl.joint[rfctrl.count].Kp = gains.joint[i].Kp;
                    rfctrl.joint[rfctrl.count].Kd = gains.joint[i].Kd;
                    rfctrl.joint[rfctrl.count].maxPWM = gains.joint[i].maxPWM;

                    rfctrl.count++;
                }
                
                else if( strcmp(type, "BD") == 0 )
                {
                    memcpy( &bodctrl.joint[bodctrl.count], &tempJC, sizeof(tempJC) );
                    bodctrl.jointIndices[bodctrl.count] = i;

                    bodctrl.joint[bodctrl.count].Kp = gains.joint[i].Kp;
                    bodctrl.joint[bodctrl.count].Kd = gains.joint[i].Kd;
                    bodctrl.joint[bodctrl.count].maxPWM = gains.joint[i].maxPWM;

                    bodctrl.count++;
                }
                
                else if( strcmp(type, "NK") == 0 )
                {
                    memcpy( &nckctrl.joint[nckctrl.count], &tempJC, sizeof(tempJC) );
                    nckctrl.jointIndices[nckctrl.count] = i;

                    nckctrl.joint[nckctrl.count].Kp = gains.joint[i].Kp;
                    nckctrl.joint[nckctrl.count].Kd = gains.joint[i].Kd;
                    nckctrl.joint[nckctrl.count].maxPWM = gains.joint[i].maxPWM;

                    nckctrl.count++;
                }
                
            } // end: if (jnkNameCheck != 1)
        } // end: if(sscanf)
    } // end: while(fgets)




	fclose(ptr_file);	// close file stream
    ach_put( &chan_hubo_ra_ctrl, &ractrl, sizeof(ractrl) );
    ach_put( &chan_hubo_la_ctrl, &lactrl, sizeof(lactrl) );
    ach_put( &chan_hubo_rl_ctrl, &rlctrl, sizeof(rlctrl) ); 
    ach_put( &chan_hubo_ll_ctrl, &llctrl, sizeof(llctrl) );
    ach_put( &chan_hubo_rf_ctrl, &rfctrl, sizeof(rfctrl) );
    ach_put( &chan_hubo_lf_ctrl, &lfctrl, sizeof(lfctrl) );
    ach_put( &chan_hubo_bod_ctrl, &bodctrl, sizeof(bodctrl) );
    ach_put( &chan_hubo_nck_ctrl, &nckctrl, sizeof(nckctrl) );

    return 0;

}





int setConversionTables( struct hubo_conversion_tables *conversion)
{
    memset(conversion, 0, sizeof(*conversion));

    FILE *ptr_file;

    if(!(ptr_file=fopen(dutyFileLocation, "r")))
    {
        fprintf(stderr, "Unable to locate %s\n "
                        " -- Try using sudo!\n"
                        " -- If that doesn\'t work, try reinstalling hubo-motion-rt\n",
                dutyFileLocation);
        return -1;
    }

    hubo_duty_table_t tempTable;
    memset(&tempTable, 0, sizeof(tempTable));

    char *charPointer;
    char buff[2048];
    char type[256];
    size_t tableID = 0;
    size_t currentLine = 0;
    ctrl_table_t tableType;

    fprintf(stdout, "Grabbing the %s table... ", dutyFileLocation); fflush(stdout);
    while( fgets(buff, sizeof(buff), ptr_file) != NULL )
    {
        currentLine++;
        FILE *tableLine;
        charPointer = strchr(buff, '#');
        if( NULL != charPointer )
            *charPointer = '\0';


        if( strlen(buff) == sizeof(buff)-1 )
        {
            fprintf(stderr, "Duty Table Parser: Line length overflow!\n");
            return -1;
        }

        tableLine = fmemopen(buff, sizeof(buff), "r");

        if(fscanf(tableLine, "%s", type)==1)
        {
            if(strcmp(type, "Duty")==0)
            {
                tableType = CTRL_TABLE_DUTY;
            }
            else if(strcmp(type, "Torque")==0)
            {
                tableType = CTRL_TABLE_TORQUE;
            }
            else if(strcmp(type, "Amp")==0)
            {
                tableType = CTRL_TABLE_AMP;
            }
            else if(strcmp(type, "")==0)
                continue; // Ignore blank lines
            else
            {
                fprintf(stderr, "Unknown type in Amp-Duty table! (%s, line %lu)\n"
                                " -- Must be \'Amp\' or \'Duty\'\n", type, currentLine);
                return -1;
            }

            if(fscanf(tableLine, "%lu", &tableID)==1)
            {
                if(tableID < MAX_DUTY_TABLE_TYPES)
                {
                    size_t entryID=0;
                    double entry = 0;
                    while(fscanf(tableLine, "%lf", &entry)==1)
                    {
                        if(entryID > MAX_DUTY_TABLE_SIZE)
                        {
                            fprintf(stderr, "Too many entries (%lu) in line %lu! Current max is %d\n"
                                            " -- You must increase MAX_DUTY_TABLE_SIZE in control-daemon.h\n",
                                            entryID, currentLine, MAX_DUTY_TABLE_SIZE);
                        }

                        if(tableType==CTRL_TABLE_DUTY)
                            conversion->table[tableID].duty[entryID] = entry;
                        else if(tableType==CTRL_TABLE_TORQUE)
                            conversion->table[tableID].torque[entryID] = entry;
                        else if(tableType==CTRL_TABLE_AMP)
                            conversion->table[tableID].amp[entryID] = entry;
                        else
                        {
                            fprintf(stderr, "setConversionTables(~) does not know how to handle a table type of %d!\n"
                                            " -- Check line %d in hubo-motion-rt/src/control-daemon.c\n",
                                            tableType, __LINE__-4);
                            return -1;
                        }
                        entryID++;
                    }
                    conversion->table[tableID].count = entryID;
                }
                else
                {
                    fprintf(stderr, "Table ID (%lu) is too high on line %lu! Current max is %d\n"
                            " -- You must increase MAX_DUTY_TABLE_TYPES in control-daemon.h\n",
                            tableID, currentLine, MAX_DUTY_TABLE_TYPES);
                    return -1;
                }
            }
            else
            {
                fprintf(stderr, "Table type %s needs an ID! (line %lu)\n", type, currentLine);
                return -1;
            }
        }
    }
    fprintf(stdout, " Got it!\n"); fflush(stdout);


    size_t jntNameCheck = 0;
    char name[4];
    hubo_joint_duty_settings_t tempSet;
    memset(&tempSet, 0, sizeof(tempSet));


    if(!(ptr_file=fopen(torqueFileLocation, "r")))
    {
        fprintf(stderr, "Unable to locate %s\n"
                        " -- Try using sudo!\n"
                        " -- If that doesn\'t work, try reinstalling hubo-motion-rt\n",
                        torqueFileLocation);
        return -1;
    }

    currentLine = 0;
    fprintf(stdout, "Grabbing the %s table... ", torqueFileLocation); fflush(stdout);
    while( fgets(buff, sizeof(buff), ptr_file) != NULL )
    {
        currentLine++;
        charPointer = strchr(buff, '#');
        if(NULL != charPointer)
            *charPointer = '\0';

        if( strlen(buff) == sizeof(buff)-1 )
        {
            fprintf(stderr, "Torque Table Parser: Line length overflow!\n");
            return -1;
        }

        if( 5 == sscanf(buff, "%s%lu%lf%lf%lf",
                        name,
                        &(tempSet.dutyType),
                        &(tempSet.kF),
                        &(tempSet.Fmax),
                        &(tempSet.deadbandScale)) )
        {
            size_t x; int i; jntNameCheck = 0;
            for( x = 0; x < sizeof(jointNames)/sizeof(jointNames[0]); x++ )
            {
                if(0 == strcmp(name, jointNames[x]))
                {
                    i = x;
                    jntNameCheck = 1;
                    break;
                }
            }

            if(jntNameCheck != 1)
            {
                fprintf(stderr, "Joint name %s is not valid! line %lu\n",
                        name, currentLine);
                return -1;
            }
            else
            {
                memcpy(&(conversion->joint[i]), &tempSet, sizeof(tempSet));
            }

        } // end: if(sscanf)
    } // end: while(fgets)
    fprintf(stdout, "Got it!\n"); fflush(stdout);

}



double getGearReduction(hubo_param_t *h, int jnt)
{
    return h->joint[jnt].driven/h->joint[jnt].drive*h->joint[jnt].harmonic;
}


