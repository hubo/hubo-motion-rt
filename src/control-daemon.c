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

static char *ctrlFileLocation = "/etc/hubo-ach/control.table";


void controlLoop();
int setCtrlDefaults( struct hubo_control *ctrl );

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
    struct hubo_param H_param;
    struct hubo_ctrl_state C_state;

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
    memset( &H_param, 0, sizeof(H_param) );
    memset( &C_state, 0, sizeof(C_state) );

    setJointParams( &H_param, &H_state);
    if(setCtrlDefaults( &ctrl )==-1)
        return;

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        ctrl.joint[i].mode = CTRL_HOME;

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        H_ref.mode[i] = 1; // Make sure that the values are not put through Dan's buffer/filter

    size_t fs;
    int result = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
    if( ACH_OK != result )
    {
        // TODO: Debug stuff
    }
    else
    {
        daemon_assert( sizeof(H_state) == fs, __LINE__ );
    }
    memcpy( &stored_ref, &H_ref, sizeof(H_ref) );
    result = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs, NULL, ACH_O_LAST );
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
    double adr;
    double dtMax = 0.1;
    double errorFactor = 10;
    double timeElapse[HUBO_JOINT_COUNT];

    int fail[HUBO_JOINT_COUNT];
    int reset[HUBO_JOINT_COUNT];
    ach_status_t cresult, rresult, sresult, presult, iter=0, maxi=15;

    // Initialize arrays
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        dr[i] = 0;  dV[i] = 0;
        V[i] = 0;   V0[i] = 0;
        V0_actual[i] = 0;
        fail[i] = 0;
    }

    double t0 = H_state.time;
    double t = H_state.time, dt, err;
    dt = 1.0; // Arbitrary non-zero number to keep things from crashing

    fprintf(stdout, "Beginning control loop\n"); fflush(stdout);

    // Main control loop
    while( !daemon_sig_quit )
    {

        struct timespec recheck;
        clock_gettime( ACH_DEFAULT_CLOCK, &recheck );
        int nanoWait = recheck.tv_nsec + (int)(dt/3.0*1E9);
        recheck.tv_sec += (int)(nanoWait/1E9);
        recheck.tv_nsec = (int)(nanoWait%((int)1E9));

        sresult = ach_get( &chan_hubo_state, &H_state, sizeof(H_state), &fs,
                             &recheck, ACH_O_WAIT | ACH_O_LAST );
        if( ACH_TIMEOUT == sresult || ACH_STALE_FRAMES == sresult )
        {
            memcpy( &H_ref, &stored_ref, sizeof(H_ref) );
        }
        else if( ACH_OK == sresult || ACH_MISSED_FRAME == sresult )
        {
            daemon_assert( sizeof(H_state) == fs, __LINE__ );

            t = H_state.time;
            dt = t - t0;
            t0 = t;

            if( dt > 0 )
            {
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
                fprintf(stderr, "Something unnatural has happened...\n");
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
                if( ctrl.joint[jnt].mode == CTRL_HOME )
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


        if( 0 < dt && dt < dtMax && H_state.refWait==0 )
        {
            iter++; if(iter>maxi) iter=0;


            for(int jnt=0; jnt<HUBO_JOINT_COUNT; jnt++)
            {
                err = H_ref.ref[jnt] - H_state.joint[jnt].pos;

                if( ctrl.joint[jnt].mode == CTRL_PASS )
                {
                    H_ref.ref[jnt] = ctrl.joint[jnt].position;
                }
                else if( fabs(err) <=  fabs(errorFactor*ctrl.joint[jnt].error_limit*dtMax)  // TODO: Validate this condition
                    && fail[jnt]==0  )
                {
                    if( ctrl.joint[jnt].mode != CTRL_OFF && ctrl.joint[jnt].mode != CTRL_RESET )
                    {

                        if( ctrl.joint[jnt].mode == CTRL_VEL )
                        {
                            if( timeElapse[jnt] > ctrl.joint[jnt].timeOut )
                                ctrl.joint[jnt].velocity = 0.0;

                            int inBounds = 0;
                            if( H_ref.ref[jnt] < ctrl.joint[jnt].pos_min )
                                ctrl.joint[jnt].velocity = fabs(ctrl.joint[jnt].velocity);
                            else if( H_ref.ref[jnt] > ctrl.joint[jnt].pos_max )
                                ctrl.joint[jnt].velocity = -fabs(ctrl.joint[jnt].velocity);
                            else
                                inBounds = 1;
                           
                            dV[jnt] = ctrl.joint[jnt].velocity - V0[jnt]; // Check how far we are from desired velocity
                            if( dV[jnt] > fabs(ctrl.joint[jnt].acceleration*dt) ) // Scale it down to be within bounds
                                dV[jnt] = fabs(ctrl.joint[jnt].acceleration*dt);
                            else if( dV[jnt] < -fabs(ctrl.joint[jnt].acceleration*dt) )
                                dV[jnt] = -fabs(ctrl.joint[jnt].acceleration*dt);

                            V[jnt] = V0[jnt] + dV[jnt]; // Step velocity forward

                            dr[jnt] = V[jnt]*dt;


                            if( H_ref.ref[jnt]+dr[jnt] < ctrl.joint[jnt].pos_min && inBounds==1 )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_min;
                            else if( H_ref.ref[jnt]+dr[jnt] > ctrl.joint[jnt].pos_max && inBounds==1 )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_max;
                            else
                                H_ref.ref[jnt] += dr[jnt];

                        }
                        else if( ctrl.joint[jnt].mode == CTRL_POS )
                        {

                            if( ctrl.joint[jnt].position < ctrl.joint[jnt].pos_min )
                                ctrl.joint[jnt].position = ctrl.joint[jnt].pos_min;
                            else if( ctrl.joint[jnt].position > ctrl.joint[jnt].pos_max )
                                ctrl.joint[jnt].position = ctrl.joint[jnt].pos_max;

                            dr[jnt] = ctrl.joint[jnt].position - H_ref.ref[jnt]; // Check how far we are from desired position

                            ctrl.joint[jnt].velocity = sign(dr[jnt])*fabs(ctrl.joint[jnt].speed); // Set velocity into the correct direction


                            dV[jnt] = ctrl.joint[jnt].velocity - V0[jnt]; // Check how far we are from desired velocity

                            adr = sqrt(fabs(2.0*ctrl.joint[jnt].acceleration*dr[jnt]));
                            if( fabs(V0[jnt]) >= adr ) // Slow down before reaching goal
                                dV[jnt] = sign(dr[jnt])*adr-V0[jnt];

                            if( dV[jnt] > fabs(ctrl.joint[jnt].acceleration*dt) ) // Scale it down to be within bounds
                                dV[jnt] = fabs(ctrl.joint[jnt].acceleration*dt);
                            else if( dV[jnt] < -fabs(ctrl.joint[jnt].acceleration*dt) ) // Make sure the sign is correct
                                dV[jnt] = -fabs(ctrl.joint[jnt].acceleration*dt);

                            V[jnt] = V0[jnt] + dV[jnt]; // Step velocity forward

                            if( fabs(dr[jnt]) > fabs(V[jnt]*dt) && V[jnt]*dr[jnt] >= 0 )
                                dr[jnt] = V[jnt]*dt;
                            else if( fabs(dr[jnt]) > fabs(V[jnt]*dt) && V[jnt]*dr[jnt] < 0 )
                                dr[jnt] = -V[jnt]*dt;

/*                            if( H_ref.ref[jnt]+dr[jnt] < ctrl.joint[jnt].pos_min )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_min;
                            else if( H_ref.ref[jnt]+dr[jnt] > ctrl.joint[jnt].pos_max )
                                H_ref.ref[jnt] = ctrl.joint[jnt].pos_max;
                            else
                                H_ref.ref[jnt] += dr[jnt];*/
                            H_ref.ref[jnt] += dr[jnt];
                        }
                        else if( ctrl.joint[jnt].mode == CTRL_HOME )
                        {
                            V[jnt]=0; V0[jnt]=0; dV[jnt]=0;
                        }
                        else
                        {
                            fprintf(stderr, "Joint %d has invalid control mode: %d\n",
                                jnt, (int)ctrl.joint[jnt].mode );
                        }

//                        timeElapse[jnt] += dt;
//                        V0[jnt] = V[jnt];
//                        C_state.velocity[jnt] = V[jnt];
                        reset[jnt]=0;
                    }

                }
                else if(fail[jnt]==0 && ctrl.joint[jnt].mode != CTRL_HOME)
                {
                    fprintf(stderr, "JOINT %d FROZEN! Exceeded error limit(%g):%g, Ref:%f, State:%f\n", jnt,
                        fabs(errorFactor*ctrl.joint[jnt].error_limit*dtMax), err, H_ref.ref[jnt], H_state.joint[jnt].pos );
                    V[jnt]=0; V0[jnt]=0;
                    H_ref.ref[jnt] = H_state.joint[jnt].pos;
                    fail[jnt]=1;
                    C_state.status[jnt] = 1;
                }
                else if( ctrl.joint[jnt].mode == CTRL_RESET && reset[jnt]==0 )
                {
                    fprintf(stdout, "Joint %d has been reset\n", jnt);
                    V[jnt]=0; V0[jnt]=0; dV[jnt]=0;
                    dr[jnt]=0;
                    fail[jnt]=0;
                    reset[jnt]=1;
                    C_state.status[jnt] = 0;
                }

                if( H_ref.ref[jnt] != H_ref.ref[jnt] && fail[jnt]==0 )
                {
                    fprintf( stderr, "JOINT FROZEN! You have requested a NaN for joint #%d!\n", jnt );
                    fail[jnt] = 1;
                    C_state.status[jnt] = 1;
                }                    

            } // end: for loop


            if(ctrl.active == 1 && C_state.paused==0) 
            {
                presult = ach_put( &chan_hubo_ref, &H_ref, sizeof(H_ref) );
                if(presult != ACH_OK)
                    fprintf(stderr, "Error sending ref command! (%d) %s\n",
                        presult, ach_result_to_string(presult));
            }
            
        }// end: time test
        else if( dt >= 0.1 )
            fprintf(stderr, "Experiencing Delay of %f seconds\n", dt);
        else if( dt < 0 )
            fprintf(stderr, "Congratulations! You have traveled backwards"
                            " through time by %f seconds!", -dt);

        fflush(stdout);
        fflush(stderr);
    } // End of Main Control Loop

}





int main(int argc, char **argv)
{
    // TODO: Parse runtime arguments


    daemonize( "control-daemon", 49 );
   
    int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME, NULL);
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

    controlLoop();

    daemon_close();
}


int setCtrlDefaults( struct hubo_control *ctrl )
{
    struct hubo_arm_control ractrl;
    struct hubo_arm_control lactrl;
    struct hubo_leg_control rlctrl;
    struct hubo_leg_control llctrl;
    struct hubo_fin_control rfctrl;
    struct hubo_fin_control lfctrl;
    struct hubo_bod_control bodctrl;
    struct hubo_nck_control nckctrl;

	memset( ctrl, 0, sizeof(struct hubo_control) );
	memset( &ractrl, 0, sizeof(struct hubo_arm_control) );
	memset( &lactrl, 0, sizeof(struct hubo_arm_control) );
	memset( &rlctrl, 0, sizeof(struct hubo_leg_control) );
	memset( &llctrl, 0, sizeof(struct hubo_leg_control) );
	memset( &rfctrl, 0, sizeof(struct hubo_fin_control) );
	memset( &lfctrl, 0, sizeof(struct hubo_fin_control) );
	memset( &bodctrl, 0, sizeof(struct hubo_bod_control) );
    memset( &nckctrl, 0, sizeof(struct hubo_nck_control) );

	FILE *ptr_file;

	// open file for read access and if it fails, return -1
	if (!(ptr_file=fopen(ctrlFileLocation, "r")))
    {
        fprintf(stderr, "Unable to locate %s\n -- Try reinstalling or reconfiguring!\n",ctrlFileLocation);
		return -1;
    }
	// instantiate stucts for getting values from control.table
	// file and copying them to
	struct hubo_joint_control tempJC;
    memset( &tempJC, 0, sizeof(tempJC) );


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
		if (8 == sscanf(buff, "%s%lf%lf%lf%lf%lf%lf%s",
			name,
			&tempJC.speed,
			&tempJC.acceleration,
			&tempJC.error_limit,
			&tempJC.pos_min,
			&tempJC.pos_max,
            &tempJC.timeOut,
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
                    lactrl.count++;
                }
                
                else if( strcmp(type, "RA") == 0 )
                {
                    memcpy( &ractrl.joint[ractrl.count], &tempJC, sizeof(tempJC) );
                    ractrl.jointIndices[ractrl.count] = i;
                    ractrl.count++;
                }
                
                else if( strcmp(type, "LL") == 0 )
                {
                    memcpy( &llctrl.joint[llctrl.count], &tempJC, sizeof(tempJC) );
                    llctrl.jointIndices[llctrl.count] = i;
                    llctrl.count++;
                }
                
                else if( strcmp(type, "RL") == 0 )
                {
                    memcpy( &rlctrl.joint[rlctrl.count], &tempJC, sizeof(tempJC) );
                    rlctrl.jointIndices[rlctrl.count] = i;
                    rlctrl.count++;
                }
                
                else if( strcmp(type, "LF") == 0 )
                {
                    memcpy( &lfctrl.joint[lfctrl.count], &tempJC, sizeof(tempJC) );
                    lfctrl.jointIndices[lfctrl.count] = i;
                    lfctrl.count++;
                }
                
                else if( strcmp(type, "RF") == 0 )
                {
                    memcpy( &rfctrl.joint[rfctrl.count], &tempJC, sizeof(tempJC) );
                    rfctrl.jointIndices[rfctrl.count] = i;
                    rfctrl.count++;
                }
                
                else if( strcmp(type, "BD") == 0 )
                {
                    memcpy( &bodctrl.joint[bodctrl.count], &tempJC, sizeof(tempJC) );
                    bodctrl.jointIndices[bodctrl.count] = i;
                    bodctrl.count++;
                }
                
                else if( strcmp(type, "NK") == 0 )
                {
                    memcpy( &nckctrl.joint[nckctrl.count], &tempJC, sizeof(tempJC) );
                    nckctrl.jointIndices[nckctrl.count] = i;
                    nckctrl.count++;
                }
                
            } // end: if (jnkNameCheck != 1)
		} // end: sscanf
	} // end: fgets

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







