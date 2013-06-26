#include "motion-trajectory.h"

ach_channel_t traj_chan;
ach_channel_t traj_state_chan;

int main( int argc, char **argv )
{

    Hubo_Control hubo("balance-daemon");

    ArmVector rArmSpeedDef, 
              rLegSpeedDef, 
              rArmAccDef,   
              rLegAccDef;

    LegVector lArmSpeedDef,
              lLegSpeedDef,
              lArmAccDef,
              lLegAccDef;

    hubo.getArmNomSpeeds( RIGHT, rArmSpeedDef );
    hubo.getArmNomSpeeds( LEFT, lArmSpeedDef );
    hubo.getLegNomSpeeds( RIGHT, rLegSpeedDef );
    hubo.getLegNomSpeeds( LEFT, lLegSpeedDef );

    hubo.getArmNomAcc( RIGHT, rArmAccDef );
    hubo.getArmNomAcc( LEFT, lArmAccDef );
    hubo.getLegNomAcc( RIGHT, rLegAccDef );
    hubo.getLegNomAcc( LEFT, lLegAccDef );
    
    hubo_traj_t traj;
    hubo_traj_output_t output;

    ach_open( &traj_chan, HUBO_TRAJ_CHAN, NULL );
    ach_open( &traj_state_chan, HUBO_TRAJ_STATE_CHAN, NULL );
    
    while( !daemon_sig_quit )
    {
        size_t fs;
        ach_status_t r;
        r = ach_get( &traj_chan, &traj, sizeof(traj), &fs, NULL, ACH_O_WAIT );
        if( ACH_OK != r )
            fprintf(stderr, "Warning: (%d) %s\n", (int)r, ach_result_to_string(r) );

        output.trajID = traj.trajID;
        output.status = TRAJ_RUNNING;
        ach_put( &traj_state_chan, &output, sizeof(output) );

        hubo.update(true);
        double clock = 0, start = hubo.getTime();
        int i=0;
        while( clock <= traj.endTime )
        {
            hubo.update(true);
            clock = hubo.getTime() - start;

            while( i<MAX_TRAJ_SIZE-1 && traj.time[i+1] <= clock
                                     && traj.time[i+1] != 0 )
                i++;

            for( int j=0; j < HUBO_JOINT_COUNT; j++ )
            {
                hubo.setJointAngle( j, traj.joint[j].position[i] );
                hubo.setJointNominalSpeed( j, traj.joint[j].velocity[i] );
                hubo.setJointNominalAcceleration( j, traj.joint[j].acceleration[i] );

                output.error[j] = traj.joint[j].position[i] - hubo.getJointAngleState(i);
            }

            output.status = TRAJ_RUNNING;
            ach_put( &traj_state_chan, &output, sizeof(output) );
            
            hubo.sendControls();

        }
        
        output.status = TRAJ_COMPLETE;
        ach_put( &traj_state_chan, &output, sizeof(output) );
        
    }


    hubo.setArmNomSpeeds( RIGHT, rArmSpeedDef );
    hubo.setArmNomSpeeds( LEFT, lArmSpeedDef );
    hubo.setLegNomSpeeds( RIGHT, rLegSpeedDef );
    hubo.setLegNomSpeeds( LEFT, lLegSpeedDef );

    hubo.setArmNomAcc( RIGHT, rArmAccDef );
    hubo.setArmNomAcc( LEFT, lArmAccDef );
    hubo.setLegNomAcc( RIGHT, rLegAccDef );
    hubo.setLegNomAcc( LEFT, lLegAccDef );




}
