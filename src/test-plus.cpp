#include "Hubo_Tech.h"
#include <iostream>
#include <vector>

int main(int argc, char **argv)
{
	
	Hubo_Tech hubo;

    hubo.update();
    double t = hubo.getTime();
    while( hubo.getTime() - t < 2 )
    {
        hubo.setJointVelocity( RSP, 0.5, true );
        hubo.update();
    }





/*
    int i=0, imax=0;

    hubo.setJointNominalAcceleration( LAR, 0.6 );
    hubo.setJointNominalAcceleration( LAP, 0.6 );
    hubo.setJointNominalAcceleration( RAP, 0.6 );
    hubo.setJointNominalAcceleration( RAR, 0.6 );

    hubo.setJointNominalAcceleration( RKN, 0.6 );
    hubo.setJointNominalAcceleration( RHP, 0.6 );
    hubo.setJointNominalAcceleration( LKN, 0.6 );
    hubo.setJointNominalAcceleration( LHP, 0.6 );

    hubo.setJointNominalAcceleration( RHR, 0.6 );
    hubo.setJointNominalAcceleration( LHR, 0.6 );


    hubo.setJointAngleMax( RHP, 0 );
    hubo.setJointAngleMax( LHP, 0 );

    double L1 = 2*0.3002;
    double L2 = 0.28947 + 0.0795;
    double height = 0;
    Eigen::Vector3d trans; Eigen::Quaterniond quat;

    double leftP=0, leftR=0, rightP=0, rightR=0, ptime, dt, knee, kneeVel, kneeAngleError;

    double compRollGain = 0.0035;
    double compPitchGain = 0.0035;
    double pitchAngleGain = 0.5*M_PI/180.0;
    double pitchRotVelGain = 0.0*M_PI/180;
    double rollAngleGain = 0.3*M_PI/180;
    double rollRotVelGain = 0.0*M_PI/180;
    double integralGain = 0; //0.05*M_PI/180;
    double kneeVelGain = 0.2;
    double leftPIntegral = 0.0;
    double leftRIntegral = 0.0;
    double rightPIntegral = 0.0;
    double rightRIntegral = 0.0;
    double pTiltIntegral = 0.0;
    double rTiltIntegral = 0.0;

    double springGainUp = 0.001;
    double springGainDown = 0.0075;

    double compLP;
    double compLR;
    double compRP;
    double compRR;
    hubo.update();
    ptime = hubo.getTime();
    double atime = hubo.getTime();

    while(!daemon_sig_quit)
    {
        hubo.update();

        dt = hubo.getTime() - ptime;
        ptime = hubo.getTime();
        atime += dt;

        if( dt > 0 )
        {
            i++; if(i>imax) i=0;

            compLP = hubo.getLeftFootMy();
            compLR = hubo.getLeftFootMx();
            compRP = hubo.getRightFootMy();
            compRR = hubo.getRightFootMx();

            height = trans(2) + L1 + L2;
            if( height-L2 > L1 )
                height = L1+L2;
            else if( height-L2 < 0.25 )
                height = L1 + 0.2; // TODO: Check if this is a good value

            knee = acos( (height-L2)/L1 )*2;
            kneeAngleError = knee - hubo.getJointAngle( RKN );

            kneeVel = kneeVelGain*kneeAngleError;

            leftP = pitchAngleGain*hubo.getAngleY() - compPitchGain*compLP + (-kneeVel/2);
            leftR = rollAngleGain*hubo.getAngleX() - compRollGain*compLR;
            rightP = pitchAngleGain*hubo.getAngleY() - compPitchGain*compRP + (-kneeVel/2);
            rightR = rollAngleGain*hubo.getAngleX() - compRollGain*compRR;

            hubo.setJointVelocity( RAP, rightP );
            hubo.setJointVelocity( RAR, rightR );
            hubo.setJointVelocity( LAP, leftP );
            hubo.setJointVelocity( LAR, leftR );

            hubo.setJointVelocity( RKN, kneeVel );
            hubo.setJointVelocity( RHP, -kneeVel/2.0 );
            hubo.setJointVelocity( LKN, kneeVel );
            hubo.setJointVelocity( LHP, -kneeVel/2.0 );


            std::cout << hubo.getLeftFootMy() << "\t" << hubo.getLeftFootMx() << "\t" << hubo.getRightFootMy() << "\t" << hubo.getRightFootMx() << std::endl;
//            hubo.setJointVelocity( RHR, -rightR ); // These cannot be permitted to comply
//            hubo.setJointVelocity( LHR, -leftR );

            knee = ( hubo.getLeftFootFz() + hubo.getRightFootFz() - 440 );

            if( knee > 0 )
                knee *= springGainDown;
            else
                knee *= springGainUp;
            

            if( knee >= 0.3 )
                knee = 0.3;
            else if( knee <= -0.3 )
                knee = -0.3;
            hubo.sendControls();

        }
    }

*/












//    Vector6d acc;
//    acc << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
//    hubo.setRightArmNomAcc( acc );
/*
    Vector6d angles1, angles2, check;
    angles1 << -0.6703, -0.101632, 0.3296, -1.8217, -1.9074, -0.0;
    angles2 << -0.6055, -0.0782,  -0.2416, -1.6655, -1.9774, 0.0;

    hubo.setRightArmAngles( angles1, true ); 
    std::cout << "Arm angles set to Angles1" << std::endl;
    hubo.update();
    hubo.getRightArmAngles( check );
    while( (angles1-check).norm() > 0.075 )
    {
        hubo.update();
        hubo.getRightArmAngleStates( check );
    }

    hubo.setRightArmAngles( angles2, true );
    std::cout << "Arm angles set to Angles2" << std::endl;*/
/*
    while(true)
    {
        hubo.update();
        printf("RMx:%3.2f\tRMy:%3.2f\tRFz:%3.2f\tLMx:%3.2f\tLMy:%3.2f\tLFz:%3.2f\n",
                hubo.getRightFootMx(), hubo.getRightFootMy(), hubo.getRightFootFz(),
                hubo.getLeftFootMx(),  hubo.getLeftFootMy(),  hubo.getLeftFootFz() );
        usleep(50000);
    }
*/


//    Vector6d qr;
//    qr << 0, 0, 0, -M_PI/2, -0.3, 0;

//    std::vector<Vector6d> angles(5);
//    std::vector<Vector6d> angles(2);
//    angles[1] << -0.6703, -0.101632, 0.3296, -1.8217, -1.9074, -0.0;
//    angles[0] << -0.6055, -0.0782,  -0.2416, -1.6655, -1.9774, 0.0;
//    angles[0] << -0.6055, -0.0782,  -0.2416, -1.6655, 0.0, 0.0;
    
/*    angles[0] <<  0.0556916,   0.577126,  0.0816814,  -0.492327, 0, 0;
    angles[1] <<  -1.07878,  0.408266, -0.477742, -0.665062, 0, 0;
    angles[2] <<   -1.17367, -0.0540511,  -0.772141,  -0.503859, 0, 0;
    angles[3] <<  -0.518417,   0.172191,  -0.566084, -0.0727671, 0, 0;
    angles[4] << 0, 0, 0, 0, 0, 0;
*/
/*
    Vector6d q;
    double y=0, V=0, tprev=hubo.getTime(), dt=0;
    int i=0, imax=100;
    hubo.HuboDrillIK( q, y );

    hubo.setRightArmAngles( q, true );

    Vector6d current;
    hubo.update();
    hubo.getRightArmAngles( current );

    while( (current-q).norm() > 0.075 )
    {
        hubo.update();
        hubo.getRightArmAngles( current );
    }

    V = -( 0.1850 + 0.2050 )/8.0;
    
    while(true)
    {
        i++;
        if(i>imax) i = 0;

        hubo.update();
        dt = hubo.getTime() - tprev;
        tprev = hubo.getTime();
        if( y > 0.170 )
            V = -fabs(V);
        else if( y < -0.190 )
            V = fabs(V);
        y += V*dt; if(i==imax) std::cout << y << std::endl;
        hubo.HuboDrillIK( q, y );

        hubo.setRightArmAngles( q );

        hubo.sendControls();
   } 
*/  
    
/*    
    double tol = 0.075;
    int traj = 0;
    while(true)
    {
        hubo.update();
        
        hubo.getRightArmAngles( current );
//      hubo.getLeftArmAngleStates( current );
        if( (current-angles[traj]).norm() < tol )
        {
            traj++;
            if(traj > 2)
                traj = 0;
        }

//        hubo.setRightArmAngles( angles[traj] );
//      hubo.setLeftArmAngles( angles[traj] );

        hubo.sendControls();
    }
*/
/*
    Vector6d angles;

    while(true)
    {
        hubo.getRightLegAngleStates( angles );
        std::cout << angles.transpose() << std::endl;
        hubo.update();
        usleep(10000);
    }
*/
    
}





