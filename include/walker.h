

#ifndef WALKER_H
#define WALKER_H


#include "balance-daemon.h"
#include "Hubo_Control.h"

typedef struct nudge_state {

    Eigen::Vector3d vprev;
    Eigen::Vector3d verr;


    Eigen::Vector3d nudge;
    Eigen::Vector3d spin;
    double ankle_roll_compliance[2];
    double ankle_pitch_compliance[2];
    double ankle_roll_resistance[2];
    double ankle_pitch_resistance[2];

    double knee_offset[2];

    Eigen::Vector3d imu_offset;
    
} nudge_state_t;







/**
 * The role of this class is to provide a
 * state machine and Kalman filter for
 * balancing and walking operations
*/
class Balance_Monitor
{

public:
    Balance_Monitor();
    

    Eigen::Vector2d shiftFilter( double FLsensor, double FRsensor, double dt );

    bool checkTransition( balance_state_t to ); // TODO: Fill this in all the way


    double hipVelocity; // TODO: Consider making this protected

protected:
    
    // Weight Shifting
    double FRprev;
    double FLprev;
    double Pprev;

    

    balance_state_t state;
 
    

    



};


#endif // WALKER_H

