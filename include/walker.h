

#define BALANCE_PARAM_CHAN "balance-param"


typedef struct nudge_state {

    Eigen::Vector3d vprev;
    Eigen::Vector3d verr;


    Eigen::Vector3d nudge;
    Eigen::Vector3d spin;
    double larerr; // Left Ankle Roll compliance
    double laperr; // Left Ankle Pitch compliance
    double rarerr;
    double raperr;

    double knee_offset[2];

    Eigen::Vector3d imu_offset;
    
} nudge_state_t;


typedef struct balance_gains {

    double flattening_gain[2];

    double straightening_gain[2];
    
    double spring_gain[2];
    double damping_gain[2];
    double fz_response[2];

} balance_gains_t;





const double hipDistance = 0.08843*2.0; // Distance between hip joints

typedef enum {

    STATE_INVALID,
    S_HORSE,
    S_CRANE,
    Q_SHIFTDIST,
    Q_LIFTLEG,
    Q_CROUCH


} balance_state_t;


typedef enum {
    
    T_INVALID,
    T_INCOMPLETE,
    T_COMPLETE

} transition_result_t;




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




