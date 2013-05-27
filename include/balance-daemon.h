
#ifndef BALANCE_DAEMON_H
#define BALANCE_DAEMON_H

#define BALANCE_PARAM_CHAN "balance-param"


typedef struct balance_gains {

    double flattening_gain[2];
    double force_min_threshold[2];
    double force_max_threshold[2];

    double straightening_pitch_gain[2];
    double straightening_roll_gain[2];
    
    double spring_gain[2];
    double damping_gain[2];
    double fz_response[2];

} balance_gains_t;






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


#endif // BALANCE_DAEMON_H

