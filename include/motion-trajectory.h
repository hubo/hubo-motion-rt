#ifndef MOTION_TRAJ_H
#define MOTION_TRAJ_H

#include "Hubo_Control.h"


#define HUBO_TRAJ_CHAN "hubo-traj"
#define HUBO_TRAJ_STATE_CHAN "hubo-traj-state"

typedef struct hubo_joint_traj {

    double position[MAX_TRAJ_SIZE];
    double velocity[MAX_TRAJ_SIZE];
    double acceleration[MAX_TRAJ_SIZE];

} __attribute__((packed)) hubo_joint_traj_t;

typedef struct hubo_traj {

    hubo_joint_traj_t joint[HUBO_JOINT_COUNT];    
    double time[MAX_TRAJ_SIZE];

    double endTime;
    int trajID;

} __attribute__((packed)) hubo_traj_t;


typedef enum {
    
    TRAJ_RUNNING,
    TRAJ_COMPLETE

} traj_status_t;

typedef struct hubo_traj_output {

    traj_status_t status;
    int trajID;
    
    double error[HUBO_JOINT_COUNT];

} __attribute__((packed)) hubo_traj_output_t;

#endif // MOTION_TRAJ_H
