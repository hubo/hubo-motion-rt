#ifndef MOTIONTRAJECTORY_H
#define MOTIONTRAJECTORY_H

extern "C" {
#include <hubo.h>
#include <stdio.h>
}

#define MOTION_TRAJ_CMD_CHAN "motion-cmd"
#define MOTION_TRAJ_CHAN "motion-traj"
#define MOTION_STATE_CHAN "motion-state"


enum { ABSOLUTE_MAX_TRAJECTORY_SIZE = 500000 };


typedef enum {

    TRAJ_READY = 0,
    TRAJ_GOTO_INIT,
    TRAJ_RUN,
    TRAJ_PAUSE,
    TRAJ_STOP

} motion_traj_type_t;

typedef enum {

    TRAJ_OFF = 0,
    TRAJ_PARSING,
    TRAJ_INITIALIZING,
    TRAJ_INITIALIZED,
    TRAJ_RUNNING,
    TRAJ_FINISHED

} motion_state_type_t;

typedef struct motion_state {

    motion_state_type_t mode;

    int iteration;

}__attribute__((packed)) motion_state_t;

typedef struct motion_traj_params {

    bool useFile;
    char filename[100];

    bool com_balancing;
    bool upper_body_compliance;
    bool pose_correction;

    double zmp_ref_x;
    double zmp_ref_y;

    double kp_zmp_com;
    double zmp_com_max;

    int negate_moments;

}__attribute__((packed)) motion_traj_params_t;

typedef struct motion_traj_cmd {

    motion_traj_type_t type;

    motion_traj_params_t params;

}__attribute__((packed)) motion_traj_cmd_t;


typedef struct motion_element {

    double angles[HUBO_JOINT_COUNT];

}__attribute__((packed)) motion_element_t;


bool parseNextElement(const char* filename, int line, FILE* fp, motion_element_t& elem);


#ifdef __cplusplus

#include <vector>

typedef std::vector<motion_element_t> motion_trajectory_t;

bool fillMotionTrajectoryFromText(const char* filename, motion_trajectory_t& motion_trajectory);



#endif // __cplusplus


#endif // MOTIONTRAJECTORY_H
