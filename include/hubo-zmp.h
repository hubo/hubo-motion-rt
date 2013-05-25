#ifndef _HUBO_ZMP_H_
#define _HUBO_ZMP_H_

#include <stdlib.h>
#include <vector>
#include "stdint.h"

#include <hubo.h>

enum stance_t {
  DOUBLE_LEFT  = 0,
  DOUBLE_RIGHT = 1,
  SINGLE_LEFT  = 2,
  SINGLE_RIGHT = 3
};

enum walkState_t {
  WALKING_FORWARD = 0,
  WALKING_BACKWARD,
  SIDESTEPPING_LEFT,
  SIDESTEPPING_RIGHT,
  STOP
};

enum walkTransition_t {
  STAY_STILL = 0,
  KEEP_WALKING,
  SWITCH_WALK,
  WALK_TO_STOP
};


enum {
  ZMP_TRAJ_FREQ_HZ = 200,
  ZMP_MAX_TRAJ_SIZE = 2000
};


typedef struct zmp_traj_element {
  double angles[HUBO_JOINT_COUNT];
  // XYZ pos/vel/accel in frame of stance ANKLE
  // (translated up ~10cm from foot -- i.e. z coord is 10cm less)
  double com[3][3];
  double zmp[2]; // XY of zmp in frame of stance ANKLE
  double forces[2][3]; // right/left predicted normal forces
  double torque[2][3]; // right/left predicted moments XYZ
  // TODO: add orientation for IMU
  stance_t stance;
} zmp_traj_element_t;

typedef struct zmp_traj {
  zmp_traj_element_t traj[ZMP_MAX_TRAJ_SIZE];
  size_t count;
  size_t trajNumber;
  size_t startTick;
  walkState_t walkState;
  walkTransition_t walkTransition;
} zmp_traj_t;

/*
typedef struct zmp_traj_element {
  uint64_t angles[HUBO_JOINT_COUNT];
  // XYZ pos/vel/accel in frame of stance ANKLE
  // (translated up ~10cm from foot -- i.e. z coord is 10cm less)
  uint64_t com[3][3];
  uint64_t zmp[2]; // XY of zmp in frame of stance ANKLE
  uint64_t forces[2][3]; // right/left predicted normal forces
  uint64_t torque[2][3]; // right/left predicted moments XYZ
  // TODO: add orientation for IMU
  stance_t stance;
} zmp_traj_element_t;

typedef struct zmp_traj {
  zmp_traj_element_t traj[MAX_TRAJ_SIZE];
  uint16_t count;
  uint16_t trajNumber;
  uint16_t startTick;
  walkState_t walkState;
  walkTransition_t walkTransition;
} zmp_traj_t;
*/

#define HUBO_CHAN_ZMP_TRAJ_NAME "hubo-zmp-traj"

#endif


