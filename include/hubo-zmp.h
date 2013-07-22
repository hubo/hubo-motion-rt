/**
 * \file hubo-zmp.h
 * \brief Header containing enumerations, structs and 
 * ach channel names used to communicate zmp commands,
 * zmp trajectories and walker state between the rviz
 * GUI and the zmp-daemon, and between the zmp-daemon
 * and the walker
 *
 * \author M.X. Grey
*/

#ifndef _HUBO_ZMP_H_
#define _HUBO_ZMP_H_

#include <stdlib.h>
#include <vector>
#include "stdint.h"
#include <hubo.h> //!< hubo main include

/// Ach channel name for zmp trajectory
#define HUBO_CHAN_ZMP_TRAJ_NAME "hubo-zmp-traj"

/// Ach channel name for walker state
#define HUBO_CHAN_WALKER_STATE_NAME "walker-state"

/**
 * \brief Current stance. The first four are set for each
 * timestep of the zmp trajectory. The last three are used
 * to determine which atomic trajectory to used
*/
enum stance_t {
  DOUBLE_LEFT  = 0, //!< double support stance, left dominant
  DOUBLE_RIGHT = 1, //!< double support stance, right dominant
  SINGLE_LEFT  = 2, //!< single support stance, left dominant
  SINGLE_RIGHT = 3, //!< single support stance, right dominant
};

/**
 * \brief Different stance types for atomic trajectories, used
 * for all walk types.
*/
enum stepStance_t {
  EVEN,             //!< feet are even and normal width
  LEFT_DOM,         //!< left foot is dominant or in front
  RIGHT_DOM,        //!< right foot is dominant or in front
  NUM_OF_STANCES = 3//!< number of stance types. used for 3D trajectory array
};

/// String constants for stepStance enum
static const char* stepStanceStrings[NUM_OF_STANCES] = {"EVEN",
                                                        "LEFT_DOM",
                                                        "RIGHT_DOM"};

/**
 * \brief Walk direction state
*/
enum walkState_t {
  WALKING_FORWARD = 0,  //!< walking forward state
  WALKING_BACKWARD,     //!< walking backward state
  ROTATING_LEFT,        //!< turning-in-place to the left state
  ROTATING_RIGHT,       //!< turning-in-place to the right state
  SIDESTEPPING_LEFT,    //!< sidestepping to the left state
  SIDESTEPPING_RIGHT,   //!< sidestepping to the right state
  TURNING_LEFT,         //!< turning left while walking forward/backward state
  TURNING_RIGHT,        //!< turning right while walking forward/backward state
  STOP,                 //!< stopped state
  NUM_OF_WALKSTATES = 6 //!< number of walk states to get trajectories for
};

/// String constants for walkState enum
static const char* walkStateStrings[NUM_OF_WALKSTATES+2] = {"WALKING_FORWARD", "WALKING_BACKWARD", 
                                                        "ROTATING_LEFT", "ROTATING_RIGHT",
                                                        "SIDESTEPPING_LEFT", "SIDESTEPPING_RIGHT",
                                                        "TURNING_LEFT", "TURNING_RIGHT"};
 
/// ZMP trajectory constants
enum {
  ZMP_TRAJ_FREQ_HZ = 200,   //!< frequency in Hertz of the zmp trajectory
  ZMP_MAX_TRAJ_SIZE = 8000  //!< maximum size of the zmp trajectory
};

/**
 * \brief Struct containing robot state for each timestep of trajectory
*/
typedef struct zmp_traj_element {
  double angles[HUBO_JOINT_COUNT]; //!< fully body joint angles for current timestep
  double com[3][3];     //!< XYZ pos/vel/acc of CoM in frame of stance ankle, 10cm up from foot
  double zmp[2];        //!< XY position of zmp in frame of stance ankle
  double forces[2][3];  //!< right/left predicted normal forces at ankles
  double torque[2][3];  //!< right/left predicted moments XYZ at ankles
  stance_t stance;      //!< current stance of robot
  // TODO: add orientation for IMU
} zmp_traj_element_t;
//}__attribute__((packed)) zmp_traj_element_t;

/**
 * \brief Struct containing entire zmp trajectory, including joint configuration
 * for each timestep, and trajectory meta data
*/
typedef struct zmp_traj {
  zmp_traj_element_t traj[ZMP_MAX_TRAJ_SIZE]; //!< array of entire zmp trajectory info
  size_t count;             //!< size of first step trajectory in timesteps
  size_t end;               //!< size of two-step trajectory ending in EVEN stance
  size_t trajNumber;        //!< trajectory number
  size_t periodStartTick;   //!< start timestep of periodic portion of trajectory
  size_t periodEndTick;     //!< end timestep of periodic portion of trajectory
  walkState_t walkDirection;//!< walk direction for trajectory
  stepStance_t startStance; //!< start stance for trajectory
  stepStance_t goalStance;  //!< goal stance for trajectory
  bool reuse;               //!< whether or not to reuse the current trajectory's periodic portion
} zmp_traj_t;
//}__attribute__((packed)) zmp_traj_t;

/**
 * \brief Struct containing walker state information so the zmp-daemon knows what
 * the walker is doing and what trajectories to send next
*/
typedef struct walker_state {
  walkState_t walkDirection;    //!< walk direction being executed
  stepStance_t startStance;     //!< start stance of current step
  stepStance_t goalStance;      //!< goal stance of current step
  int cyclesLeft;               //!< cycles left in current step trajectory
} walker_state_t;
//}__attribute__((packed)) walker_state_t;


#endif // _HUBO_ZMP_H_
