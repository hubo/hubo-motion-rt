/**
 * \file hubo-zmp.h
 * \brief Header containing enumerations, structs and 
 * ach channel names used to communicate zmp commands,
 * zmp trajectories and walker state between the rviz
 * GUI and the zmp-daemon, and between the zmp-daemon
 * and the walker
 *
 * \author M.X. Grey and friends
*/

#ifndef _HUBO_ZMP_H_
#define _HUBO_ZMP_H_

#include <stdlib.h>
#include <string>
#include <ostream>
#include <vector>
#include "stdint.h"
#include <hubo.h> //!< hubo main include

/// Ach channel name for zmp trajectory
#define HUBO_CHAN_ZMP_TRAJ_NAME "hubo-zmp-traj"

/// Ach channel name for zmp-daemon state
#define HUBO_CHAN_ZMP_STATE_NAME "hubo-zmp-state"

/// Ach channel name for walker state
#define HUBO_CHAN_WALKER_STATE_NAME "walker-state"

/**
 * \brief End effector enum for hands and feet
*/
enum effector_t {
  EFFECTOR_L_FOOT,
  EFFECTOR_R_FOOT,
  EFFECTOR_L_HAND,
  EFFECTOR_R_HAND
};

/**
 * \brief Current stance. The first four are set for each
 * timestep of the zmp trajectory. The last three are used
 * to determine which atomic trajectory to used
*/
enum bipedStance_t {
  DOUBLE_LEFT  = 0, //!< double support stance, left dominant
  DOUBLE_RIGHT = 1, //!< double support stance, right dominant
  SINGLE_LEFT  = 2, //!< single support stance, left dominant
  SINGLE_RIGHT = 3, //!< single support stance, right dominant
};

/**
 * \brief Different stance types for atomic trajectories, used
 * for all walk types.
*/
typedef enum {
  EVEN,             //!< feet are even and normal width
  LEFT_DOM,         //!< left foot is dominant or in front
  RIGHT_DOM,        //!< right foot is dominant or in front
  NUM_OF_STANCES = 3//!< number of stance types. used for 3D trajectory array
} stepStance_t;

/*
/// String constants for stepStance enum
static const char* stepStanceStrings[NUM_OF_STANCES] = {"EVEN",
                                                        "LEFT_DOM",
                                                        "RIGHT_DOM"};
*/

const char* stepStanceString(int i);

/**
 * \brief: walk type
*/
typedef enum walktype {
  WALK_FORWARD,      //!< walking forward state
  WALK_BACKWARD,     //!< walking backward state
  SIDESTEP_LEFT,     //!< sidestepping to the left state
  SIDESTEP_RIGHT,    //!< sidestepping to the right state
  ROTATE_LEFT,       //!< turning-in-place to the left state
  ROTATE_RIGHT,      //!< turning-in-place to the right state
  GOTO_BIPED,        //!< go from biped stance into quadruped stance
  GOTO_QUADRUPED,    //!< go from quadruped stance into biped stance
  STOP_WALKING,      //!< stop walking
  NUM_OF_WALKTYPES   //!< number of walk types
} walktype_t;
             
static const char* walktype_strings[NUM_OF_WALKTYPES] = {"WALK_FORWARD",
                                                         "WALK_BACKWARD",
                                                         "SIDESTEP_LEFT",
                                                         "SIDESTEP_RIGHT",
                                                         "ROTATE_LEFT",
                                                         "ROTATE_RIGHT",
                                                         "GOTO_BIPED",
                                                         "GOTO_QUADRUPED",
                                                         "STOP_WALKING"};

static std::string walktype_to_string(walktype_t walkType)
{
    if(0 <= walkType && walkType < NUM_OF_WALKTYPES)
        return walktype_strings[walkType];
    else
        return "Invalid Walk Type";
}

/**
 * \brief Walk mode enum
*/
typedef enum
{
  MODELESS = 0,     //!< Not in a mode
  BIPED_MODE,       //!< Biped walking mode
  QUADRUPED_MODE,   //!< Quadruped walking mode
  NUM_OF_WALKMODES
} walkMode_t;

static const char* walkMode_strings[NUM_OF_WALKMODES] = {"MODELESS",
                                                         "BIPED_MODE",
                                                         "QUADRUPED_MODE"};

static std::string walkMode_to_string(walkMode_t walkMode)
{
    if(0 <= walkMode && walkMode < NUM_OF_WALKMODES)
        return walkMode_strings[walkMode];
    else
        return "Invalid Walk Mode";
}

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
  double forces[4][3];  //!< right/left predicted normal forces at ankles
  double torque[4][3];  //!< right/left predicted moments XYZ at ankles
  effector_t effector_frame;    //!< Frame the end effector is in
  unsigned char supporting[4];  //!< Supporting limb
  bipedStance_t bipedStance;
  // TODO: add orientation for IMU
}__attribute__((packed)) zmp_traj_element_t;

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
  walktype_t walkDirection;//!< walk direction for trajectory
  stepStance_t startStance; //!< start stance for trajectory
  stepStance_t goalStance;  //!< goal stance for trajectory
  int reuse;               //!< whether or not to reuse the current trajectory's periodic portion
}__attribute__((packed)) zmp_traj_t;


enum ik_error_sensitivity {
  ik_strict,            //!< (default)
  ik_swing_permissive,  //!< allows ik errors on swing foot when above 0.5 * step_height
  ik_sloppy,             //!< never ever ever ever run this on the robot
  num_of_ik_senses
};

static const char* ik_error_sensitivity_strings[num_of_ik_senses] = {"STRICT",
                                                                    "PERMISSIVE",
                                                                    "SLOPPY"};

static std::string ik_sense_to_string(ik_error_sensitivity ikSense)
{
    if(0 <= ikSense && ikSense < num_of_ik_senses)
        return ik_error_sensitivity_strings[ikSense];
    else
        return "Invalid IK Sense";
}

/**
 * \brief Struct containing parameters for ZMP Walking, used by ZMPWalkGenerator and various daemons/demos.
 */
typedef struct zmp_params {

  ik_error_sensitivity ik_sense; //!< How much error is allowed in the IK solution

  double com_height;               //!< Height in meters of center of mass above ANKLE

  double zmp_R;               //!< Jerk penalty on ZMP controller
  double zmpoff_x;            //!< Sagittal (front/back) displacement between zmp and ankle (m)
  double zmpoff_y;            //!< Lateral displacement between zmp and ankle (m)
  double lookahead_time;      //!< Seconds to look ahead in preview controller

  double nominal_foot_pos_rate; //!< nominal velocity of swing foot during swing (m/s)
  double nominal_foot_rot_rate; //!< nominal angular vel. of swing foot during swing (rad/s)
  
  double com_ik_angle_weight; //!< Scale factor on center of mass rotation for solving IK

  double min_single_support_time; //!< Seconds that a single foot is on the ground during each step
  double min_double_support_time; //!< Seconds that both feet are on the ground during each step
  double zmp_dist_gain;           //!< additional seconds of double support per meter of ZMP motion
  double min_pause_time;          //!< Seconds that we hold ZMP stationary before/after double/quad support
  double walk_startup_time;       //!< Seconds to be stationary at startup
  double walk_shutdown_time;      //!< Seconds to be stationary at end of trajectory (includes equalizing)

  double half_stance_width;   //!< Half of horizontal separation distance between feet (m)
  double step_height;         //!< Foot liftoff height in meters
  double step_length;         //!< Desired step length in meters
  double walk_dist;           //!< Total distance in meters to walk

  double sidestep_length;     //!< Length in meters of sidestep
  double sidewalk_dist;       //!< Total distance in meters to sidestep

  size_t max_step_count;      //!< Maximum number of steps for trajectory
  double walk_circle_radius;  //!< Walk circle radius (m)
  double turn_in_place_angle;   //!< Angle in radians to turn in place

  double torso_pitch;           //!< Constant pitch of torso (rad) during walking
  double quad_stance_length;    //!< X-distance (m) from pegs to feet during quad walking
  double quad_transition_time;  //!< How much time (s) to take to ease in/out of quad from biped
  double quad_stability_margin; //!< Desired distance inside support polygon for ZMP in quad mode

  double half_peg_width;        //!< Half of horizontal separation distance between pegs (m)
  double peg_radius;            //!< Radius of pegs for quad walking (m)

  int constant_body_z;          //!< If non-zero, holds body z constant during walking
  int use_fixed_com;            //!< If non-zero, places COM relative to body instead of getting it from mass model

  double footrect_x0;           //!< Negative ankle to rear of foot distance (m)
  double footrect_y0;           //!< Negative ankle to right of foot distance (m)
  double footrect_x1;           //!< Positive ankle to front of foot distance (m)
  double footrect_y1;           //!< Positive ankle to left of foot distance (m)

  double fixed_com_offset_x;    //!< Offset of fixed com from trunk (m)
  double fixed_com_offset_y;    //!< Offset of fixed com from trunk (m)
  double fixed_com_offset_z;    //!< Offset of fixed com from trunk (m)

  

}__attribute__((packed)) zmp_params_t;



/**
 * \brief Struct containing walker state information so the zmp-daemon knows what
 * the walker is doing and what trajectories to send next
*/
typedef struct walker_state {
  walktype_t walkDirection;    //!< walk direction being executed
  stepStance_t startStance;     //!< start stance of current step
  stepStance_t goalStance;      //!< goal stance of current step
  int cyclesLeft;               //!< cycles left in current step trajectory
}__attribute__((packed)) walker_state_t;


#endif // _HUBO_ZMP_H_
