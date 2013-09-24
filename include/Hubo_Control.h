/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Feb 03, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */


/**
 * \file Hubo_Control.h
 * \brief Programming interface for passing joint control and CAN commands to Hubo daemons.
 * 
 * \author M.X. Grey
 */

#ifndef HUBO_CONTROL_H
#define HUBO_CONTROL_H

// C Headers
extern "C" {
// For Hubo
#include "hubo.h"
#include "control-daemon.h"
}
#include <iostream>


// For data handling
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <complex.h>

typedef Eigen::Matrix< double, 6, 1 > Vector6d;
typedef Eigen::Matrix< double, ARM_JOINT_COUNT, 1 > ArmVector;
typedef Eigen::Matrix< double, LEG_JOINT_COUNT, 1 > LegVector;
typedef Eigen::Vector3d Vector3d;

#define CtrlRA  0
#define CtrlLA  1
#define CtrlRL  2
#define CtrlLL  3
#define CtrlRF  4
#define CtrlLF  5
#define CtrlBD  6
#define CtrlNK  7


#define SP      0
#define SR      1
#define SY      2
#define EB      3
#define WY      4
#define WP      5
#define WR      6

#define HY      0
#define HR      1
#define HP      2
#define KN      3
#define AP      4
#define AR      5



typedef enum {
    SUCCESS = 0,    ///< The function completed with no errors
    JOINT_OOB,      ///< The joint you tried to specify is out of bounds
    SENSOR_OOB,     ///< You requested data from a sensor which doesn't exist
    VALUE_OOB,      ///< Some generic value was out of acceptable bounds
    WRONG_MODE,     ///< You are not in the correct control mode to do what you asked
    BAD_SIDE,       ///< You did not use LEFT or RIGHT correctly
    SHORT_VECTOR,   ///< The VectorXd you tried to use has too few entries
    LONG_VECTOR,    ///< The VectorXd you tried to use has too many entries
    REF_STALE,      ///< The reference values were not able to update for some reason
    STATE_STALE,    ///< The state values were not able to update for some reason
    ALL_STALE,      ///< Nothing was able to update for some reason
    CHAN_OPEN_FAIL, ///< A channel failed to open
    IK_EDGE,        ///< You're trying to push something out of bounds with an IK
    IK_DANGER      ///< The IK is on a singularity and is resorting to a safety feature

} ctrl_flag_t;


class DrcHuboKin;


class Hubo_Control
{
public:

    friend class DrcHuboKin;
    /**
     * Constructor for the Hubo_Control class.
    */
    Hubo_Control(bool live=true); /// Use daemonize(const char *daemon_name) after calling this constructor
    /**
     * Constructor for the Hubo_Control class that daemonizes with the name specificied by the parameter.
    */
    Hubo_Control(const char *daemon_name, int priority=-1);
    /**
     * Gets the current time in seconds from Hubo's 'hubo_state' struct member 'time' over the state ach channel
    */
    ~Hubo_Control();
    double getTime();

    /**
     * Returns the latest data from the hubo_state and hubo_ref ach channels.
     * \n The hubo_state channel contains:
     * \li IMU: angular position and velocity around x, y, z;
     * \li Force/Torque: moment in x and y; force in z;
     * \li Joints: latest encoder reference value, position (radians), current (amps), velocity (rad/s), heat (J), temperature (C), if it's active, if it's zeroed;
     * \li Joint motor controller (JMC) state;
     * \li Current time (s);
     * \li refWait (whether or not to wait before before sending reference commands).
     *
     * \n The hubo_ref channel contains:
     * \li Joint reference (encoder value);
    */
    /// Retrieves the latest data from the state and ref channels
    ctrl_flag_t update(bool stateWait=true, double quitSec = 0.1, bool printError=false);   


    // ~~~*** Sending Control Commands ***~~~ //
    // ~~** Setting reference values

    // ~* General sets
    /**
     * Resets the error flags for the joint.
     * \param joint joint name
     * \param send  Whether or not to send command immediately to the control daemon.
    */
    ctrl_flag_t resetJointStatus( int joint, bool send=false );
    // Position control
    /**
     * Safely switches joint to position control. Using this is encouraged if you
     * want to switch between control modes during runtime. Otherwise it is not necessary.
    */
    ctrl_flag_t setPositionControl( int joint );
    /**
     * Sets the desired correctness metric for a trajectory. The 'correctness' refers
     * to how far the trajectory will deviate from the velocity profile in order to
     * get to the position profile.
    */
    ctrl_flag_t setJointTrajCorrectness( int joint, double correctness );
    /**
     * Sets the desired correctness metric for a trajectory. The 'correctness' refers
     * to how far the trajectory will deviate from the velocity profile in order to
     * get to the position profile.
    */
    void setAllTrajCorrectness(double correctness );
    /**
     * Sets the expected frequency for a joint's trajectory.
    */
    ctrl_flag_t setJointTrajFrequency( int joint, double frequency );
    /**
      * Sets the expected frequency for all trajectories.
     **/
    void setAllTrajFrequency( double frequency );
    /**
     * Sets joint-space trajectory values for a joint (position, velocity)
    */
    ctrl_flag_t setJointTraj(int joint, double radians, double vel, bool send=false );
    /**
     * Sets a joint-space trajectory waypoint for all the arms angles on the specified
     * side.
     */
    ctrl_flag_t setArmTraj(int side, ArmVector angles, ArmVector vels, bool send=false);
    /**
     * Sets a joint-space trajectory waypoint for all angles on the Left Arm.
     */
    ctrl_flag_t setLeftArmTraj(ArmVector angles, ArmVector vels, bool send=false);
    /**
     * Sets a joint-space trajectory waypoint for all angles on the Right Arm.
     */
    ctrl_flag_t setRightArmTraj(ArmVector angles, ArmVector vels, bool send=false);
    /**
     * Sets a joint-space trajectory waypoint for all the leg angles on the specified
     * side.
     */
    ctrl_flag_t setLegTraj(int side, ArmVector angles, ArmVector vels, bool send=false);
    /**
     * Sets a joint-space trajectory waypoint for all angles on the Left Leg.
     */
    ctrl_flag_t setLeftLegTraj(LegVector angles, LegVector vels, bool send=false);
    /**
     * Sets a joint-space trajectory waypoint for all angles on the Left Leg.
     */
    ctrl_flag_t setRightLegTraj(LegVector angles, LegVector vels, bool send=false);
    /**
     * Toggles joint-level friction (back-EMF) compensation in the specified
     * joint.
     */
    ctrl_flag_t setJointAntiFriction(int joint, bool on);
    /**
     * Toggles joint-level friction (back-EMF) compensation in the specified
     * arm.
     */
    ctrl_flag_t setArmAntiFriction(int side, bool on);
    /**
     * Toggles joint-space compliance in the specified joint, using the default
     * gains.
     */
    ctrl_flag_t setJointCompliance(int joint, bool on);
    /**
     * Toggles joint-space compliance in the specified joint, and sets the default
     * gains.
     */
    ctrl_flag_t setJointCompliance(int joint, bool on, double Kp, double Kd=0);
    /**
     * Sets the max PWM used by the jointspace compliance mode.
     */
    ctrl_flag_t setJointMaxPWM(int joint, double maxPWM);
    /**
     * Toggles joint-space compliance in the specified arm.
     */
    ctrl_flag_t setArmCompliance(int side, bool on);
    /**
     * Toggles joint-space compliance in the specified arm and sets the default
     * gains.
     */
    ctrl_flag_t setArmCompliance(int side, bool on, ArmVector Kp, ArmVector Kd=ArmVector::Zero());
    /**
     * Sets the feedforward torque command for the specified joint.
     */
    ctrl_flag_t setJointTorque( int joint, double torque );
    /**
     * Turns off torque control for the specified joint
     */
    ctrl_flag_t releaseJointTorque( int joint );
    /**
     * Turns off torque control for the specified arm
     */
    ctrl_flag_t releaseArmTorques( int side );
    /**
     * Sets the feedforward torque commands for the specified arm.
     */
    ctrl_flag_t setArmTorques( int side, ArmVector torques );
    /**
     * Sets the feedforward torque commands for the left arm.
     *
     * NOTE: Torque control in the legs is not currently supported.
     */
    ctrl_flag_t setLeftArmTorques(ArmVector torques);
    /**
     * Sets the feedforward torque commands for the right arm.
     */
    ctrl_flag_t setRightArmTorques(ArmVector torques);
    /**
     * Sets joint angle for the joint to specified radian value. If send is true then 
     * the command will be sent to the motor board immediately, otherwise it won't.
     */
    ctrl_flag_t setJointAngle( int joint, double radians, bool send=false );
    /**
     * Sets the nominal speed for the joint to speed in radians/sec.
     */
    ctrl_flag_t setJointNominalSpeed( int joint, double speed );
    // Velocity control
    /**
     * Safely switches the joint to velocity control. Using this is encouraged if you
     * want to switch between control modes during runtime. Otherwise it is not necessary.
     */
    ctrl_flag_t setVelocityControl( int joint );
    /**
     * Sets the joint velocity in rad/s.
     */
    ctrl_flag_t setJointVelocity( int joint, double vel, bool send=false );
    // Acceleration setting
    /**
     * Sets the nomical acceleration for the joint in rad/s^2.
     */
    ctrl_flag_t setJointNominalAcceleration( int joint, double acc );

    // ~* Arm control sets
    // Position control
    /**
     * Extension of setPositionControl() which acts on all joints in an arm.
     */
    ctrl_flag_t setArmPosCtrl( int side );
    /**
     * Moves the joint angles for all the arm angles of the arm specified by the "side" argument
     * to the values specified by the angles argument. The send argument specifies whether or
     * not to send the commands immediately or not.
     *
     * Example: Say "leftAngles" is a Vector6d containing the six desired values for the left arm,
     * and "rightAngles" is a Vector6d containing the six desired values for the right arm.
     * 
     * setArmAngles( LEFT, leftAngles, true ) will immediately send a request to the control daemon
     * to move the left arm joint configuration from its current values to the values specified in
     * leftAngles.
     *
     * setArmAngles( RIGHT, rightAngles ) will set up a request for the control daemon to move the
     * right arm joint configuration from its current values to the values specified in rightAngles.
     * HOWEVER, that request will not be sent until you run the command sendControls(). This is
     * because "send" defaults to false if it is not specified.
     */
    ctrl_flag_t setArmAngles( int side, ArmVector angles, bool send=false );
    /**
     * Extension of setArmPosCtrl(int side) where side = LEFT
     */
    void setLeftArmPosCtrl();
    /**
     * Extension of setArmAngles() where side = LEFT
     */
    ctrl_flag_t setLeftArmAngles( ArmVector angles, bool send=false );
    /**
     * Extension of setArmPosCtrl(int side) where side = RIGHT
     */
    void setRightArmPosCtrl();
    /**
     * Extension of setArmAngles() where side = RIGHT
     */
    ctrl_flag_t setRightArmAngles( ArmVector angles, bool send=false );
    /**
     * Extension of setJointNominalSpeed() which sets all joints in an arm according to the values
     * in "speeds".
     */
    ctrl_flag_t setArmNomSpeeds( int side, ArmVector speeds );
    /**
     * Extension of setArmNomSpeeds() where side = LEFT
     */
    ctrl_flag_t setLeftArmNomSpeeds( ArmVector speeds );
    /**
     * Extension of setArmNomSpeeds() where side = RIGHT
     */
    ctrl_flag_t setRightArmNomSpeeds( ArmVector speeds );
    /**
     * Extension of setVelocityControl() which acts on all joint in an arm designated by "side" (LEFT or RIGHT)
     */
    ctrl_flag_t setArmVelCtrl( int side );
    /**
     * Moves the joint angles for all the arm angles of the arm specified by the "side" argument
     * at a rate of radians/sec, according to the six values in the "vels" ArmVector. The "send"
     * argument specifies whether or not to send the commands immediately.
     *
     * NOTE: If the control-daemon does not receive a velocity control command within a certain
     * elapsed time, it will bring the velocity back down to zero. This is to make sure the joints
     * stop if the program requestion their movement crashes.
     *
     * Example: Say "leftVels" is a ArmVector containing the six desired velocities for the left
     * arm joints, and "rightVels" is a ArmVector containing the six desired valocities for the
     * right arm joints
     * 
     * setArmVels( LEFT, leftVels, true ) will immediately send a request to the control-daemon
     * to drive the left arm joints at the velocities specified in "leftVels".
     *
     * setArmVels( RIGHT, rightVels ) will set up a request for the control-daemon to drive
     * right arm joints at the velocities specified in "rightVels".
     * HOWEVER, that request will not be sent until you run the command sendControls(). This is
     * because "send" defaults to false if it is not specified.
     */
    ctrl_flag_t setArmVels( int side, ArmVector vels, bool send=false );
    /**
     * Extension of setArmVelCtrl() where side = LEFT
     */
    void setLeftArmVelCtrl();
    /**
     * Extension of setArmVelCtrl() where side = LEFT
     */
    ctrl_flag_t setLeftArmVels( ArmVector vels, bool send=false );
    /**
     * Extension of setArmVelCtrl() where side = RIGHT
     */
    void setRightArmVelCtrl();
    /**
     * Extension of setArmVelCtrl() where side = RIGHT
     */
    ctrl_flag_t setRightArmVels( ArmVector vels, bool send=false );
    /**
     * Extension of setJointNominalAcceleration() which applies the six values in "acc" to the six
     * joints in the arm corresponding to "side" (LEFT or RIGHT)
     */
    ctrl_flag_t setArmNomAcc(int side, ArmVector acc );
    /**
     * Extension of setArmNomAcc() where side = LEFT
     */
    ctrl_flag_t setLeftArmNomAcc( ArmVector acc );
    /**
     * Extension of setArmNomAcc() where side = RIGHT
     */
    ctrl_flag_t setRightArmNomAcc( ArmVector acc );

    // ~* Leg control sets
    // Position control
    /**
     * Same as setArmPosCtrl() but applied to the leg
    */
    ctrl_flag_t setLegPosCtrl( int side );
    /**
     * Same as setArmAngles() but applied to the leg
    */
    ctrl_flag_t setLegAngles( int side, LegVector angles, bool send=false );
    /**
     * Same as setLeftArmPosCtrl() but applied to the leg
    */
    void setLeftLegPosCtrl();
    /**
     * Same as setLeftArmAngles() but applied to the leg
    */
    ctrl_flag_t setLeftLegAngles( LegVector angles, bool send=false );
    /**
     * Same as setRightArmPosCtrl() but applied to the leg
    */
    void setRightLegPosCtrl();
    /**
     * Same as setRightArmAngles() but applied to the leg
    */
    ctrl_flag_t setRightLegAngles( LegVector angles, bool send=false );
    /**
     * Same as setArmNomSpeeds() but applied to the leg
    */
    ctrl_flag_t setLegNomSpeeds( int side, LegVector speeds );
    /**
     * Same as setLeftArmNomSpeeds() but applied to the leg
    */
    ctrl_flag_t setLeftLegNomSpeeds( LegVector speeds );
    /**
     * Same as setRightArmNomSpeeds() but applied to the leg
    */
    ctrl_flag_t setRightLegNomSpeeds( LegVector speeds );
    /**
     * Same as setArmVelCtrl() but applied to the leg
    */
    ctrl_flag_t setLegVelCtrl( int side );
    /**
     * Same as setArmVels() but applied to the leg
    */
    ctrl_flag_t setLegVels( int side, LegVector vels, bool send=false );
    /**
     * Same as setLeftArmVelCtrl() but applied to the leg
    */
    void setLeftLegVelCtrl();
    /**
     * Same as setLeftArmVels() but applied to the leg
    */
    ctrl_flag_t setLeftLegVels( LegVector vels, bool send=false );
    /**
     * Same as setRightArmVelCtrl() but applied to the leg
    */
    void setRightLegVelCtrl();
    /**
     * Same as setRightArmVels() but applied to the leg
    */
    ctrl_flag_t setRightLegVels( LegVector vels, bool send=false );
    /**
     * Same as setArmNomAcc() but applied to the leg
    */
    ctrl_flag_t setLegNomAcc(int side, LegVector acc );
    /**
     * Same as setLeftArmNomAcc() but applied to the leg
    */
    ctrl_flag_t setLeftLegNomAcc( LegVector acc );
    /**
     * Same as setRightArmNomAcc() but applied to the leg
    */
    ctrl_flag_t setRightLegNomAcc( LegVector acc );

    // ~~** Setting limit values
    // ~* General sets
    /**
     * Sets the lowest angle permitted by the control-daemon for a particular joint.
    */
    ctrl_flag_t setJointAngleMin( int joint, double radians );
    /**
     * Sets the highest angle permitted by the control-daemon for a particular joint.
    */
    ctrl_flag_t setJointAngleMax( int joint, double radians );
    /**
     * Adjusts the error tolerance of the control-daemon. A higher speed will make the 
     * control-daemon increase the error tolerance, and a lower speed will decrease the
     * error tolerance ("error" meaning how far the requested value is from the current
     * state). If this error is ever exceeded, the control-daemon will freeze that joint
     * until a reset command is sent.
    */
    ctrl_flag_t setJointErrorMax( int joint, double speed );

    // ~~** Send Off Latest Control Commands
    /**
     * Sends off all control commands to the control-daemon. Recommended at the end of your
     * control loop.
    */
    void sendControls();


    // ~~** Getting Reference Values
    // ~* General gets
    // Position control
    hubo_ctrl_mode_t getCtrlMode( int joint );
    /**
     * Returns the current \b reference position value of the joint specified by "joint".
     * In general, it is best to make decisions solely on the reference value of a joint,
     * because the state value can be noisy. Also, you should not be performing feedback
     * control with the state of a joint because the motors already perform feedback control,
     * and your feedback controller might clash with theirs, producing violent outcomes.
     *
     * To read the current \b state value, look at getJointAngleState()
    */
    double getJointAngle( int joint );
    /**
     * Returns the current \b command position value of the joint specified by "joint".
     * In other words, this is the latest position control command currently being held
     * in your instance of Hubo_Control. If you run sendControls(), then this is the position
     * control value which will be requested.
     *
     * To read the current \b reference value, look at getJointAngle()
    */
    double getJointAngleCtrl( int joint );
    /**
     * Returns the current nominal speed value of the joint specified by "joint".
    */
    double getJointNominalSpeed( int joint );
    /**
     * Returns the current velocity of the joint specified by "joint"
    */
    double getJointVelocity( int joint );
    /**
     * Returns the current reference velocity of the joint specified by "joint"
    */
    double getJointRefVelocity( int joint );
    // Velocity control
    /**
     * Returns the current velocity command value of the joint specified by "joint".
     *
     * Similar to getJointAngleCtrl() but for velocity control.
    */
    double getJointVelocityCtrl( int joint );
    //double getJointVelocityState( int joint ); // TODO: add velocity to the state
    // Acceleration setting
    /**
     * Returns the current nominal acceleration value of the joint specified by "joint".
    */
    double getJointNominalAcceleration( int joint );
    /**
     * Returns whether the control-daemon has frozen a joint: 0-Active 1-Frozen
    */
    int getJointStatus( int joint ); // 0:Good 1:Frozen

    /**
     * Returns whether or not the joint is homed
    */
    bool isHomed( int joint );

    /**
     * Returns whether or not there are errors on the joint's JMC
    */
    bool errorsExist( int joint);

    // ~* Arm control gets
    // Position control
    /**
     * Extension of getJointAngle() for the arm corresponding to "side" (LEFT or RIGHT).
     *
     * Declare a ArmVector named "angles" and pass it into the second argument. After the function
     * is finished, "angles" will be filled with the joint position reference values of the target
     * arm. This is known as "passing by reference".
     *
     * Example: getArmAngles( RIGHT, angles ) will take a "ArmVector angles" and fill it with the
     * current reference values of the right arm.
    */
    ctrl_flag_t getArmAngles( int side, ArmVector &angles );
    /**
     * Extension of getArmAngles() where side = LEFT
    */
    void getLeftArmAngles( ArmVector &angles );
    /**
     * Extension of getArmAngles() where side = RIGHT
    */
    void getRightArmAngles( ArmVector &angles );
    /**
     * Extension of getJointNominalSpeed()
     *
     * Operates similarly to getArmAngles()
    */
    ctrl_flag_t getArmNomSpeeds( int side, ArmVector &speeds );
    /**
     * Extension of getArmNomSpeeds() where side = LEFT
    */
    void getLeftArmNomSpeeds( ArmVector &speeds );
    /**
     * Extension of getArmNomSpeeds() where side = RIGHT
    */
    void getRightArmNomSpeeds( ArmVector &speeds );
    /**
     * Extension of getJointVelocity() for the arm corresponding to "side" (LEFT or RIGHT).
     *
     * Operates similarly to getArmAngles()
    */
    ctrl_flag_t getArmVels( int side, ArmVector &vels );
    /**
     * Extension of getArmVels() where side = LEFT
    */
    void getLeftArmVels( ArmVector &vels );
    /**
     * Extension of getArmVels() where side = RIGHT
    */
    void getRightArmVels( ArmVector &vels );
    /**
     * Get the velocity of an arm's reference
    */
    ctrl_flag_t getArmRefVels( int side, ArmVector &vels );

    // Velocity control
    /**
     * Extension of getJointVelocityCtrl() for the arm corresponding to "side" (LEFT or RIGHT).
     *
     * Operates similarly to getArmAngles()
    */
    ctrl_flag_t getArmVelCtrls( int side, ArmVector &vels );
    /**
     * Extension of getArmVelCtrls() where side = LEFT
    */
    void getLeftArmVelCtrls( ArmVector &vels );
    /**
     * Extension of getArmVelCtrls() where side = RIGHT
    */
    void getRightArmVelCtrls( ArmVector &vels );
    // Acceleration settings
    /**
     * Extension of getJointNominalAcceleration() for the arm corresponding to "side" (LEFT or RIGHT).
     *
     * Operates similarly to getArmAngles()
    */
    ctrl_flag_t getArmNomAcc(int side, ArmVector &acc );
    /**
     * Extension of getArmNomAcc() where side = LEFT
    */
    void getLeftArmNomAcc( ArmVector &acc );
    /**
     * Extension of getArmNomAcc() where side = RIGHT
    */
    void getRightArmNomAcc( ArmVector &acc );

    // ~* Leg control gets
    // Position control
    /**
     * Similar to getArmAngles() but applied to the leg
    */
    ctrl_flag_t getLegAngles( int side, LegVector &angles );
    /**
     * Similar to getLeftArmAngles() but applied to the leg
    */
    void getLeftLegAngles( LegVector &angles );
    /**
     * Similar to getRightArmAngles() but applied to the leg
    */
    void getRightLegAngles( LegVector &angles );
    /**
     * Similar to getArmNomSpeeds() but applied to the leg
    */
    ctrl_flag_t getLegNomSpeeds( int side, LegVector &speeds );
    /**
     * Similar to getLeftArmNomSpeeds() but applied to the leg
    */
    void getLeftLegNomSpeeds( LegVector &speeds );
    /**
     * Similar to getRightArmNomSpeeds() but applied to the leg
    */
    void getRightLegNomSpeeds( LegVector &speeds );
    /**
     * Similar to getArmVels() but applied to the leg
    */
    ctrl_flag_t getLegVels( int side, LegVector &vels );
    /**
     * Similar to getLeftArmVels() but applied to the leg
    */
    void getLeftLegVels( LegVector &vels );
    /**
     * Similar to getRightArmVels() but applied to the leg
    */
    void getRightLegVels( LegVector &vels );
    // Velocity control
    /**
     * Similar to getArmVelCtrls() but applied to the leg
    */
    ctrl_flag_t getLegVelCtrls( int side, LegVector &vels );
    /**
     * Similar to getLeftArmVelCtrls() but applied to the leg
    */
    void getLeftLegVelCtrls( LegVector &vels );
    /**
     * Similar to getRightArmVelCtrls() but applied to the leg
    */
    void getRightLegVelCtrls( LegVector &vels );
    // Acceleration settings
    /**
     * Similar to getArmNomAcc() but applied to the leg
    */
    ctrl_flag_t getLegNomAcc(int side, LegVector &acc );
    /**
     * Similar to getLeftArmNomAcc() but applied to the leg
    */
    void getLeftLegNomAcc( LegVector &acc );
    /**
     * Similar to getRightArmNomAcc() but applied to the leg
    */
    void getRightLegNomAcc( LegVector &acc );

    // ~~** Getting limit values
    // ~* General gets
    /**
     * Returns the control-daemon's lower joint angle limit for a particular joint
    */
    double getJointAngleMin( int joint );
    /**
     * Returns the control-daemon's upper joint angle limit for a particular joint
    */
    double getJointAngleMax( int joint );
    /**
     * Returns the control-daemon's joint speed limit for a particular joint
    */
    double getJointErrorMax( int joint );

    // Passthrough Mode: Reference values will not be touched by
    // the control-daemon. Use of this mode is strongly discouraged.
    /**
     * Instructs the control-daemon to send the position reference value given by "radians"
     * straight through to the motor boards without filtering or controlling it.
     * 
     * \b NOTE: Use of this mode is \em strongly discouraged.
    */
    ctrl_flag_t passJointAngle( int joint, double radians, bool send=false );
    /**
      * Instructs the control-daemon to send the given PWM command straight through
      * to the motor board without any kind of conversion or smoothing.
      *
      * \b NOTE: Use of this mode is \em strongly discouraged.
    */
    ctrl_flag_t setJointPWM( int joint, double pwm , bool send=false );

    // ~~~*** State Readings ***~~~ //

    // ~~** State
    /**
     * Returns the current \b state position value of the joint specified by "joint".
     * In general, it is best to make decisions solely on the \b reference value of a joint,
     * because the state value can be noisy. Also, you should not be performing feedback
     * control with the state of a joint because the motors already perform feedback control,
     * and your feedback controller might clash with theirs, producing violent outcomes.
     * 
     * When the a joint is not moving, the state and reference value should be practically
     * identical, plus or minus some noise.
     *
     * To read the current \b reference value, look at getJointAngle()
    */
    double getJointAngleState( int joint );
    /**
     * Extension of getJointAngleState() for the arm corresponding to "side" (LEFT or RIGHT)
     * 
     * Operates similarly to getArmAngles()
    */
    ctrl_flag_t getArmAngleStates( int side, ArmVector &angles );
    /**
     * Extension of getJointAngleState() where side = RIGHT
    */
    void getRightArmAngleStates( ArmVector &angles );
    /**
     * Extension of getJointAngleState() where side = LEFT
    */
    void getLeftArmAngleStates( ArmVector &angles );
    /**
     * Similar to getArmAngleStates() but applied to the leg
    */
    ctrl_flag_t getLegAngleStates( int side, LegVector &angles );
    /**
     * Similar to getRightArmAngleStates() but applied to the leg
    */
    void getRightLegAngleStates( LegVector &angles );
    /**
     * Similar to getLeftArmAngleStates() but applied to the leg
    */
    void getLeftLegAngleStates( LegVector &angles );
    /**
      * Returns the PWM % Duty command that is being sent to the motor for the specified joint
    */
    double getJointDuty(int joint);
    // TODO: All of these (state position, velocity, whatever)

    // ~~** Sensors
    // ~* Force-torque
    // Mx
    /**
     * Returns the the FT-sensor reading of moment around the x-axis for "sensor"
     *
     * "sensor" can take on the values:
     * \li HUBO_FT_R_HAND
     * \li HUBO_FT_L_HAND
     * \li HUBO_FT_R_FOOT
     * \li HUBO_FT_L_FOOT
    */
    double getMx( hubo_sensor_index_t sensor );
    /**
     * Extension of getMx() where sensor = HUBO_FT_R_HAND
    */
    double getRightHandMx();
    /**
     * Extension of getMx() where sensor = HUBO_FT_L_HAND
    */
    double getLeftHandMx();
    /**
     * Extension of getMx() where sensor = HUBO_FT_R_FOOT
    */
    double getRightFootMx();
    /**
     * Extension of getMx() where sensor = HUBO_FT_L_FOOT
    */
    double getLeftFootMx();
    // My
    /**
     * Similar to getMx() but about the y-axis
    */
    double getMy( hubo_sensor_index_t sensor );
    /**
     * Extension of getMy() where sensor = HUBO_FT_R_HAND
    */
    double getRightHandMy();
    /**
     * Extension of getMy() where sensor = HUBO_FT_L_HAND
    */
    double getLeftHandMy();
    /**
     * Extension of getMy() where sensor = HUBO_FT_R_FOOT
    */
    double getRightFootMy();
    /**
     * Extension of getMy() where sensor = HUBO_FT_L_FOOT
    */
    double getLeftFootMy();
    // Fz
    /**
     * Returns the FT-sensor reading of force along the z-axis for "sensor"
     *
     * "sensor" can take on the values:
     * \li HUBO_FT_R_HAND
     * \li HUBO_FT_L_HAND
     * \li HUBO_FT_R_FOOT
     * \li HUBO_FT_L_FOOT
    */
    double getFz( hubo_sensor_index_t sensor );
    /**
     * Extension of getFz() where sensor = HUBO_FT_R_HAND
    */
    double getRightHandFz();
    /**
     * Extension of getFz() where sensor = HUBO_FT_L_HAND
    */
    double getLeftHandFz();
    /**
     * Extension of getFz() where sensor = HUBO_FT_R_FOOT
    */
    double getRightFootFz();
    /**
     * Extension of getFz() where sensor = HUBO_FT_L_FOOT
    */
    double getLeftFootFz();
    // ~* Accelerometers
    // Tilt X
    /**
     * Returns the tilt-sensor reading of the x-axis for the foot corresponding to "side" (LEFT or RIGHT)
    */
    double getTiltX( int side );
    /**
     * Extension of getTiltX() where side = LEFT
    */
    double getLeftTiltX();
    /**
     * Extension of getTiltX() where side = RIGHT
    */
    double getRightTiltX();
    // Tilt Y
    /**
     * Returns the tilt-sensor reading of the y-axis for the foot corresponding to "side" (LEFT or RIGHT)
    */
    double getTiltY( int side );
    /**
     * Extension of getTiltY() where side = LEFT
    */
    double getLeftTiltY();
    /**
     * Extension of getTiltY() where side = RIGHT
    */
    double getRightTiltY();
    // Tilt Z
    /**
     * Returns the tilt-sensor reading of the y-axis for the foot corresponding to "side" (LEFT or RIGHT)
    */
    double getTiltZ( int side );
    /**
     * Extension of getTiltZ() where side = LEFT
    */
    double getLeftTiltZ();
    /**
     * Extension of getTiltZ() where side = RIGHT
    */
    double getRightTiltZ();
    // ~* IMU
    /**
     * Returns the IMU-sensor reading of the rotation around the x-axis
    */
    double getAngleX();
    /**
     * Returns the IMU-sensor reading of the rotation around the y-axis
    */
    double getAngleY();
    /**
     * Returns the IMU-sensor reading of the rotational velocity around the x-axis
    */
    double getRotVelX();
    /**
     * Returns the IMU-sensor reading of the rotational velocity around the y-axis
    */
    double getRotVelY();


    // ~~~*** Kinematics ***~~~ //
    inline double wrapToPi(double fAng)
    {
        return mod(fAng + M_PI, 2*M_PI) - M_PI;
    }

    inline double mod(double x, double y)
    {
        if (0 == y)
            return x;
        
        return x - y * floor(x/y);
    }
    
    void DH2HG(Eigen::Isometry3d &B, double t, double f, double r, double d);
    
    void huboArmFK(Eigen::Isometry3d &B, ArmVector &q, int side);
    /**
     * The first argument (B) is a homogeneous transformation matrix which gets filled in with the
     * end effector transformation produced by the the joint configuration of the second argument
     * (q) for the arm specified by the third argument (side = LEFT or RIGHT)
    */
    void huboArmFK(Eigen::Isometry3d &B, Vector6d &q, int side);
    
    void huboArmFK(Eigen::Isometry3d &B, ArmVector &q, int side, const Eigen::Isometry3d &endEffector);
    /**
     * Same as huboArmFK(Eigen::Isometry3d &B, Vector6d &q, int side), but it will also apply a
     * final end effector transformation at the end, specified by the fourth argument.
     *
     * This is useful if a different tool has been attached to a hand, thereby producing a
     * different end-effector offset/orientation than usual.
    */
    void huboArmFK(Eigen::Isometry3d &B, Vector6d &q, int side, const Eigen::Isometry3d &endEffector);
    
    bool huboArmIK(ArmVector &q, const Eigen::Isometry3d B, ArmVector qPrev, int side);
    /**
     * Performs an Analytical Inverse Kinematics calculation for the arm.
     *
     * q is the joint angle configuration which the IK will fill in. It is "passed in by reference".
     *
     * B is the desired end effector transformation.
     *
     * qPrev is a joint configuration which might be used to compare the result. Generally you will
     * want to pass in the current joint configuration. Technically, however, this is not presently
     * being used by the IK, and it might be removed in the future.
     *
     * side specifies which arm (LEFT or RIGHT)
    */
    bool huboArmIK(Vector6d &q, const Eigen::Isometry3d B, Vector6d qPrev, int side);
    
    bool huboArmIK(ArmVector &q, const Eigen::Isometry3d B, ArmVector qPrev, int side, const Eigen::Isometry3d &endEffector);
    /**
     * Same as huboArmIK(Vector6d &q, Eigen::Isometry3d B, Vector6d qPrev, int side), but it will
     * also apply a final end effector transformation at the end, specified by the fifth argument.
     *
     * This is useful if a different tool has been attached to a hand, thereby producing a
     * different end-effector offset/orientation than usual.
     * @Return: returns true if the goal was outside the feasible workspace, otherwise false
    */
    bool huboArmIK(Vector6d &q, const Eigen::Isometry3d B, Vector6d qPrev, int side, const Eigen::Isometry3d &endEffector);
    
    void huboLegFK(Eigen::Isometry3d &B, LegVector &q, int side);
    /**
     * Similar to huboArmFK() but applied to the leg
     * @Return: returns true if the goal was outside the feasible workspace, otherwise false
    */
    void huboLegFK(Eigen::Isometry3d &B, Vector6d &q, int side);
    bool huboLegIK(LegVector &q, const Eigen::Isometry3d B, LegVector qPrev, int side);
    /**
     * Similar to huboArmIK() but applied to the leg
     * @Return: returns true if the goal was outside the feasible workspace, otherwise false
    */
    bool huboLegIK(Vector6d &q, const Eigen::Isometry3d B, Vector6d qPrev, int side);

    /**
     * A specialized differential IK (solved analytically) for moving the hip
     * position. NOT THOROUGHLY TESTED -- USE WITH EXTREME CAUTION.
    */
    ctrl_flag_t hipVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity, int side );
    ctrl_flag_t hipVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity, const LegVector &q ); 
    ctrl_flag_t hipVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity, Eigen::Vector3d &angularVel, int side );
    ctrl_flag_t hipVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity, Eigen::Vector3d &angularVel, const LegVector &q );
    ctrl_flag_t footVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity, int side );
    ctrl_flag_t footVelocityIK( LegVector &qdot, Eigen::Vector3d &velocity, Eigen::Vector3d &angularVel, int side );
    /**
     * Calibration for arbitrary joints
    */
    ctrl_flag_t calibrateJoint( int joint, double offset=0.0 );
    /**
     * Calibration to get consistent force readings between the ankle FT sensors
    */
    void calibrateAnkleForces();
    /**
     * A specialized Forward Kinematics calculation which accounts for the end effector
     * transformation of our experimental drill.
    */
    void HuboDrillFK(Eigen::Isometry3d &B, ArmVector &q);
    /**
     * A specialized Inverse Kinematics Calculation which constrains the end effector to a
     * particular height and forward displacement which maximizes the length of its feasible
     * traversal along the y-axis. Useful for 1-DOF horizontal control of the end effector.
     *
     * Only applies to the right arm.
     *
     * q is the joint angle configuration which gets filled in. y is the desired horizontal
     * location.
    */
    void HuboDrillIK(ArmVector &q, double y);
    
    void storeArmDefaults(int side);
    void storeRightArmDefaults();
    void storeLeftArmDefaults();
    
    void storeLegDefaults(int side);
    void storeRightLegDefaults();
    void storeLeftLegDefaults();
    
    void storeBodyDefaults();
    void storeNeckDefaults();

    void storeAllDefaults();
    
    void resetArmDefaults(int side, bool send=false);
    void resetRightArmDefaults(bool send=false);
    void resetLeftArmDefaults(bool send=false);
    
    void resetLegDefaults(int side, bool send=false);
    void resetRightLegDefaults(bool send=false);
    void resetLeftLegDefaults(bool send=false);
    
    void resetBodyDefaults(bool send=false);
    void resetNeckDefaults(bool send=false);

    void resetAllDefaults(bool send=false);
    
    void releaseArm(int side);
    void releaseRightArm();
    void releaseLeftArm();
    
    void releaseLeg(int side);
    void releaseRightLeg();
    void releaseLeftLeg();
    
    void releaseFingers(int side);
    void releaseRightFingers();
    void releaseLeftFingers();
    
    void releaseBody();
    void releaseNeck();


protected:

    void controlInit(bool live);

    ach_channel_t chan_hubo_ref;
    ach_channel_t chan_hubo_board_cmd;
    ach_channel_t chan_hubo_state;
    ach_channel_t chan_ctrl_state;
    ach_channel_t chan_hubo_arm_ctrl_right;
    ach_channel_t chan_hubo_arm_ctrl_left;
    ach_channel_t chan_hubo_leg_ctrl_right;
    ach_channel_t chan_hubo_leg_ctrl_left;
    ach_channel_t chan_hubo_fin_ctrl_right;
    ach_channel_t chan_hubo_fin_ctrl_left;
    ach_channel_t chan_hubo_bod_ctrl;
    ach_channel_t chan_hubo_nck_ctrl;

    hubo_ref_t H_Ref;
    hubo_board_cmd_t H_Cmd;
    hubo_state_t H_State;
    hubo_ctrl_state_t C_State;
    hubo_arm_control_t H_Arm_Ctrl[2];
    hubo_leg_control_t H_Leg_Ctrl[2];
    hubo_fin_control_t H_Fin_Ctrl[2];
    hubo_bod_control_t H_Bod_Ctrl;
    hubo_nck_control_t H_Nck_Ctrl;
    
    hubo_arm_control_t H_Arm_Ctrl_Defaults[2];
    hubo_leg_control_t H_Leg_Ctrl_Defaults[2];
    hubo_fin_control_t H_Fin_Ctrl_Defaults[2];
    hubo_bod_control_t H_Bod_Ctrl_Defaults;
    hubo_nck_control_t H_Nck_Ctrl_Defaults;
    

    int armjoints[2][ARM_JOINT_COUNT];
    int legjoints[2][LEG_JOINT_COUNT];
    int finjoints[2][FIN_JOINT_COUNT];
    int bodjoints[BOD_JOINT_COUNT];
    int nckjoints[NCK_JOINT_COUNT];

    bool ctrlOn[8];
    // 0) Right Arm
    // 1) Left Arm
    // 2) Right Leg
    // 3) Left Leg
    // 4) Right Fingers
    // 5) Left Fingers
    // 6) Body
    // 7) Neck



    double kneeSingularityThreshold;
    double kneeSingularityDanger;
    double kneeSingularitySpeed;

    double apc[2]; // Ankle Pitch Calibration
    double arc[2]; // Ankle Roll Calibration
    double jointAngleCalibration[HUBO_JOINT_COUNT];
    double afc[2]; // Ankle Force Calibration

    double shinLength;
    bool outOfWorkspace; ///< Goal pose is outside workspace

private:
    
    int ctrlMap[HUBO_JOINT_COUNT];
    int localMap[HUBO_JOINT_COUNT];
};

#endif // HUBO_CONTROL_H
