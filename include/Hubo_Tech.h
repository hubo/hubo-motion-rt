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
 * \file Hubo_Tech.h
 * \brief Programming interface for passing joint control and CAN commands to Hubo daemons.
 * 
 * \author MX Grey
 */

#ifndef HUBO_PLUS_H
#define HUBO_PLUS_H

// C Headers
extern "C" {
// For Hubo
#include "hubo.h"
#include "control-daemon.h"
#include "hubo-jointparams.h"

// For process management
#include "daemonizer.h"
}
#include <iostream>


// For data handling
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <complex.h>

typedef Eigen::Matrix< double, 6, 1 > Vector6d;
typedef Eigen::Vector3d Vector3d;

#define CtrlNE  0
#define CtrlRA  1
#define CtrlLA  2
#define CtrlRL  3
#define CtrlLL  4
#define CtrlRF  5
#define CtrlLF  6
#define CtrlAX  7


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

} tech_flag_t;




class Hubo_Tech
{
public:
    /**
     * Constructor for the Hubo_Tech class.
    */
    Hubo_Tech(); /// Use daemonize(const char *daemon_name) after calling this constructor
    /**
     * Constructor for the Hubo_Tech class that daemonizes with the name specificied by the parameter.
    */
    Hubo_Tech(const char *daemon_name, int priority=-1);
    /**
     * Gets the current time in seconds from Hubo's 'hubo_state' struct member 'time' over the state ach channel
    */
    ~Hubo_Tech();
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
    tech_flag_t update(bool printError=false);   
                    // Returns true if successful
                    // Returns false if both channels are not ACH_OK
    // TODO: Consider making the update return more meaningful


    // ~~~*** Sending Control Commands ***~~~ //
    // ~~** Setting reference values

    // ~* General sets
    /**
     * Resets the error flags for the joint.
     * \param joint joint name
     * \param send  Whether or not to send command immediately to the control daemon.
    */
    tech_flag_t resetJointStatus( int joint, bool send=false );
    // Position control
    /**
     * Safely switches joint to position control. Using this is encouraged if you
     * want to switch between control modes during runtime. Otherwise it is not necessary.
    */
    tech_flag_t setPositionControl( int joint );
    /**
     * Sets joint angle for the joint to specified radian value. If send is true then 
     * the command will be sent to the motor board immediately, otherwise it won't.
    */
    tech_flag_t setJointAngle( int joint, double radians, bool send=false );
    /**
     * Sets the nominal speed for the joint to speed in radians/sec.
    */
    tech_flag_t setJointNominalSpeed( int joint, double speed );
    // Velocity control
    /**
     * Safely switches the joint to velocity control. Using this is encouraged if you
     * want to switch between control modes during runtime. Otherwise it is not necessary.
    */
    tech_flag_t setVelocityControl( int joint );
    /**
     * Sets the joint velocity in rad/s.
    */
    tech_flag_t setJointVelocity( int joint, double vel, bool send=false );
    // Acceleration setting
    /**
     * Sets the nomical acceleration for the joint in rad/s^2.
    */
    tech_flag_t setJointNominalAcceleration( int joint, double acc );

    // ~* Arm control sets
    // Position control
    /**
     * Extension of setPositionControl() which acts on all joints in an arm.
    */
    tech_flag_t setArmPosCtrl( int side );
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
    tech_flag_t setArmAngles( int side, Vector6d angles, bool send=false );
    /**
     * Extension of setArmPosCtrl(int side) where side = LEFT
    */
    void setLeftArmPosCtrl();
    /**
     * Extension of setArmAngles() where side = LEFT
    */
    tech_flag_t setLeftArmAngles( Vector6d angles, bool send=false );
    /**
     * Extension of setArmPosCtrl(int side) where side = RIGHT
    */
    void setRightArmPosCtrl();
    /**
     * Extension of setArmAngles() where side = RIGHT
    */
    tech_flag_t setRightArmAngles( Vector6d angles, bool send=false );
    /**
     * Extension of setJointNominalSpeed() which sets all joints in an arm according to the values
     * in "speeds".
    */
    tech_flag_t setArmNomSpeeds( int side, Vector6d speeds );
    /**
     * Extension of setArmNomSpeeds() where side = LEFT
    */
    tech_flag_t setLeftArmNomSpeeds( Vector6d speeds );
    /**
     * Extension of setArmNomSpeeds() where side = RIGHT
    */
    tech_flag_t setRightArmNomSpeeds( Vector6d speeds );
    /**
    /*
     * Extension of setVelocityControl() which acts on all joint in an arm designated by "side" (LEFT or RIGHT)
    */
    tech_flag_t setArmVelCtrl( int side );
    /**
     * Moves the joint angles for all the arm angles of the arm specified by the "side" argument
     * at a rate of radians/sec, according to the six values in the "vels" Vector6d. The "send"
     * argument specifies whether or not to send the commands immediately.
     *
     * NOTE: If the control-daemon does not receive a velocity control command within a certain
     * elapsed time, it will bring the velocity back down to zero. This is to make sure the joints
     * stop if the program requestion their movement crashes.
     *
     * Example: Say "leftVels" is a Vector6d containing the six desired velocities for the left
     * arm joints, and "rightVels" is a Vector6d containing the six desired valocities for the
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
    tech_flag_t setArmVels( int side, Vector6d vels, bool send=false );
    /**
     * Extension of setArmVelCtrl() where side = LEFT
    */
    void setLeftArmVelCtrl();
    /**
     * Extension of setArmVelCtrl() where side = LEFT
    */
    tech_flag_t setLeftArmVels( Vector6d vels, bool send=false );
    /**
     * Extension of setArmVelCtrl() where side = RIGHT
    */
    void setRightArmVelCtrl();
    /**
     * Extension of setArmVelCtrl() where side = RIGHT
    */
    tech_flag_t setRightArmVels( Vector6d vels, bool send=false );
    /**
     * Extension of setJointNominalAcceleration() which applies the six values in "acc" to the six
     * joints in the arm corresponding to "side" (LEFT or RIGHT)
    */
    tech_flag_t setArmNomAcc(int side, Vector6d acc );
    /**
     * Extension of setArmNomAcc() where side = LEFT
    */
    tech_flag_t setLeftArmNomAcc( Vector6d acc );
    /**
     * Extension of setArmNomAcc() where side = RIGHT
    */
    tech_flag_t setRightArmNomAcc( Vector6d acc );

    // ~* Leg control sets
    // Position control
    /**
     * Same as setArmPosCtrl() but applied to the leg
    */
    tech_flag_t setLegPosCtrl( int side );
    /**
     * Same as setArmAngles() but applied to the leg
    */
    tech_flag_t setLegAngles( int side, Vector6d angles, bool send=false );
    /**
     * Same as setLeftArmPosCtrl() but applied to the leg
    */
    void setLeftLegPosCtrl();
    /**
     * Same as setLeftArmAngles() but applied to the leg
    */
    tech_flag_t setLeftLegAngles( Vector6d angles, bool send=false );
    /**
     * Same as setRightArmPosCtrl() but applied to the leg
    */
    void setRightLegPosCtrl();
    /**
     * Same as setRightArmAngles() but applied to the leg
    */
    tech_flag_t setRightLegAngles( Vector6d angles, bool send=false );
    /**
     * Same as setArmNomSpeeds() but applied to the leg
    */
    tech_flag_t setLegNomSpeeds( int side, Vector6d speeds );
    /**
     * Same as setLeftArmNomSpeeds() but applied to the leg
    */
    tech_flag_t setLeftLegNomSpeeds( Vector6d speeds );
    /**
     * Same as setRightArmNomSpeeds() but applied to the leg
    */
    tech_flag_t setRightLegNomSpeeds( Vector6d speeds );
    /**
     * Same as setArmVelCtrl() but applied to the leg
    */
    tech_flag_t setLegVelCtrl( int side );
    /**
     * Same as setArmVels() but applied to the leg
    */
    tech_flag_t setLegVels( int side, Vector6d vels, bool send=false );
    /**
     * Same as setLeftArmVelCtrl() but applied to the leg
    */
    void setLeftLegVelCtrl();
    /**
     * Same as setLeftArmVels() but applied to the leg
    */
    tech_flag_t setLeftLegVels( Vector6d vels, bool send=false );
    /**
     * Same as setRightArmVelCtrl() but applied to the leg
    */
    void setRightLegVelCtrl();
    /**
     * Same as setRightArmVels() but applied to the leg
    */
    tech_flag_t setRightLegVels( Vector6d vels, bool send=false );
    /**
     * Same as setArmNomAcc() but applied to the leg
    */
    tech_flag_t setLegNomAcc(int side, Vector6d acc );
    /**
     * Same as setLeftArmNomAcc() but applied to the leg
    */
    tech_flag_t setLeftLegNomAcc( Vector6d acc );
    /**
     * Same as setRightArmNomAcc() but applied to the leg
    */
    tech_flag_t setRightLegNomAcc( Vector6d acc );

    // ~~** Setting limit values
    // ~* General sets
    /**
     * Sets the lowest angle permitted by the control-daemon for a particular joint.
    */
    tech_flag_t setJointAngleMin( int joint, double radians );
    /**
     * Sets the highest angle permitted by the control-daemon for a particular joint.
    */
    tech_flag_t setJointAngleMax( int joint, double radians );
    /**
     * Adjusts the error tolerance of the control-daemon. A higher speed will make the 
     * control-daemon increase the error tolerance, and a lower speed will decrease the
     * error tolerance ("error" meaning how far the requested value is from the current
     * state). If this error is ever exceeded, the control-daemon will freeze that joint
     * until a reset command is sent.
    */
    tech_flag_t setJointSpeedMax( int joint, double speed );

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
     * in your instance of Hubo_Tech. If you run sendControls(), then this is the position
     * control value which will be requested.
     *
     * To read the current \b reference value, look at getJointAngle()
    */
    double getJointAngleCtrl( int joint );
    /**
     * Returns the current nominal speed value of the joint specified by "joint".
    */
    double getJointNominalSpeed( int joint );
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

    // ~* Arm control gets
    // Position control
    /**
     * Extension of getJointAngle() for the arm corresponding to "side" (LEFT or RIGHT).
     *
     * Declare a Vector6d named "angles" and pass it into the second argument. After the function
     * is finished, "angles" will be filled with the joint position reference values of the target
     * arm. This is known as "passing by reference".
     *
     * Example: getArmAngles( RIGHT, angles ) will take a "Vector6d angles" and fill it with the
     * current reference values of the right arm.
    */
    tech_flag_t getArmAngles( int side, Vector6d &angles );
    /**
     * Extension of getArmAngles() where side = LEFT
    */
    void getLeftArmAngles( Vector6d &angles );
    /**
     * Extension of getArmAngles() where side = RIGHT
    */
    void getRightArmAngles( Vector6d &angles );
    /**
     * Extension of getJointNominalSpeed()
     *
     * Operates similarly to getArmAngles()
    */
    tech_flag_t getArmNomSpeeds( int side, Vector6d &speeds );
    /**
     * Extension of getArmNomSpeeds() where side = LEFT
    */
    void getLeftArmNomSpeeds( Vector6d &speeds );
    /**
     * Extension of getArmNomSpeeds() where side = RIGHT
    */
    void getRightArmNomSpeeds( Vector6d &speeds );
    // Velocity control
    /**
     * Extension of getJointVelocityCtrl() for the arm corresponding to "side" (LEFT or RIGHT).
     *
     * Operates similarly to getArmAngles()
    */
    tech_flag_t getArmVelCtrls( int side, Vector6d &vels );
    /**
     * Extension of getArmVelCtrls() where side = LEFT
    */
    void getLeftArmVelCtrls( Vector6d &vels );
    /**
     * Extension of getArmVelCtrls() where side = RIGHT
    */
    void getRightArmVelCtrls( Vector6d &vels );
    // Acceleration settings
    /**
     * Extension of getJointNominalAcceleration() for the arm corresponding to "side" (LEFT or RIGHT).
     *
     * Operates similarly to getArmAngles()
    */
    tech_flag_t getArmNomAcc(int side, Vector6d &acc );
    /**
     * Extension of getArmNomAcc() where side = LEFT
    */
    void getLeftArmNomAcc( Vector6d &acc );
    /**
     * Extension of getArmNomAcc() where side = RIGHT
    */
    void getRightArmNomAcc( Vector6d &acc );

    // ~* Leg control gets
    // Position control
    /**
     * Similar to getArmAngles() but applied to the leg
    */
    tech_flag_t getLegAngles( int side, Vector6d &angles );
    /**
     * Similar to getLeftArmAngles() but applied to the leg
    */
    void getLeftLegAngles( Vector6d &angles );
    /**
     * Similar to getRightArmAngles() but applied to the leg
    */
    void getRightLegAngles( Vector6d &angles );
    /**
     * Similar to getArmNomSpeeds() but applied to the leg
    */
    tech_flag_t getLegNomSpeeds( int side, Vector6d &speeds );
    /**
     * Similar to getLeftArmNomSpeeds() but applied to the leg
    */
    void getLeftLegNomSpeeds( Vector6d &speeds );
    /**
     * Similar to getRightArmNomSpeeds() but applied to the leg
    */
    void getRightLegNomSpeeds( Vector6d &speeds );
    // Velocity control
    /**
     * Similar to getArmVelCtrls() but applied to the leg
    */
    tech_flag_t getLegVelCtrls( int side, Vector6d &vels );
    /**
     * Similar to getLeftArmVelCtrls() but applied to the leg
    */
    void getLeftLegVelCtrls( Vector6d &vels );
    /**
     * Similar to getRightArmVelCtrls() but applied to the leg
    */
    void getRightLegVelCtrls( Vector6d &vels );
    // Acceleration settings
    /**
     * Similar to getArmNomAcc() but applied to the leg
    */
    tech_flag_t getLegNomAcc(int side, Vector6d &acc );
    /**
     * Similar to getLeftArmNomAcc() but applied to the leg
    */
    void getLeftLegNomAcc( Vector6d &acc );
    /**
     * Similar to getRightArmNomAcc() but applied to the leg
    */
    void getRightLegNomAcc( Vector6d &acc );

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
    double getJointSpeedMax( int joint );

    // Passthrough Mode: Reference values will not be touched by
    // the control-daemon. Use of this mode is strongly discouraged.
    /**
     * Instructs the control-daemon to send the position reference value given by "radians"
     * straight through to the motor boards without filtering or controlling it.
     * 
     * \b NOTE: Use of this mode is \em strongly discouraged.
    */
    tech_flag_t passJointAngle( int joint, double radians, bool send=false );

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
    tech_flag_t getArmAngleStates( int side, Vector6d &angles );
    /**
     * Extension of getJointAngleState() where side = RIGHT
    */
    void getRightArmAngleStates( Vector6d &angles );
    /**
     * Extension of getJointAngleState() where side = LEFT
    */
    void getLeftArmAngleStates( Vector6d &angles );
    /**
     * Similar to getArmAngleStates() but applied to the leg
    */
    tech_flag_t getLegAngleStates( int side, Vector6d &angles );
    /**
     * Similar to getRightArmAngleStates() but applied to the leg
    */
    void getRightLegAngleStates( Vector6d &angles );
    /**
     * Similar to getLeftArmAngleStates() but applied to the leg
    */
    void getLeftLegAngleStates( Vector6d &angles );
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
     * \li HUBO_FT_R_FOOT
     * \li HUBO_FT_L_FOOT
    */
    double getFz( hubo_sensor_index_t sensor );
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


    // ~~~*** Board Commands ***~~~ //
    // TODO: All of these
    void sendCommands();

    tech_flag_t homeJoint( int joint, bool send=true );
    void homeAllJoints( bool send=true );
    tech_flag_t jointBeep( int joint, double elapseTime, bool send=true );
    tech_flag_t resetJoint( int joint, bool send=true );
    tech_flag_t startSensor( hubo_sensor_index_t sensor, bool send=true );
    tech_flag_t zeroTilt( hubo_sensor_index_t sensor, bool send=true );
    void startAllSensors( bool send=true );
    tech_flag_t initializeBoard( int joint, bool send=true );
    void initializeAll( bool send=true );
    tech_flag_t motorCtrlOn( int joint, bool send=true );
    tech_flag_t motorCtrlOff( int joint, bool send=true );
    tech_flag_t motorCtrlSwitch( int joint, bool on, bool send=true );
    tech_flag_t fetOn( int joint, bool send=true );
    tech_flag_t fetOff( int joint, bool send=true );
    tech_flag_t fetSwitch( int joint, bool on, bool send=true );

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
    
    /**
     * The first argument (B) is a homogeneous transformation matrix which gets filled in with the
     * end effector transformation produced by the the joint configuration of the second argument
     * (q) for the arm specified by the third argument (side = LEFT or RIGHT)
    */
    void huboArmFK(Eigen::Isometry3d &B, Vector6d &q, int side);
    /**
     * Same as huboArmFK(Eigen::Isometry3d &B, Vector6d &q, int side), but it will also apply a
     * final end effector transformation at the end, specified by the fourth argument.
     *
     * This is useful if a different tool has been attached to a hand, thereby producing a
     * different end-effector offset/orientation than usual.
    */
    void huboArmFK(Eigen::Isometry3d &B, Vector6d &q, int side, const Eigen::Isometry3d &endEffector);
        
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
    void huboArmIK(Vector6d &q, const Eigen::Isometry3d B, Vector6d qPrev, int side);
    /**
     * Same as huboArmIK(Vector6d &q, Eigen::Isometry3d B, Vector6d qPrev, int side), but it will
     * also apply a final end effector transformation at the end, specified by the fifth argument.
     *
     * This is useful if a different tool has been attached to a hand, thereby producing a
     * different end-effector offset/orientation than usual.
    */
    void huboArmIK(Vector6d &q, Eigen::Isometry3d B, Vector6d qPrev, int side, const Eigen::Isometry3d &endEffector);
    /**
     * \b NOTE: This is \em not thoroughly tested. Use with extreme caution.
     *
     * Similar to huboArmFK() but applied to the leg
    */
    void huboLegFK(Eigen::Isometry3d &B, Vector6d &q, int side);
    /**
     * \b NOTE: This is \em not thoroughly tested. Use with extreme caution.
     *
     * Similar to huboArmIK() but applied to the leg
    */
    void huboLegIK(Vector6d &q, const Eigen::Isometry3d B, Vector6d qPrev, int side);

    /**
     * A specialized Forward Kinematics calculation which accounts for the end effector
     * transformation of our experimental drill.
    */
    void HuboDrillFK(Eigen::Isometry3d &B, Vector6d &q);
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
    void HuboDrillIK(Vector6d &q, double y);

    // ~~~*** Center of Mass ***~~~ //

    /**
     * Still experimental. PLEASE DO NOT USE.
    */
    void initLocalCOMs();
    /**
     * Still experimental. PLEASE DO NOT USE.
    */
    Eigen::Vector3d getCOM_FullBody();
    /**
     * Still experimental. PLEASE DO NOT USE.
    */
    Eigen::Vector3d getCOM_Arm( int _side = LEFT );
    /**
     * Still experimental. PLEASE DO NOT USE.
    */
    Eigen::Vector3d getCOM_Leg( int _side = LEFT );
    /**
     * Still experimental. PLEASE DO NOT USE.
    */
    Eigen::Vector3d getCOM_Torso();

protected:

    void techInit();

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
    ach_channel_t chan_hubo_aux_ctrl;

    hubo_ref H_Ref;
    hubo_board_cmd H_Cmd;
    hubo_state H_State;
    hubo_ctrl_state C_State;
    hubo_arm_control H_Arm_Ctrl[2];
    hubo_leg_control H_Leg_Ctrl[2];
    hubo_fin_control H_Fin_Ctrl[2];
    hubo_aux_control H_Aux_Ctrl;
    
    hubo_param H_Param;

    int armjoints[2][ARM_JOINT_COUNT];
    int legjoints[2][LEG_JOINT_COUNT];
    int finjoints[2][FIN_JOINT_COUNT];

    bool ctrlOn[8];
    // 0) Right Arm
    // 1) Left Arm
    // 2) Right Leg
    // 3) Left Leg
    // 4) Right Fingers
    // 5) Left Fingers
    // 6) Auxiliary ( Neck & Waist )



    /** Constant values */
    static const int mNumDofs_Arm = 6;
    static const int mNumDofs_Leg = 6;
    static const int mNumDofs_Torso = 2;

    double mMass_Total_Arm;
    double mMass_Total_Leg;
    std::vector<Eigen::Vector3d> mLocalCOMs_Leg;
    std::vector<double> mMasses_Leg;

    std::vector<Eigen::Vector3d> mLocalCOMs_Arm;
    std::vector<double> mMasses_Arm;

    std::vector<Eigen::Vector3d> mLocalCOMs_Torso;
    std::vector<double> mMasses_Torso;

private:
    
    int ctrlMap[HUBO_JOINT_COUNT];
    int localMap[HUBO_JOINT_COUNT];
};

#endif // HUBO_PLUS_H
