/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Grey <mxgrey@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
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
 *
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
    Hubo_Tech(const char *daemon_name);
    /**
     * Gets the current time in seconds from Hubo's 'hubo_state' struct member 'time' over the state ach channel
    */
    double getTime();

    /**
     * Returns the latest data from the hubo_state and hubo_ref ach channels.
     * \n The hubo_state channel contains:
     * \li IMU: angular position and velocity around x, y, z;
     * \li Force/Torque: moment in x and y; force in z;
     * \li Joints: latest encoder reference value, position (radians), current (amps), velocity (rad/s), heat (J), temperature (C), if it's active, if it's zeroed;
     * \li Joint motor controller (JMC) state;
     * \li Current time (s); refWait (whether or not to wait before before sending reference commands).
     * \n The hubo_ref channel contains:
     * \li Joint reference (encoder value);
     * \li Joint status: GOOD or FROZEN;
     * \li Timestamp: time reference was sent.
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
     * Sets joint to position control.
    */
    tech_flag_t setPositionControl( int joint );
    /**
     * Sets joint angle for the joint to specified radian value. If send is true then 
     * the command will be sent to the motor board immediately, otherwise it won't.
    */
    tech_flag_t setJointAngle( int joint, double radians, bool send=false );
    /**
     * Sets the nominal speed for the joint to speed in radians/s.
    */
    tech_flag_t setJointNominalSpeed( int joint, double speed );
    // Velocity control
    /**
     * Sets the joint to velocity control.
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
     * Sets the entire arm to position control. Which arm to set is specified by the side parameter.
    */
    tech_flag_t setArmPosCtrl( int side );
    /**
     * Sets the joint angles for all the arm angles of the arm specified by the side argument
     * to the values specified by the angles argument. The send argument specifies whether or
     * not to send the commands immediately or not.
    */
    tech_flag_t setArmAngles( int side, Vector6d angles, bool send=false );
    /**
     * Sets the left arm joints to position control
    */
    void setLeftArmPosCtrl();
    /**
     * Sets the left arm joint angles to the values specified by the angles argument. The send argument
     * specifies whether or not to send the commands immediately or not.
    */
    tech_flag_t setLeftArmAngles( Vector6d angles, bool send=false );
    /**
     * Sets the right arm joints to position control
    */
    void setRightArmPosCtrl();
    /**
     * Sets the right arm joint angles to the values specified by the angles argument. The send argument
     * specifies whether or not to send the commands immediately or not.
    */
    tech_flag_t setRightArmAngles( Vector6d angles, bool send=false );
    /**
     * Sets the nominal speeds for the arm joints, specified by the side argument, to the values
     * specified by the speeds argument.
    */
    tech_flag_t setArmNomSpeeds( int side, Vector6d speeds );
    /**
     * Sets the nominal speeds for the left arm joints to the values specified by the speeds argument.
    */
    tech_flag_t setLeftArmNomSpeeds( Vector6d speeds );
    /**
     * Sets the nominal speeds for the right arm joints to the values specified by the speeds argument.
    */
    tech_flag_t setRightArmNomSpeeds( Vector6d speeds );
    // Velocity control
    /*
     * Sets the arm joints to velocity control. The side argument specifies which arm (LEFT or RIGHT).
    */
    tech_flag_t setArmVelCtrl( int side );
    /*
     * Sets the velocities (in rad/s) for the joints of the arm to the values specified by vels argument.
     * The side argument specifies which arm (LEFT or RIGHT). The send argument specifies whether or not
     * to send the commands immediately.
    */
    tech_flag_t setArmVels( int side, Vector6d vels, bool send=false );
    /**
     * Sets the left arm joints to velocity controlled. The send argument specifies whether or not
     * to send the commands immediately.
    */
    void setLeftArmVelCtrl();
    /**
     * Sets the velocities for the left arm joints to the values specified by the vels argument.
     * The send argument specifies whether or not to send the commands immediately
    */
    tech_flag_t setLeftArmVels( Vector6d vels, bool send=false );
    /**
     * Sets the right arm joints to velocity controlled. The send argument specifies whether or not
     * to send the commands immediately.
    */
    void setRightArmVelCtrl();
    tech_flag_t setRightArmVels( Vector6d vels, bool send=false );
    // Acceleration settings
    tech_flag_t setArmNomAcc(int side, Vector6d acc );
    tech_flag_t setLeftArmNomAcc( Vector6d acc );
    tech_flag_t setRightArmNomAcc( Vector6d acc );

    // ~* Leg control sets
    // Position control
    tech_flag_t setLegPosCtrl( int side );
    tech_flag_t setLegAngles( int side, Vector6d angles, bool send=false );
    void setLeftLegPosCtrl();
    tech_flag_t setLeftLegAngles( Vector6d angles, bool send=false );
    void setRightLegPosCtrl();
    tech_flag_t setRightLegAngles( Vector6d angles, bool send=false );
    tech_flag_t setLegNomSpeeds( int side, Vector6d speeds );
    tech_flag_t setLeftLegNomSpeeds( Vector6d speeds );
    tech_flag_t setRightLegNomSpeeds( Vector6d speeds );
    // Velocity control
    tech_flag_t setLegVelCtrl( int side );
    tech_flag_t setLegVels( int side, Vector6d vels, bool send=false );
    void setLeftLegVelCtrl();
    tech_flag_t setLeftLegVels( Vector6d vels, bool send=false );
    void setRightLegVelCtrl();
    tech_flag_t setRightLegVels( Vector6d vels, bool send=false );
    // Acceleration settings
    tech_flag_t setLegNomAcc(int side, Vector6d acc );
    tech_flag_t setLeftLegNomAcc( Vector6d acc );
    tech_flag_t setRightLegNomAcc( Vector6d acc );

    // ~~** Setting limit values
    // ~* General sets
    tech_flag_t setJointAngleMin( int joint, double radians );
    tech_flag_t setJointAngleMax( int joint, double radians );
    tech_flag_t setJointSpeedMax( int joint, double speed );

    // ~~** Send Off Latest Control Commands
    void sendControls();


    // ~~** Getting Reference Values
    // ~* General gets
    // Position control
    hubo_ctrl_mode_t getCtrlMode( int joint );
    double getJointAngle( int joint );
    double getJointAngleCtrl( int joint );
    double getJointNominalSpeed( int joint );
    // Velocity control
    double getJointVelocity( int joint );
    //double getJointVelocityState( int joint ); // TODO: add velocity to the state
    // Acceleration setting
    double getJointNominalAcceleration( int joint );

    int getJointStatus( int joint ); // 0:Good 1:Frozen

    // ~* Arm control gets
    // Position control
    tech_flag_t getArmAngles( int side, Vector6d &angles );
    void getLeftArmAngles( Vector6d &angles );
    void getRightArmAngles( Vector6d &angles );
    tech_flag_t getArmNomSpeeds( int side, Vector6d &speeds );
    void getLeftArmNomSpeeds( Vector6d &speeds );
    void getRightArmNomSpeeds( Vector6d &speeds );
    // Velocity control
    tech_flag_t getArmVels( int side, Vector6d &vels );
    void getLeftArmVels( Vector6d &vels );
    void getRightArmVels( Vector6d &vels );
    // Acceleration settings
    tech_flag_t getArmNomAcc(int side, Vector6d &acc );
    void getLeftArmNomAcc( Vector6d &acc );
    void getRightArmNomAcc( Vector6d &acc );

    // ~* Leg control gets
    // Position control
    tech_flag_t getLegAngles( int side, Vector6d &angles );
    void getLeftLegAngles( Vector6d &angles );
    void getRightLegAngles( Vector6d &angles );
    tech_flag_t getLegNomSpeeds( int side, Vector6d &speeds );
    void getLeftLegNomSpeeds( Vector6d &speeds );
    void getRightLegNomSpeeds( Vector6d &speeds );
    // Velocity control
    tech_flag_t getLegVels( int side, Vector6d &vels );
    void getLeftLegVels( Vector6d &vels );
    void getRightLegVels( Vector6d &vels );
    // Acceleration settings
    tech_flag_t getLegNomAcc(int side, Vector6d &acc );
    void getLeftLegNomAcc( Vector6d &acc );
    void getRightLegNomAcc( Vector6d &acc );

    // ~~** Getting limit values
    // ~* General gets
    double getJointAngleMin( int joint );
    double getJointAngleMax( int joint );
    double getJointSpeedMax( int joint );

    // Passthrough Mode: Reference values will not be touched by
    // the control-daemon. Use of this mode is strongly discouraged.
    tech_flag_t passJointAngle( int joint, double radians, bool send=false );

    // ~~~*** State Readings ***~~~ //

    // ~~** State
    double getJointAngleState( int joint );
    tech_flag_t getArmAngleStates( int side, Vector6d &angles );
    void getRightArmAngleStates( Vector6d &angles );
    void getLeftArmAngleStates( Vector6d &angles );
    tech_flag_t getLegAngleStates( int side, Vector6d &angles );
    void getRightLegAngleStates( Vector6d &angles );
    void getLeftLegAngleStates( Vector6d &angles );
    // TODO: All of these (state position, velocity, whatever)

    // ~~** Sensors
    // ~* Force-torque
    // Mx
    double getMx( hubo_sensor_index_t sensor );
    double getRightHandMx();
    double getLeftHandMx();
    double getRightFootMx();
    double getLeftFootMx();
    // My
    double getMy( hubo_sensor_index_t sensor );
    double getRightHandMy();
    double getLeftHandMy();
    double getRightFootMy();
    double getLeftFootMy();
    // Fz
    double getFz( hubo_sensor_index_t sensor );
    double getRightFootFz();
    double getLeftFootFz();
    // ~* Accelerometers
    // Tilt X
    double getTiltX( int side );
    double getLeftTiltX();
    double getRightTiltX();
    // Tilt Y
    double getTiltY( int side );
    double getLeftTiltY();
    double getRightTiltY();
    // Tilt Z
    double getTiltZ( int side );
    double getLeftTiltZ();
    double getRightTiltZ();
    // ~* IMU
    double getAngleX();
    double getAngleY();
    double getRotVelX();
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
    
    void huboArmFK(Eigen::Isometry3d &B, Vector6d &q, int side);
    void huboArmFK(Eigen::Isometry3d &B, Vector6d &q, int side, const Eigen::Isometry3d &endEffector);
    
    void huboArmIK(Vector6d &q, Eigen::Isometry3d B, Vector6d qPrev, int side);
    void huboArmIK(Vector6d &q, Eigen::Isometry3d B, Vector6d qPrev, int side, const Eigen::Isometry3d &endEffector);
    
    void huboLegFK(Eigen::Isometry3d &B, Vector6d &q, int side);

    void huboLegIK(Vector6d &q, Eigen::Isometry3d B, Vector6d qPrev, int side);

    void HuboDrillFK(Eigen::Isometry3d &B, Vector6d &q);
    void HuboDrillIK(Vector6d &q, double y);

    // ~~~*** Center of Mass ***~~~ //

    /** Functions */
    void initLocalCOMs();
    Eigen::Vector3d getCOM_FullBody();
    Eigen::Vector3d getCOM_Arm( int _side = LEFT );
    Eigen::Vector3d getCOM_Leg( int _side = LEFT );
    Eigen::Vector3d getCOM_Torso();

protected:

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

    void techInit();


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
