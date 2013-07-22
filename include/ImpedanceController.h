/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Pete Vieira <pete.vieira@gmail.com>
 * Date: July 21, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
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


#ifndef IMPEDANCE_CONTROLLER_H
#define IMPEDANCE_CONTROLLER_H

class ImpedanceController
{
public:

    /**
     * \brief Constructor for ImpedanceController class
    */
    ImpedanceController(double mass=default_mass);

    /**
     * \brief Destructor for ImpedanceController class
    */
    ~ImpedanceController();

    //-----------------------
    //   MEMBER FUNCTIONS
    //-----------------------
    /**
     * \brief takes the current desired feet offset positions and velocities
     *        and adjusts it to achieve desired force in the z-direction and moment
     *        about the ankles
     * \return no return value. it adjust the desired feet offset by reference
    */
    void run(Eigen::Matrix<double,6,1>  &dFeet, const Eigen::Vector3d &dForceTorque, double dt);

    /**
     * \brief Sets the spring gains, damping gains mass in the impedance controller
     * \return void
    */
    void setGains(const Eigen::Vector3d &spring_gain, const Eigen::Vector3d &damping_gain, const double mass=default_mass);

    //-------------------------
    //    MEMBER VARIABLES
    //-------------------------
    Eigen::Vector3d m_K;    //!< Spring gain for impedance controller
    Eigen::Vector3d m_Q;    //!< Damping gain for impedance controller
    double m_M;             //!< Mass of robot for impedance controller
    static const double default_mass = 45.0; //kg
    // best numbers so far: M=0.008, Q=0.4, K=7.0

private:

    /**
     * \brief takes in position and velocity of feet offset and
     *      returns output of impedance controller, which is
     *      the velocity and acceleration in the feet offset
     *      based on the change in force/torque value at the ankles.
     * \return 3d vector of qdot and qddot
    */
    Eigen::Matrix<double,6,1> impedanceEq(const Eigen::Matrix<double,6,1> &dFeet, const Eigen::Vector3d &dForceTorque);

    /**
     * \brief Intergration using runge-kutta method rk4
     *      to get feet offset position and velocity from
     *      feet offset velocity and acceleration.
     * \return Void. No return. The feet offset position
     *      and velocity get updated by reference.
    */
    void rk4(Eigen::Matrix<double,6,1> &dFeet, const Eigen::Vector3d &dForceTorque, double dt);


};

#endif // IMPEDANCE_CONTROLLER_H
