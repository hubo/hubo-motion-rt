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

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include "ImpedanceController.h"

ImpedanceController::ImpedanceController(double mass) : m_M(mass),
                                                        m_K(0,0,0),
                                                        m_Q(0,0,0)
{
    std::cout << "Mass for impedance controller = " << m_M << std::endl;
}

ImpedanceController::~ImpedanceController()
{
}

Eigen::Matrix<double,6,1> ImpedanceController::impedanceEq(const Eigen::Matrix<double,6,1> &dFeet,
                                                            const Eigen::Vector3d &dForceTorque)
{

    // state space rep.
    Eigen::Matrix<double,6,1> dFeetDot;
    Eigen::Matrix<double,6,6> A;
    Eigen::Matrix<double,6,1> B;

    A.setZero();
    A.topRightCorner<3,3>() = Eigen::Matrix3d::Identity();
    A.bottomLeftCorner<3,3>() = (-m_K/m_M).asDiagonal();
    A.bottomRightCorner<3,3>() = (-m_Q/m_M).asDiagonal();
/*
    A <<
           0,    0,    0,    1,    0,    0,
           0,    0,    0,    0,    1,    0,
           0,    0,    0,    0,    0,    1,
       -Kx/M,    0,    0,-Qx/M,    0,    0;
           0,-Ky/M,    0,    0,-Qy/M,    0;
           0,    0,-Kz/M,    0,    0,-Qz/M;
*/
    B << 0, 0, 0, dForceTorque/m_M;

    dFeetDot = A*dFeet + B;
//    std::cout << "A:\n" << A << "\nB:\n" << B << "\ndFeet: "
//              << dFeet << "\ndFeetDot: " << dFeetDot << "\ndForceTorque: " << dForceTorque << std::endl;

    return dFeetDot;
};

void ImpedanceController::rk4(Eigen::Matrix<double,6,1> &dFeet, const Eigen::Vector3d &dForceTorque, double dt )
{
    Eigen::Matrix<double,6,1> k1, k2, k3, k4; // runge-kutta  increments

    // compute runge-kutta increments
    k1 = impedanceEq(dFeet, dForceTorque);
    k2 = impedanceEq(dFeet + .5*dt*k1, dForceTorque);
    k3 = impedanceEq(dFeet + .5*dt*k2, dForceTorque);
    k4 = impedanceEq(dFeet + dt*k3, dForceTorque);

    // compute new delta q
    dFeet = dFeet + dt*(k1 + 2*k2 + 2*k3 + k4)/6;
};

void ImpedanceController::run(Eigen::Matrix<double,6,1> &dFeet, const Eigen::Vector3d &dForceTorque, double dt)
{
    // Integrate using Runge-Kutta 4
    rk4(dFeet, dForceTorque, dt); // pass in dFeet(n) and get out dFeet(n+1)
};

void ImpedanceController::setGains(const Eigen::Vector3d &spring_gain, const Eigen::Vector3d &damping_gain, const double mass)
{
    m_K = spring_gain;
    m_Q = damping_gain;
    m_M = mass;
}
