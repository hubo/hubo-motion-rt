
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <iomanip>
#include "ImpedanceController.h"
#include "KeyboardInput.h"

int main(int argc, char **argv)
{
    Eigen::Matrix<double,6,1> dFeetOffset;
    Eigen::Vector3d forceTorqueErr;

    double dt = 0.005; // seconds
    forceTorqueErr.setZero();
    dFeetOffset.setZero();

    double M = 49;
    Eigen::Vector3d K, Q;

    K << 0,0,2000.0;
    Q << 0,0,180.0;

    forceTorqueErr(2) = 4;

    ImpedanceController impCtrl(M);

    // Keyboard input initialization and setup
    keyboard_init();

    char c;
    int counter = 0;
    int counterMax = 40;

    while(1)
    {
        counter++;

        // Get character
        if(read(STDIN_FILENO, &c, 1) == 1)
        {
            if('u' == c)
                M += 2;
            else if('j' == c)
                M -= 2;
            else if('i' == c)
                K(2) += 5;
            else if('k' == c)
                K(2) -= 5;
            else if('o' == c)
                Q(2) += 5;
            else if('l' == c)
                Q(2) -= 5;
            else if('y' == c)
                forceTorqueErr(2) += 10;
            else if('h' == c)
                forceTorqueErr(2) -= 10;
            else {}
        }

        // Set impedance controller gains Mass, Spring, Damper
        impCtrl.setMass(M);
        impCtrl.setGains(K,Q);

        // Run impedance controller
        impCtrl.run(dFeetOffset, forceTorqueErr, dt);

        if(counter >= counterMax)
        {
            // Print out result
            std::cout << std::setprecision(4)
                      << "F = " << forceTorqueErr(2) << " off = " << dFeetOffset(2)
                      << " M = " << M << " K = " << K(2) << " Q = " << Q(2)
                      << std::endl;
        }

        // Increment and reset counter

        if(counter >= counterMax)
            counter = 0;
    }

    return 0;
}

