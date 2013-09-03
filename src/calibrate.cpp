/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: August 9, 2013
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


#include "calibration.h"
#include <iostream>

using namespace std;




int main(int argc, char **argv)
{
    cout << endl << endl
         << "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||" << endl
         << "----------------------------------------------------------------------" << endl
         << "|                What would you like to calibrate?                   |" << endl
         << "----------------------------------------------------------------------" << endl
         << "|   all    --   (cycle through each joint and calibrate each one)    |" << endl
         << "|                                                                    |" << endl
         << "|   right  --   (calibrate each joint on the right arm)              |" << endl
         << "|                                                                    |" << endl
         << "|   left   --   (calibrate each joint on the left arm)               |" << endl
         << "|                                                                    |" << endl
         << "|   JNT    --   (calibrate a specific joint)                         |" << endl
         << "|  Note: Instead of JNT, type in the 3-letter identifier (e.g. RSP)  |" << endl
         << "----------------------------------------------------------------------" << endl
         << " >> ";

    string target_str;
    bool prompt = true;

    double lmass=0, lcomx=0, lcomy=0, lcomz=0,
            rmass=0, rcomx=0, rcomy=0,rcomz=0;

    cin >> target_str;

    if(target_str.compare("all")==0
            || target_str.compare("right")==0
            || target_str.compare("left")==0)
    {
        bool valid=false;

        while(!valid)
        {
            cout << endl << endl
                 << " -- Should I prompt you each time a joint is finished? [y/n] >> ";

            string prompt_str;
            cin >> prompt_str;

            if(prompt_str.compare("y")==0 || prompt_str.compare("yes")==0)
            {
                cout << ":: Each time a joint finishes, I will wait for you to press enter!" << endl;
                prompt = true;
                valid = true;
            }
            else if(prompt_str.compare("n")==0 || prompt_str.compare("no")==0)
            {
                cout << ":: Okay, I won't stop until I'm finished!" << endl;
                prompt = false;
                valid = true;
            }
            else
            {
                cout << "... I don't understand that response. Just type y or n!" << endl;
                valid = false;
            }
        }
    }


    if( target_str.compare("right")==0 )
    {
        bool valid = false;

        while(!valid)
        {
            cout << endl << endl
                 << "Is the right hand holding additional weight? [y/n] >> ";

            string prompt_str;
            cin >> prompt_str;

            if(prompt_str.compare("y")==0 || prompt_str.compare("yes")==0)
            {
                cout << endl << "Alright, then provide me with the following details:";
                cout << endl << "Mass:  >> ";
                cin >> rmass;
                cout << endl << "CoM X: >> ";
                cin >> rcomx;
                cout << endl << "CoM Y: >> ";
                cin >> rcomy;
                cout << endl << "CoM Z: >> ";
                cin >> rcomz;

                calibrateRightArm(prompt, rmass, rcomx, rcomy, rcomz);

                valid = true;
            }
            else if(prompt_str.compare("n")==0 || prompt_str.compare("no")==0)
            {
                cout << endl << "Okay, then I'll just use the arm's natural weight.";

                calibrateRightArm(prompt);

                valid = true;
            }
            else
            {
                cout << "... I don't understand that response. Just type y or n!" << endl;
                valid = false;
            }

        }

    }
    else if( target_str.compare("left")==0 )
    {
        bool valid = false;

        while(!valid)
        {
            cout << endl << endl
                 << "Is the left hand holding additional weight? [y/n] >> ";

            string prompt_str;
            cin >> prompt_str;

            if(prompt_str.compare("y")==0 || prompt_str.compare("yes")==0)
            {
                cout << endl << "Alright, then provide me with the following details:";
                cout << endl << "Mass:  >> ";
                cin >> lmass;
                cout << endl << "CoM X: >> ";
                cin >> lcomx;
                cout << endl << "CoM Y: >> ";
                cin >> lcomy;
                cout << endl << "CoM Z: >> ";
                cin >> lcomz;

                calibrateLeftArm(prompt, lmass, lcomx, lcomy, lcomz);

                valid = true;
            }
            else if(prompt_str.compare("n")==0 || prompt_str.compare("no")==0)
            {
                cout << endl << "Okay, then I'll just use the arm's natural weight.";

                calibrateLeftArm(prompt);

                valid = true;
            }
            else
            {
                cout << "... I don't understand that response. Just type y or n!" << endl;
                valid = false;
            }
        }
    }
    else if( target_str.compare("all")==0 )
    {
        bool valid = false;

        while(!valid)
        {
            cout << endl << endl
                 << "Is the left hand holding additional weight? [y/n] >> ";

            string prompt_str;
            cin >> prompt_str;

            if(prompt_str.compare("y")==0 || prompt_str.compare("yes")==0)
            {
                cout << endl << "Alright, then provide me with the following details:";
                cout << endl << "Mass:  >> ";
                cin >> lmass;
                cout << endl << "CoM X: >> ";
                cin >> lcomx;
                cout << endl << "CoM Y: >> ";
                cin >> lcomy;
                cout << endl << "CoM Z: >> ";
                cin >> lcomz;

                calibrateLeftArm(prompt, lmass, lcomx, lcomy, lcomz);

                valid = true;
            }
            else if(prompt_str.compare("n")==0 || prompt_str.compare("no")==0)
            {
                cout << endl << "Okay, then I'll just use the arm's natural weight.";

                calibrateLeftArm(prompt);

                valid = true;
            }
            else
            {
                cout << "... I don't understand that response. Just type y or n!" << endl;
                valid = false;
            }
        }

        valid = false;

        while(!valid)
        {
            cout << endl << endl
                 << "Is the right hand holding additional weight? [y/n] >> ";

            string prompt_str;
            cin >> prompt_str;

            if(prompt_str.compare("y")==0 || prompt_str.compare("yes")==0)
            {
                cout << endl << "Alright, then provide me with the following details:";
                cout << endl << "Mass:  >> ";
                cin >> rmass;
                cout << endl << "CoM X: >> ";
                cin >> rcomx;
                cout << endl << "CoM Y: >> ";
                cin >> rcomy;
                cout << endl << "CoM Z: >> ";
                cin >> rcomz;

                calibrateRightArm(prompt, rmass, rcomx, rcomy, rcomz);

                valid = true;
            }
            else if(prompt_str.compare("n")==0 || prompt_str.compare("no")==0)
            {
                cout << endl << "Okay, then I'll just use the arm's natural weight.";

                calibrateRightArm(prompt);

                valid = true;
            }
            else
            {
                cout << "... I don't understand that response. Just type y or n!" << endl;
                valid = false;
            }

        }
    }
    else
    {
        bool validJoint;

        bool valid=false;

        while(!valid)
        {
            cout << endl << endl
                 << "Will the hand be holding additional weight? [y/n] >> ";

            string prompt_str;
            cin >> prompt_str;

            if(prompt_str.compare("y")==0 || prompt_str.compare("yes")==0)
            {
                cout << endl << "Alright, then provide me with the following details:";
                cout << endl << "Mass:  >> ";
                cin >> rmass;
                cout << endl << "CoM X: >> ";
                cin >> rcomx;
                cout << endl << "CoM Y: >> ";
                cin >> rcomy;
                cout << endl << "CoM Z: >> ";
                cin >> rcomz;

                validJoint = calibrateJoint(target_str, rmass, rcomx, rcomy, rcomz);

                valid = true;
            }
            else if(prompt_str.compare("n")==0 || prompt_str.compare("no")==0)
            {
                cout << endl << "Okay, then I'll just use the arm's natural weight.";

                validJoint = calibrateJoint(target_str);

                valid = true;
            }
            else
            {
                cout << "... I don't understand that response. Just type y or n!" << endl;
                valid = false;
            }
        }



        if(!validJoint)
        {
            cout << "... I don't understand that request. Next time, pick an option out of the table!" << endl;
            return 1;
        }
    }





    return 0;
}















