

#include "DrcHuboKin.h"
#include "calibration.h"
#include <limits>

using namespace std;
using namespace RobotKin;

double findDeadband(Hubo_Control& hubo, int joint)
{
    hubo.update();

    double pwm=0;

    int complement;
    switch( joint )
    {
        case LSP:
            complement = LSR; break;
        case LSR:
            complement = LSP; break;
        case LSY:
            complement = LEB; break;
        case LEB:
            complement = LSY; break;
        case LWY:
            complement = LWP; break;
        case LWP:
            complement = LWY; break;

        
        case RSP:
            complement = RSR; break;
        case RSR:
            complement = RSP; break;
        case RSY:
            complement = REB; break;
        case REB:
            complement = RSY; break;
        case RWY:
            complement = RWP; break;
        case RWP:
            complement = RWY; break;
    }


    hubo.setJointPWM(complement, 0);

    hubo.update();
    double stime = hubo.getTime();
    double time = hubo.getTime();
    do
    {
        hubo.update();
        time = hubo.getTime();
    } while( fabs(hubo.getJointVelocity(joint)) > 1e-5 || (time-stime) < 3)
    

    while(fabs(hubo.getJointVelocity(joint)) < 1e-3)
    {
        pwm -= 0.1;
        cout << "Checking deadband of " << jointNames[joint] << " at PWM " << pwm << endl;
        hubo.setJointPWM(joint, pwm, true);

        sleep(1);
        hubo.update();
    }

    hubo.setJointPWM(joint, 0, true);

    return fabs(pwm) - 0.1;
}

bool calibrateJoint(string joint,
                    double mass, double comx, double comy, double comz,
                    double resolution)
{
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        if(joint.compare(jointNames[i])==0)
            return calibrateJoint(i, mass, comx, comy, comz, resolution);

    return false;
}



bool calibrateJoint(int joint,
                    double mass, double comx, double comy, double comz,
                    double resolution)
{

    Hubo_Control hubo;
    DrcHuboKin kin;
    hubo.update();
    kin.updateHubo(hubo);
    TRANSLATION com(comx, comy, comz);

    ArmVector startAngles, currentAngles, zeroAngles;
    startAngles.setZero(); currentAngles.setZero(); zeroAngles.setZero();

    double rangeStart = 0;
    double rangeEnd = 0;

    string name = "";

    int side;

    switch( joint )
    {
    case LSP:
    case RSP:
    case LSR:
    case RSR:
    case LEB:
    case REB:
    case LWP:
    case RWP:
        rangeStart = 0;
        rangeEnd = -M_PI/2;
        break;

    case LSY:
    case RSY:
        startAngles(SP) = M_PI/2;
        startAngles(EB) = -M_PI/2;
        rangeStart = 0;
        rangeEnd = -M_PI/2;
        break;

    case LWY:
    case RWY:
        startAngles(EB) = -M_PI/2;
        startAngles(WP) = -M_PI/2;
        rangeStart = -2.18; // TODO: Base this off of the mass model
        rangeEnd = -0.5585; // TODO: Base this off of the mass model
        break;

    // TODO: Add in wrist roll

    default:
        cout << "Invalid joint: " << jointNames[joint] << " (" << joint << ")" << endl;
        return false;
    }

    if(         joint == LSR
            ||  joint == LSY
            ||  joint == LWY )
    {
        rangeStart = -rangeStart;
        rangeEnd   = -rangeEnd;
    }

    switch( joint )
    {
    case LSP:
    case LSR:
    case LSY:
    case LEB:
    case LWY:
    case LWP:
    case LWR:
        side = LEFT; break;
    default:
        side = RIGHT; break;
    }

    string limb;
    if(side==LEFT)
        limb = "LeftArm";
    else
        limb = "RightArm";

    kin.linkage(limb).tool().massProperties.setMass(mass, com);

    name = string(jointNames[joint]);


    hubo.setArmCompliance(side, false);
    hubo.setArmAngles(side, zeroAngles, true);
    hubo.update();
    hubo.getArmAngleStates(side, currentAngles);

    cout << endl << "Moving into position to find the deadband of " << jointNames[joint] << endl;
    while((zeroAngles-currentAngles).norm() > 0.01)
    {
        hubo.update();
        hubo.getArmAngleStates(side, currentAngles);
    }
    cout << " :: Ready!" << endl << endl;

    double deadBand = findDeadband(hubo, joint);
    // TODO: print/report deadband

    cout << endl << "Found deadband for " << jointNames[joint] << ": " << deadBand << endl;

    hubo.update();
    hubo.setArmAngles(side, startAngles, true);
    hubo.update();
    hubo.getArmAngleStates(side, currentAngles);

    cout << endl << "Moving into position to calibrate " << jointNames[joint] << endl;
    while((startAngles-currentAngles).norm() > 0.01)
    {
        hubo.update();
        hubo.getArmAngleStates(side, currentAngles);
    }
    cout << " :: Ready!" << endl << endl;




    double pwm = deadBand*(rangeEnd-rangeStart)/fabs(rangeEnd-rangeStart);

    while( fabs(hubo.getJointAngleState(joint)-rangeStart) < fabs(rangeEnd-rangeStart) )
    {
        pwm += resolution*(rangeEnd-rangeStart)/fabs(rangeEnd-rangeStart);

        cout << "Checking torque of " << pwm << "\% Duty for joint " << jointNames[joint] << endl;
        hubo.setJointPWM(joint, pwm, true);

        double stime = hubo.getTime();
        double time = hubo.getTime();
        do
        {
            hubo.update();
            time = hubo.getTime();
        } while( (fabs(hubo.getJointVelocity(joint)) > 1e-5 || (time-stime) < 3)
                && fabs(hubo.getJointAngleState(joint)-rangeStart) < fabs(rangeEnd-rangeStart)  );

        kin.updateHubo(hubo);

        if( fabs(hubo.getJointAngleState(joint)-rangeStart) < fabs(rangeEnd-rangeStart) )
            // Print result to table
            cout << "Angle: " << hubo.getJointAngleState(joint)
                 << "\tTorque: " << kin.joint(jointNames[joint]).gravityTorque()
                 << "\t PWM: " << hubo.getJointDuty(joint) << endl;
    }

    hubo.setJointCompliance(joint, true);
    hubo.passJointAngle(joint, rangeEnd, true);

    do {
        hubo.update();
    } while( fabs(hubo.getJointVelocity(joint)) > 1e-3 );

    hubo.setJointCompliance(joint, false);
    hubo.sendControls();

    hubo.update();

    hubo.setJointAngle(joint, rangeStart, true);

    /*
     *
    hubo.setJointMaxPWM(joint, 10);
    double testAngle = rangeStart;
    for(int i=0; i<=resolution; i++)
    {
        hubo.setJointCompliance(joint, false);
        hubo.setJointAngle(joint, testAngle, true);
        cout << "Moving joint " << jointNames[joint] << " to " << testAngle << "..."; fflush(stdout);
        while( fabs(testAngle-hubo.getJointAngleState(joint)) > 0.01 )
            hubo.update();
        cout << " Arrived!" << endl;

        hubo.setJointCompliance(joint, true, 120, 0);
        hubo.sendControls();

        double stime = hubo.getTime();
        double time = hubo.getTime();
        do
        {
            hubo.update();
            time = hubo.getTime();
        } while( fabs(hubo.getJointVelocity(joint)) > 1e-5 || (time-stime) < 3 );

        kin.updateHubo(hubo);


        cout << "Angle: " << hubo.getJointAngleState(joint)
             << "\tTorque: " << kin.joint(jointNames[joint]).gravityTorque()
             << "\t PWM: " << hubo.getJointDuty(joint) << endl;

        // TODO: Grab the current PWM reading and record it versus the torque prediction

        testAngle += (rangeEnd-rangeStart)/resolution;
    }
    */

    cout << endl << "Finished with joint " << jointNames[joint] << "!" << endl;

    return true;
}

void calibrateArm(int side, bool prompt, double mass, double comx, double comy, double comz, int resolution)
{
    if(prompt)
        cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );

    if(side == RIGHT)
    {
        calibrateJoint(RSP, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
        calibrateJoint(RSR, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
        calibrateJoint(RSY, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
        calibrateJoint(REB, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
        calibrateJoint(RWY, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
        calibrateJoint(RWP, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
    }
    else if(side == LEFT)
    {
        calibrateJoint(LSP, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
        calibrateJoint(LSR, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
        calibrateJoint(LSY, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
        calibrateJoint(LEB, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
        calibrateJoint(LWY, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
        calibrateJoint(LWP, mass, comx, comy, comz, resolution);
        if(prompt)
        {
            cout << endl << "Press ENTER to continue...";
            cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
        }
    }
}

void calibrateRightArm(bool prompt, double mass, double comx, double comy, double comz, int resolution)
{
    calibrateArm(RIGHT, prompt, mass, comx, comy, comz, resolution);
}

void calibrateLeftArm(bool prompt, double mass, double comx, double comy, double comz, int resolution)
{
    calibrateArm(LEFT, prompt, mass, comx, comy, comz, resolution);
}

