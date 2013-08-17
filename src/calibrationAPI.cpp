

#include "DrcHuboKin.h"
#include "calibration.h"

using namespace std;

void calibrateJoint(int joint, int resolution)
{
    Hubo_Control hubo;
    DrcHuboKin kin;
    hubo.update();
    kin.updateHubo(hubo);

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
        startAngles(SP) = -M_PI/2;
        startAngles(EB) = -M_PI/2;
        rangeStart = 0;
        rangeEnd = -M_PI/2;
        break;

    case LWY:
    case RWY:
        startAngles(EB) = -M_PI/2;
        startAngles(WP) = -M_PI/2;
        rangeStart = -2.18;
        rangeEnd = -0.5585;
        break;

    // TODO: Add in wrist roll

    default:
        cout << "Invalid joint: " << jointNames[joint] << " (" << joint << ")" << endl;
        return;
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

    double deadBand = findDeadband(joint);


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


    bool finished = false;

    hubo.setJointCompliance(joint, false);
    hubo.setJointMaxPWM(joint, 10);

    double testAngle = rangeStart;

    for(int i=0; i<resolution; i++)
    {
        hubo.setJointAngle(joint, testAngle, true);
        while( fabs(testAngle-hubo.getJointAngleState(joint)) < 0.01 )
            hubo.update();

        hubo.setJointCompliance(joint, true, 80, 20);

        double stime = hubo.getTime();
        double time = hubo.getTime();
        do
        {
            hubo.update();
            time = hubo.getTime();
        } while( fabs(hubo.getJointVelocity(joint)) > 1e-5 && (time-stime) < 1 );

        // TODO: Grab the current PWM reading and record it versus the torque prediction
    }

}



double findDeadband(int joint)
{
    Hubo_Control hubo;
    hubo.update();

    double pwm=0;


    while(fabs(hubo.getJointVelocity(joint)) < 1e-3)
    {
        pwm += 0.1;
        cout << "Checking deadband of " << jointNames[joint] << " at PWM " << pwm << endl;
        hubo.setJointPWM(joint, pwm, true);

        sleep(1);
        hubo.update();
    }

    hubo.setJointPWM(joint, 0, true);

    return pwm - 0.1;
}
