
#include "DrcHuboKin.h"
#include <RobotKin/urdf_parsing.h>


using namespace std;
using namespace Eigen;
using namespace RobotKin;


DrcHuboKin::DrcHuboKin()
    : Robot()
{
    linkages_.resize(0);


    if( !RobotKinURDF::loadURDF(*this, "/etc/hubo-ach/drchubo_v2.urdf") )
         RobotKinURDF::loadURDF(*this, "/etc/hubo-ach/drchubo-v2.urdf");




    linkage("Body_RSP").name("RightArm");
    linkage("Body_LSP").name("LeftArm");
    linkage("Body_RHY").name("RightLeg");
    linkage("Body_LHY").name("LeftLeg");

    joint("LKP").name("LKN");
    joint("RKP").name("RKN");
    joint("REP").name("REB");
    joint("LEP").name("LEB");
    joint("TSY").name("WST");

    joint("LEB").max(0);
    joint("REB").max(0);

    // Note: These are all basically meaningless
//    joint("RF11").name("RF1");
//    joint("RF21").name("RF2");
//    joint("LF11").name("LF1");
//    joint("LF21").name("LF2");

    updateFrames();

/*
    RobotKin::TRANSFORM toolTf[RIGHT] = RobotKin::TRANSFORM::Identity();
//    toolTf[RIGHT].translate(joint("RWR_dummy").respectToFixed().translation());
    toolTf[RIGHT] = joint("RWR_dummy").respectToFixed();
    linkage("RightArm").tool().respectToFixed(toolTf[RIGHT]);
    toolTf[LEFT] = RobotKin::TRANSFORM::Identity();
//    toolTf[LEFT].translate(joint("LWR_dummy").respectToFixed().translation());
    toolTf[LEFT] = joint("LWR_dummy").respectToFixed();
    linkage("LeftArm").tool().respectToFixed(toolTf[LEFT]);
*/
/*
    toolTfR = RobotKin::TRANSFORM::Identity();
    toolTfR.translate(joint("RWR_dummy").respectToFixed().translation());
    toolTfL = RobotKin::TRANSFORM::Identity();
    toolTfL.translate(joint("LWR_dummy").respectToFixed().translation());
*/
    toolTfR = joint("RWR_dummy").respectToFixed();
    toolTfL = joint("LWR_dummy").respectToFixed();
    
    linkage("RightArm").tool().respectToFixed(toolTfR);
    linkage("LeftArm").tool().respectToFixed(toolTfL);


//    linkage("LeftArm").tool().respectToFixed(joint("LWR_dummy").respectToFixed());
//    linkage("RightArm").tool().respectToFixed(joint("RWR_dummy").respectToFixed()); 

//    TRANSFORM foot = joint("LAR_dummy").respectToFixed();
//    std::cout << foot.matrix() << std::endl;
//    TRANSFORM footWorld = joint("LAR_dummy").respectToRobot();
//    std::cout << footWorld.matrix() << std::endl;
//    foot.translate(-foot.translation());
//    foot.rotate(footWorld.rotation().transpose());
//    linkage("LeftLeg").tool().respectToFixed(foot);

//    foot = joint("RAR_dummy").respectToFixed();
//    footWorld = joint("RAR_dummy").respectToRobot();
//    foot.rotate(footWorld.rotation().transpose());
//    linkage("RightLeg").tool().respectToFixed(foot);
//    updateFrames();


    armRestValues.resize(7);
    armRestValues <<  0, 0, 0, -30*M_PI/180, 0, 0, 0;

    
    armConstraints.performNullSpaceTask = false;
    armConstraints.maxAttempts = 1;
    armConstraints.maxIterations = 20;
    armConstraints.convergenceTolerance = 0.001;
    armConstraints.wrapToJointLimits = false;
    armConstraints.wrapSolutionToJointLimits = false;
    armConstraints.restingValues(armRestValues);
    armConstraints.performNullSpaceTask = false;
    
    
    jointVals.resize(7); 
}

DrcHuboKin::DrcHuboKin(string filename)
    : Robot(filename, "drchubo")
{
}

void DrcHuboKin::resetTool(int side)
{
/*
    if( side == LEFT )
        linkage("LeftArm").tool().respectToFixed(toolTf[LEFT]);
    else
        linkage("RightArm").tool().respectToFixed(toolTf[RIGHT]);
*/

    if( side == LEFT )
        linkage("LeftArm").tool().respectToFixed(toolTfL);
    else
        linkage("RightArm").tool().respectToFixed(toolTfR);

/*
    if( side == LEFT )
        linkage("LeftArm").tool().respectToFixed(joint("LWR_dummy").respectToFixed());
    else
        linkage("RightArm").tool().respectToFixed(joint("RWR_dummy").respectToFixed());
*/
}

void DrcHuboKin::setTool(int side, const TRANSFORM offset)
{
    if( side == LEFT )
        linkage("LeftArm").tool().respectToFixed(
                  toolTfL*offset);
    else
        linkage("RightArm").tool().respectToFixed(
                  toolTfR*offset);
}

RobotKin::TRANSFORM DrcHuboKin::getTool(int side)
{
    if( side == LEFT )
        return toolTfL.inverse() * linkage("LeftArm").tool().respectToFixed();
    else
        return toolTfR.inverse() * linkage("RightArm").tool().respectToFixed();
}

RobotKin::rk_result_t DrcHuboKin::armTorques(int side, ArmVector &jointTorque, const Vector6d &eeWrench)
{
    VectorXd torques;
    if(side==RIGHT)
        linkage("RightArm").gravityJointTorques(torques);
    else
        linkage("LeftArm").gravityJointTorques(torques);


    MatrixXd J;
    if(side==RIGHT)
        linkage("RightArm").jacobian(J, linkage("RightArm").tool().respectToLinkage().translation(), &linkage("RightArm"));
    else
        linkage("LeftArm").jacobian(J, linkage("LeftArm").tool().respectToLinkage().translation(), &linkage("LeftArm"));

    torques += J.transpose()*eeWrench;


    for(int i=0; i<7; i++)
        jointTorque[i] = torques[i];

    for(int i=7; i<ARM_JOINT_COUNT; i++)
        jointTorque[i] = 0;

    return RK_SOLVED;
}

RobotKin::rk_result_t DrcHuboKin::armTorques(int side, ArmVector &jointTorque, const Vector6d &eeWrench, const ArmVector &jointAngles)
{
    updateArmJoints(side, jointAngles);
    return armTorques(side, jointTorque, eeWrench, jointAngles);
}

void DrcHuboKin::updateHubo(Hubo_Control &hubo, bool state)
{
    if(state)
    {
        for(int i=0; i<HUBO_JOINT_COUNT; i++)
            if( !(strcmp(jointNames[i], "RF1")==0
                    || strcmp(jointNames[i], "RF2")==0
                    || strcmp(jointNames[i], "RF3")==0
                    || strcmp(jointNames[i], "RF4")==0
                    || strcmp(jointNames[i], "RF5")==0
                    || strcmp(jointNames[i], "LF1")==0
                    || strcmp(jointNames[i], "LF2")==0
                    || strcmp(jointNames[i], "LF3")==0
                    || strcmp(jointNames[i], "LF4")==0
                    || strcmp(jointNames[i], "LF5")==0
                    || hubo.H_State.joint[i].active==0) )
                setJointValue(jointNames[i], hubo.getJointAngleState(i), true);
    }
    else
    {
        for(int i=0; i<HUBO_JOINT_COUNT; i++)
            if( !(strcmp(jointNames[i], "RF1")==0
                    || strcmp(jointNames[i], "RF2")==0
                    || strcmp(jointNames[i], "RF3")==0
                    || strcmp(jointNames[i], "RF4")==0
                    || strcmp(jointNames[i], "RF5")==0
                    || strcmp(jointNames[i], "LF1")==0
                    || strcmp(jointNames[i], "LF2")==0
                    || strcmp(jointNames[i], "LF3")==0
                    || strcmp(jointNames[i], "LF4")==0
                    || strcmp(jointNames[i], "LF5")==0
                    || hubo.H_State.joint[i].active==0) )
                setJointValue(jointNames[i], hubo.getJointAngle(i), true);
    }
    updateFrames();
}

void DrcHuboKin::updateArmJoints(int side, const ArmVector &jointValues)
{
    if(side==RIGHT)
        for(int i=0; i<linkage("RightArm").nJoints(); i++)
            linkage("RightArm").setJointValue(i, jointValues[i]);
    else
        for(int i=0; i<linkage("LeftArm").nJoints(); i++)
            linkage("LeftArm").setJointValue(i, jointValues[i]);
}

void DrcHuboKin::updateLegJoints(int side, const LegVector &jointValues)
{
    if(side==RIGHT)
        for(int i=0; i<linkage("RightLeg").nJoints(); i++)
            linkage("RightLeg").setJointValue(i, jointValues[i]);
    else
        for(int i=0; i<linkage("LeftLeg").nJoints(); i++)
            linkage("LeftLeg").setJointValue(i, jointValues[i]);
}

TRANSFORM DrcHuboKin::armFK(int side)
{
    if(side==RIGHT)
        return linkage("RightArm").tool().respectToRobot();
    else
        return linkage("LeftArm").tool().respectToRobot();
}

TRANSFORM DrcHuboKin::legFK(int side)
{
    if(side==RIGHT)
        return linkage("RightLeg").tool().respectToRobot();
    else
        return linkage("LeftLeg").tool().respectToRobot();
}


ArmJacobian DrcHuboKin::armJacobian(int side)
{
    ArmJacobian result; result.setZero();
    MatrixXd J;
    if( side == RIGHT )
        linkage("RightArm").jacobian(J, linkage("RightArm").tool().respectToRobot().translation(), this);
    else if( side == LEFT )
        linkage("LeftArm").jacobian(J, linkage("LeftArm").tool().respectToRobot().translation(), this);

    result = J.block<6,7>(0,0);
    return result;
}

ArmJacobian DrcHuboKin::armJacobian(int side, ArmVector &q)
{
    updateArmJoints(side, q);
    return armJacobian(side);
}


RobotKin::rk_result_t DrcHuboKin::armIK(int side, ArmVector &q, const TRANSFORM target)
{
    for(int i=0; i<7; i++)
        jointVals[i] = q[i];

    RobotKin::rk_result_t result;
    if(side==LEFT)
        result = dampedLeastSquaresIK_linkage("LeftArm", jointVals, target, armConstraints);
    else
        result = dampedLeastSquaresIK_linkage("RightArm", jointVals, target, armConstraints);

    for(int i=0; i<7; i++)
        q[i] = jointVals[i];

    for(int i=7; i<ARM_JOINT_COUNT; i++)
        q[i] = 0;


    return result;
}

RobotKin::rk_result_t DrcHuboKin::armIK(int side, ArmVector &q, const TRANSFORM target, const ArmVector &qNull)
{
    RobotKin::rk_result_t result;

    result = armIK(side, q, target);

    if((qNull-q).norm() > 1e-10)
    {
        double maxAngle = 0;
        int maxIdx = -1;
        for(int i=0; i<q.size(); i++)
        {
            if(fabs(qNull[i]-q[i]) > fabs(maxAngle))
            {
                maxAngle = qNull[i]-q[i];
                maxIdx = i;
            }
        }

        if(maxIdx == -1)
        {
            std::cout << "This should not be happening." << std::endl;
            return result;
        }

        Eigen::VectorXd dNull(7); dNull.setZero();
        dNull(maxIdx) = maxAngle;

        ArmJacobian J = armJacobian(side, q);
        JacobiSVD<ArmJacobian> svdJ;

        svdJ.compute(J, ComputeFullV);


        double nullStepSize = 5/180*M_PI/200;
        double thresh = 1e-6;
        double max = 0;
        int col = -1;
        for(int i=0; i<svdJ.singularValues().size(); i++)
        {
            if( svdJ.singularValues()(i) < thresh)
            {
                double check = svdJ.matrixV().col(i).transpose().dot(dNull);
                if( fabs(check) >= fabs(max) )
                {
                    max = check;
                    col = i;
                }
            }
        }

        for(int i=svdJ.singularValues().size(); i<svdJ.matrixV().cols(); i++)
        {
            double check = svdJ.matrixV().col(i).transpose().dot(dNull);
            if( fabs(check) >= fabs(max) )
            {
                max = check;
                col = i;
            }
        }

        if( col >= 0 && fabs(max) > 1e-6 )
        {
            VectorXd nullStep = max*svdJ.matrixV().col(col).transpose()/fabs(max);
            clampMaxAbs(nullStep, nullStepSize);

            for(int j=0; j<nullStep.size(); j++)
                q[j] += nullStep[j];
        }
    }

    return result;
}

RobotKin::rk_result_t DrcHuboKin::legIK(int side, LegVector &q, const Eigen::Isometry3d target)
{ return legIK(side, q, target, q); }

RobotKin::rk_result_t DrcHuboKin::legIK(int side, LegVector &q, const Eigen::Isometry3d target, const LegVector &qPrev)
{
    // FIXME: Clean up all the slop in this function and test it
    Vector6d qA;
    for(int i=0; i<6; i++)
        q(i) = qA(i);

    Eigen::ArrayXXd qAll(6,8);

    // Declarations
    Eigen::Isometry3d waist, waistInv, BInv, foot, footInv;
    Eigen::MatrixXd limits(6,2);
    Vector6d offset; offset.setZero();
    double nx, sx, ax, px;
    double ny, sy, ay, py;
    double nz, sz, az, pz;
    double q1, q2, q3, q4, q5, q6;
    double C45, psi, q345;
    Eigen::Matrix<int, 8, 3> m;

    double S2, S4, S6;
    double C2, C4, C5, C6;

    RobotKin::Linkage tempLinkage;
    if(side==LEFT)
        tempLinkage = linkage("LeftLeg");
    else
        tempLinkage = linkage("RightLeg");

    // Parameters
    // TODO: Consider removing some of these
//    double l1 = (79.5+107)/1000.0;      // Neck to waist Z
    double l2 = fabs(tempLinkage.joint(1).respectToRobot().translation()[1]); //88.43/1000.0;           // Waist to hip  Y
    double l3 = fabs(tempLinkage.joint(1).respectToRobot().translation()[2]); //(289.47-107)/1000.0;   // Waist to hip  Z
    double l4 = fabs(tempLinkage.joint(3).respectToFixed().translation()[2]); //300.03/1000.0;          // Hip to knee   Z
    double l5 = fabs(tempLinkage.joint(4).respectToFixed().translation()[2]); //300.38/1000.0;          // Knee to ankle Z
    double l6 = fabs(tempLinkage.joint(6).respectToFixed().translation()[2]);           // Ankle to foot Z

//    double l1 = 0;          // Neck to waist Z
//    double l2 = 0.0885;     // Waist to hip  Y
//    double l3 = 0.16452;    // Waist to hip  Z
//    double l4 = 0.33008;    // Hip to knee   Z
//    double l5 = 0.32995;    // Knee to ankle Z
//    double l6 = 0.119063;   // Ankle to foot Z


    // Transformation from Neck frame to Waist frame
//    neck(0,0) = 1; neck(0,1) =  0; neck(0,2) = 0; neck(0,3) =   0;
//    neck(1,0) = 0; neck(1,1) =  1; neck(1,2) = 0; neck(1,3) =   0;
//    neck(2,0) = 0; neck(2,1) =  0; neck(2,2) = 1; neck(2,3) = -l1;
//    neck(3,0) = 0; neck(3,1) =  0; neck(3,2) = 0; neck(3,3) =   1;

    limits <<
            tempLinkage.joint(0).min(), tempLinkage.joint(0).max(),
            tempLinkage.joint(1).min(), tempLinkage.joint(1).max(),
            tempLinkage.joint(2).min(), tempLinkage.joint(2).max(),
            tempLinkage.joint(3).min(), tempLinkage.joint(3).max(),
            tempLinkage.joint(4).min(), tempLinkage.joint(4).max(),
            tempLinkage.joint(5).min(), tempLinkage.joint(5).max();


    if (side == LEFT) {
        // Transformation from Waist frame to left hip yaw frame
        waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
        waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) =  l2;
        waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
        waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;

//        waist = linkage("LeftLeg").joint(1).respectToRobot();



    } else {
        // Transformation from Waist frame to right hip yaw frame
        waist(0,0) = 0; waist(0,1) = -1; waist(0,2) = 0; waist(0,3) =   0;
        waist(1,0) = 1; waist(1,1) =  0; waist(1,2) = 0; waist(1,3) = -l2;
        waist(2,0) = 0; waist(2,1) =  0; waist(2,2) = 1; waist(2,3) = -l3;
        waist(3,0) = 0; waist(3,1) =  0; waist(3,2) = 0; waist(3,3) =   1;

//        waist = linkage("RightLeg").joint(1).respectToRobot();

    }

    // Rotation of -90 about y to make x forward, y left, z up
    foot(0,0) = 0;  foot(0,1) =  0; foot(0,2) =-1;  foot(0,3) = 0;
    foot(1,0) = 0;  foot(1,1) =  1; foot(1,2) = 0;  foot(1,3) = 0;
    foot(2,0) = 1;  foot(2,1) =  0; foot(2,2) = 0;  foot(2,3) = 0;
    foot(3,0) = 0;  foot(3,1) =  0; foot(3,2) = 0;  foot(3,3) = 1;



    waistInv = waist.inverse();
    footInv = foot.inverse();

    // Variables
    BInv = (waistInv*target*footInv).inverse();

    nx = BInv(0,0); sx = BInv(0,1); ax = BInv(0,2); px = BInv(0,3);
    ny = BInv(1,0); sy = BInv(1,1); ay = BInv(1,2); py = BInv(1,3);
    nz = BInv(2,0); sz = BInv(2,1); az = BInv(2,2); pz = BInv(2,3);

    m <<
     1,  1,  1,
     1,  1, -1,
     1, -1,  1,
     1, -1, -1,
    -1,  1,  1,
    -1,  1, -1,
    -1, -1,  1,
    -1, -1, -1;

    for (int i = 0; i < 8; i++) {
        C4 = ((l6 + px)*(l6 + px) - l4*l4 - l5*l5 + py*py + pz*pz)/(2*l4*l5);
        double complex radical = 1-C4*C4;
        q4 = atan2(m(i,0)*creal(csqrt(radical)),C4);

        S4 = sin(q4);
        psi = atan2(S4*l4, C4*l4+l5);
        radical = ((px+l6)*(px+l6)+(py*py));
        q5 = wrapToPi(atan2(-pz, m(i,1)*creal(csqrt(radical)))-psi);

        q6 = atan2(py, -px-l6);
        C45 = cos(q4+q5);
        C5 = cos(q5);
        if (C45*l4 + C5*l5 < 0)
        {
            q6 = wrapToPi(q6 + M_PI);
        }

        S6 = sin(q6);
        C6 = cos(q6);

        S2 = C6*ay + S6*ax;
        radical = 1-S2*S2;
        q2 = atan2(S2,m(i,2)*creal(csqrt(radical)));

        q1 = atan2(C6*sy + S6*sx,C6*ny + S6*nx);
        C2 = cos(q2);
        if (C2 < 0) {
            q1 = wrapToPi(q1 + M_PI);
        }

        q345 = atan2(-az/C2,-(C6*ax - S6*ay)/C2);
        q3 = wrapToPi(q345-q4-q5);

        qAll(0,i) = q1;
        qAll(1,i) = q2;
        qAll(2,i) = q3;
        qAll(3,i) = q4;
        qAll(4,i) = q5;
        qAll(5,i) = q6;
    }

    // Set to offsets
    for (int i = 0; i < 6; i++) {
        if (side==RIGHT) {
            qA(i) = wrapToPi(qA(i) + offset(i));
        } else {
            qA(i) = wrapToPi(qA(i) + offset(i));
        }
    }

    // Find best solution
    Eigen::ArrayXd qDiff(6,1); qDiff.setZero();
    Eigen::ArrayXd qDiffSum(8,1);
    bool withinLim[8];
    int minInd;
    double zeroSize = 0.000001;

    // if any joint solution is infintesimal, set it to zero
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if(qAll(j,i) < zeroSize && qAll(j,i) > -zeroSize)
                qAll(j,i) = 0.0;

    // Initialize withinLim to all trues for all eight solutions
    for(int i=0; i<8; i++)
        withinLim[i] = true;

    // Check each set of solutions to see if any are outside the limits
    for(int i=0; i<8; i++)
        for(int j=0; j<6; j++)
            if( limits(j,0) > qAll(j,i) || qAll(j,i) > limits(j,1) )
                withinLim[i] = false;

    // Initialze anyWithin boolean array to all trues
    bool anyWithin=false;
    for(int i=0; i<8; i++)
        if( withinLim[i] )
            anyWithin = true;


    // If any solution has all joints within the limits...
    if(anyWithin)
    {
        // for each solution...
        for (int i = 0; i < 8; i++)
        {
            // if all the joints of solution i are within the limits...
            if( withinLim[i] )
            {
                // calculate the differences between solution angles, j, and previous angles
                for (int j=0; j < 6; j++)
                {
                    qDiff(j) = wrapToPi(qAll(j,i) - qPrev(j));
                }
                // sum the absolute values of the differences to get total difference
                qDiffSum(i) = qDiff.abs().sum();
            }
            // if the solution doesn't have all the joints within the limits...
            else
                // set the difference for that solution to infinity
                qDiffSum(i) = std::numeric_limits<double>::infinity();
        }
        // and take the solution closest to previous solution
        qDiffSum.minCoeff(&minInd);
        qA = qAll.col(minInd);
    }



    // if no solution has all the joints within the limits...
    else
    {
        // then for each solution...
        for(int i=0; i<8; i++)
        {

            // create a 6d vector of angles of solution i
            Vector6d qtemp = qAll.col(i).matrix();

            // take the min of the angles and the joint upper limits
            qtemp = qtemp.cwiseMin(limits.col(1));
            // then take the max of those angles and the joint lower limits
            qtemp = qtemp.cwiseMax(limits.col(0));
            // create an Isometry3d 4x4 matrix for the temp pose
            Eigen::Isometry3d Btemp;
            // find the pose associated with the temp angles
            //huboLegFK( Btemp, qtemp, side );

            if(side==LEFT)
            {
                for(size_t k=0; k<6; k++)
                    linkage("LeftLeg").setJointValue(k, qtemp[k]);
                Btemp = linkage("LeftLeg").tool().respectToLinkage();
            }
            else
            {
                for(size_t k=0; k<6; k++)
                    linkage("RightLeg").setJointValue(k, qtemp[k]);
                Btemp = linkage("RightLeg").tool().respectToLinkage();
            }
            // calculate the distance from previous pose to temp pose locations
            qDiffSum(i) = (Btemp.translation() - target.translation()).norm();
        }
        // find the solution that's closest the previous position
        qDiffSum.minCoeff(&minInd);
        qA = qAll.col(minInd).matrix();
    }
    // set the final joint angles to the solution closest to the previous solution

    for( int i=0; i<6; i++)
        qA(i) = max( min( qA(i), limits(i,1)), limits(i,0) );

    for(int i=0; i<6; i++)
        q(i) = qA(i);
    for(int i=6; i<LEG_JOINT_COUNT; i++)
        q(i) = 0;

    if(anyWithin)
        return RobotKin::RK_SOLVED;
    else
        return RobotKin::RK_NO_SOLUTION;
}



double sign(double x)
{
    return (x < 0) ? -1 : (x > 0);
}

void DrcConstraints::iterativeJacobianSeed(Robot &robot, size_t attemptNumber,
                                           const std::vector<size_t> &indices, Eigen::VectorXd &values)
{
    if(attemptNumber == 0)
    {
        return;
    }
    else
    {
        if(values(EB) > -5*M_PI/180)
            values(EB) = -30*M_PI/180;

        if( robot.joint(indices[SR]).name()=="RSR" && fabs(values(SR)+90*M_PI/180) < 3*M_PI/180)
            values(SR) = (-90+10*sign(values(SR)+90*M_PI/180))*M_PI/180;

        if( robot.joint(indices[SR]).name()=="LSR" && fabs(values(SR)-90*M_PI/180) < 3*M_PI/180)
            values(SR) = (90+10*sign(values(SR)+90*M_PI/180))*M_PI/180;

        if( fabs(values(WP)) < 3*M_PI/180 )
            values(WP) = sign(values(WP))*10*M_PI/180;

    }
}





