#include <iostream>
using std::cout;
using std::endl;

#include <fstream>
using std::ifstream;
using std::ofstream;
using namespace std;

#include <cstring>
#include <cstdlib>
#include <cmath>
#include <string>

#include <eigen3/Eigen/Geometry>
#include "DrcHuboKin.h"
using namespace RobotKin;
using namespace Eigen;


const int MAX_CHARS_PER_LINE = 512;
const char* const DELIMITER = " ";


void readTraj(std::string filename);

int main(int argc, char **argv) {
    
    std::string filename(argv[1]);
    //std:: cout << filename << std::endl;
    //const char* file = "offline_ingress_1.txt";
    readTraj(filename);
    return 0;
}

void readTraj(std::string filename)
{
    ArmVector qLA, qRA, qLL, qRL, qLAprev, qRAprev, qLLprev, qRLprev;
    Vector3d tempPel;
    Vector3d tempArm;
    Vector3d tempLeg;
    AngleAxisd tempRotArm;
    AngleAxisd tempRotLeg;
    
    double time = 0, dt = 0;

//    bool live = false;
    bool live = true;

    DrcHuboKin robot;
//    Hubo_Control hubo(false);
    Hubo_Control hubo(live);
//    daemon_prioritize(40);

    robot.updateHubo(hubo);		// current state of hubo

    clock_t countTime;
    clock_t endTime;
    int tests=2;

    clock_t totalTime = clock();

    //rainbow init pose

    // TODO: Actually move the robot here at startup

//	robot.joint("RHY").value(0*3.1415926/180);
//    robot.joint("RHR").value(0*3.1415926/180);
//    robot.joint("RHP").value(-10*3.1415926/180);
//    robot.joint("RKN").value(20*3.1415926/180);
//    robot.joint("RAP").value(-10*3.1415926/180);
//    robot.joint("RAR").value(0*3.1415926/180);

    qRL(HY) =   0*M_PI/180;
    qRL(HR) =   0*M_PI/180;
    qRL(HP) = -10*M_PI/180;
    qRL(KN) =  20*M_PI/180;
    qRL(AP) = -10*M_PI/180;
    qRL(AR) =   0*M_PI/180;

    hubo.setRightLegAngles(qRL);


//    robot.joint("LHY").value(0*3.1415926/180);
//    robot.joint("LHR").value(0*3.1415926/180);
//    robot.joint("LHP").value(-10*3.1415926/180);
//    robot.joint("LKN").value(20*3.1415926/180);
//    robot.joint("LAP").value(-10*3.1415926/180);
//    robot.joint("LAR").value(0*3.1415926/180);

    qLL(HY) =   0*M_PI/180;
    qLL(HR) =   0*M_PI/180;
    qLL(HP) = -10*M_PI/180;
    qLL(KN) =  20*M_PI/180;
    qLL(AP) = -10*M_PI/180;
    qLL(AR) =   0*M_PI/180;

    hubo.setLeftLegAngles(qLL);

    robot.joint("WST").value(0*3.1415926/180);

    hubo.setJointAngle(WST, 0);

//    robot.joint("RSP").value(20*3.1415926/180);
//    robot.joint("RSR").value(-45*3.1415926/180);
//    robot.joint("RSY").value(0*3.1415926/180);
//    robot.joint("REB").value(-90*3.1415926/180);
//    robot.joint("RWY").value(0*3.1415926/180);
//    robot.joint("RWP").value(-20*3.1415926/180);
//    robot.joint("RWR").value(-45*3.1415926/180);

    qRA(SP) =  20*M_PI/180;
    qRA(SR) = -45*M_PI/180;
    qRA(SY) =   0*M_PI/180;
    qRA(EB) = -90*M_PI/180;
    qRA(WY) =   0*M_PI/180;
    qRA(WP) = -20*M_PI/180;
    qRA(WR) = -45*M_PI/180;

    hubo.setRightArmAngles(qRA);


//    robot.joint("LSP").value(20*3.1415926/180);
//    robot.joint("LSR").value(45*3.1415926/180);
//    robot.joint("LSY").value(0*3.1415926/180);
//    robot.joint("LEB").value(-90*3.1415926/180);
//    robot.joint("LWY").value(0*3.1415926/180);
//    robot.joint("LWP").value(-20*3.1415926/180);
//    robot.joint("LWR").value(45*3.1415926/180);

    qLA(SP) =  20*M_PI/180;
    qLA(SR) =  45*M_PI/180;
    qLA(SY) =   0*M_PI/180;
    qLA(EB) = -90*M_PI/180;
    qLA(WY) =   0*M_PI/180;
    qLA(WP) = -20*M_PI/180;
    qLA(WR) =  45*M_PI/180;

    hubo.setLeftArmAngles(qLA);

    hubo.setJointNominalSpeed(RAP, hubo.getJointNominalSpeed(RHP));
    hubo.setJointNominalSpeed(RKN, 2*hubo.getJointNominalSpeed(RHP));

    hubo.setJointNominalSpeed(LHP, hubo.getJointNominalSpeed(RHP));
    hubo.setJointNominalSpeed(LAP, hubo.getJointNominalSpeed(LHP));
    hubo.setJointNominalSpeed(LKN, 2*hubo.getJointNominalSpeed(LHP));

    hubo.setArmCompliance(RIGHT, false);
    hubo.setArmCompliance(LEFT, false);

    hubo.sendControls();

    do {
        
        hubo.update(live);
        hubo.getRightArmAngleStates(qRAprev);
        hubo.getLeftArmAngleStates(qLAprev);
        hubo.getRightLegAngleStates(qRLprev);
        hubo.getLeftLegAngleStates(qLLprev);
        
    } while( (  (qRA-qRAprev).norm() > 0.075
              || (qLA-qLAprev).norm() > 0.075
              || (qLL-qLLprev).norm() > 0.075
              || (qRL-qRLprev).norm() > 0.075 )
             && live );

    RobotKin::TRANSFORM tfArm[2], tfLeg[2], startTfPelvis, stepTfPelvis, tfPelvis, stepTfArm[2], stepTfLeg[2], startTfArm[2], startTfLeg[2];


    robot.updateArmJoints(RIGHT, qRA);
    startTfArm[RIGHT] = robot.linkage("RightArm").tool().respectToRobot();
    robot.updateArmJoints(LEFT, qLA);
    startTfArm[LEFT] = robot.linkage("LeftArm").tool().respectToRobot();

    robot.updateLegJoints(RIGHT, qRL);
    startTfLeg[RIGHT] = robot.linkage("RightLeg").tool().respectToRobot();
    robot.updateLegJoints(LEFT, qLL);
    startTfLeg[LEFT] = robot.linkage("LeftLeg").tool().respectToRobot();

    startTfPelvis = robot.respectToWorld();

    // create a file-reading object
    ifstream fin;

    // open a file
    fin.open(filename.c_str());
    if (!fin.good())

        return; // exit if file not found

    //std::ofstream myfile("joint_angles_taegoo_computer1.txt");

    hubo.update(live);   // set it as true when running on robot
    //dt = hubo.getTime() - time;
    //time = hubo.getTime();

	int k = 0;
    // read file line by line
    while(!fin.eof()) {
        char buf[MAX_CHARS_PER_LINE];
        fin.getline(buf, MAX_CHARS_PER_LINE);
        // parsed data
        float data[31];
        char* line_value;
		
        // parse the line
        line_value = strtok(buf, DELIMITER); // first data


        int n = 0;

        while(line_value != NULL)
        {
            //cout << line_value << DELIMITER;
            data[n++] = atof(line_value);
            line_value = strtok(NULL, DELIMITER);
        }	


        // Initialization
        hubo.update(live);
        dt = hubo.getTime() - time;
        time = hubo.getTime();
//        robot.updateHubo(hubo);
//        hubo.getRightArmAngles(qRA);
//        hubo.getLeftArmAngles(qLA);
//        hubo.getRightLegAngles(qRL);
//        hubo.getLeftLegAngles(qLL);

//        hubo.setArmCompliance(RIGHT, true);
//        hubo.setArmCompliance(LEFT, true);

        // Isometry 3d transformation
        for (int i = 0; i < 5; ++i)
        {
            int pos = (i * 3);

            if (i == 0)
            {
//                Vector3d tempPel(data[2], data[1], data[0]);
                tempPel << data[2], data[1], data[0];
                tempPel = startTfPelvis.rotation().transpose()*tempPel;          // used when pelvis rotated
                //std::cout<< "parsed pelvis " << "\t" << tempPel.transpose() << std::endl;
                stepTfPelvis = TRANSFORM::Identity();
                stepTfPelvis.translate(tempPel);
            }
            else if(i > 0 && i < 3)
            {
//                Vector3d tempArm(data[pos],data[pos + 1], data[pos + 2]);
                tempArm << data[pos], data[pos+1], data[pos+2];
                tempRotArm = AngleAxisd(data[pos + (i-1) + 12], Vector3d(data[pos + (i-1) + 13], data[pos + (i-1) + 14], data[pos + (i-1) + 15]));
                //std::cout<< "parsed arm " << i-1 << "\t" << tempArm.transpose()<< "\t:\t" << tempRotArm.angle() << ", " << tempRotArm.axis().transpose() << std::endl;
                tempArm = startTfArm[i-1].rotation().transpose()*(tempArm - stepTfPelvis.translation());
                tempRotArm = startTfArm[i-1].rotation().transpose()*tempRotArm*startTfArm[i-1].rotation();
                stepTfArm[i-1] = TRANSFORM::Identity();
                stepTfArm[i-1].translate(tempArm);
                stepTfArm[i-1].rotate(tempRotArm);
            }
            else
            {
                tempLeg << data[pos],data[pos + 1], data[pos + 2];
                tempRotLeg = AngleAxisd(data[pos + (i-1) + 12], Vector3d(data[pos + (i-1) + 13], data[pos + (i-1) + 14], data[pos + (i-1) + 15]));
                //std::cout<< "parsed leg " << i-2 << "\t" << tempLeg.transpose()<< "\t:\t" << tempRotLeg.angle() << ", " << tempRotLeg.axis().transpose() << std::endl;
                tempLeg = startTfLeg[i-3].rotation().transpose()*(tempLeg - stepTfPelvis.translation());
                tempRotLeg = startTfLeg[i-3].rotation().transpose()*tempRotLeg*startTfLeg[i-3].rotation();
                stepTfLeg[i-3] = TRANSFORM::Identity();
                stepTfLeg[i-3].translate(tempLeg);
                stepTfLeg[i-3].rotate(tempRotLeg);
            }
        }


        if(!live)
        {
            std::cout << "Step Pelvis:\n" << stepTfPelvis.matrix() << std::endl;
            std::cout << "Before Pelvis:\n" << startTfPelvis.matrix()<< std::endl;
        }

        tfPelvis = startTfPelvis * stepTfPelvis;

        if(!live)
            std::cout << "After Pelvis:\n" << tfPelvis.matrix()<< std::endl;

        if(!live)
        {
            std::cout << "Step Right Arm:\n" << stepTfArm[RIGHT].matrix() << std::endl;
            std::cout << "Before Right Arm:\n" << startTfArm[RIGHT].matrix()<< std::endl;
        }
        tfArm[RIGHT] = startTfArm[RIGHT]*stepTfArm[RIGHT];
        if(!live)
            std::cout << "After Right Arm:\n" << tfArm[RIGHT].matrix() << std::endl;

        //tfArm[RIGHT] = startTfArm[RIGHT] * stepTfPelvis.inverse()*stepTfArm[RIGHT];
        //tfArm[LEFT] = startTfArm[LEFT] * stepTfPelvis.inverse()*stepTfArm[LEFT];

        if(!live)
        {
            std::cout << "Step Left Arm:\n" << stepTfArm[LEFT].matrix() << std::endl;
            std::cout << "Before Left Arm:\n" << startTfArm[LEFT].matrix()<< std::endl;
        }
        tfArm[LEFT]  = startTfArm[LEFT]*stepTfArm[LEFT];
        if(!live)
            std::cout << "After Left Arm:\n" << tfArm[LEFT].matrix() << std::endl;


        if(!live)
        {
            std::cout << "Step Right Leg:\n" << stepTfLeg[RIGHT].matrix() << std::endl;
            std::cout << "Before Right Leg:\n" << startTfLeg[RIGHT].matrix()<< std::endl;
        }

        tfLeg[RIGHT] = startTfLeg[RIGHT]*stepTfLeg[RIGHT];
        if(!live)
            std::cout << "After Right Leg:\n" << tfLeg[RIGHT].matrix() << std::endl;


        if(!live)
        {
            std::cout << "Step Left Leg:\n" << stepTfLeg[LEFT].matrix() << std::endl;
            std::cout << "Before Left Leg:\n" << startTfLeg[LEFT].matrix()<< std::endl;
        }
        tfLeg[LEFT]  = startTfLeg[LEFT]*stepTfLeg[LEFT];
        if(!live)
            std::cout << "After Left Leg:\n" << tfLeg[LEFT].matrix() << std::endl;



        // solving IK


        countTime = clock();

        rk_result_t rstatus = robot.armIK(RIGHT, qRA, tfArm[RIGHT]);
        rk_result_t lstatus = robot.armIK(LEFT, qLA, tfArm[LEFT]);

        endTime = clock();

/*
        cout << k << ":\t" << rk_result_to_string(rstatus) << ":"
             << rk_result_to_string(lstatus) << "\tTime: " << (endTime - countTime)/((double)CLOCKS_PER_SEC*tests) << " : " <<
                (double)CLOCKS_PER_SEC*tests/(endTime-countTime)
             << endl;
*/
		k = k + 1;


        robot.legIK(RIGHT, qRL, tfLeg[RIGHT]);
        robot.legIK(LEFT, qLL, tfLeg[LEFT]);
        // print out joint angles

        if(!live)
        {
            std::cout << "right arm angles\t" << qRA.transpose() << std::endl;
            std::cout << "left arm angles\t" << qLA.transpose() << std::endl;
            std::cout << "right leg angles\t" << qRL.transpose() << std::endl;
            std::cout << "left leg angles\t" << qLL.transpose() << std::endl;

            std::cout << "_________________________________________________________________" << std::endl;
        }


    //     write joint angles to textfile

//        std::ofstream myfile("joint_angles_hubo_read_new2.txt", ios::app);
/*
    myfile << qRA.transpose() << std::endl;
    myfile << qLA.transpose() << std::endl;
    myfile << qRL.transpose() << std::endl;
    myfile << qLL.transpose() << std::endl;
*/
/*		
		for(int p = 0; p < 4; p++)
			myfile << 0 << " ";

        for(int p = 0; p < 6; p++)
            myfile << qLA[p] << " ";

        for(int p = 0; p < 6; p++)
            myfile << qRA[p] << " ";

        for(int p = 0; p < 7; p++)
            myfile << qLL[p] << " ";

        for(int p = 0; p < 7; p++)
            myfile << qRL[p] << " ";
		
		for(int p = 0; p < 9; p++)
			myfile << 0 << " "; 


*/


/*


	// for sim on taegoo , hubo-read-trajectory format
        for(int p = 0; p < 6; p++)
            myfile << qRL[p] << " ";

        for(int p = 0; p < 6; p++)
            myfile << qLL[p] << " ";

        for(int p = 0; p < 7; p++)
            myfile << qRA[p] << " ";

	    for(int p = 0; p < 7; p++)
            myfile << qLA[p] << " ";

		for(int p = 0; p < 14; p++)
			myfile << 0 << " "; 


*/



    /*

        // joint angles for andy

        myfile << 0 << "\t";

        for(int p = 0; p < 6; p++)
            myfile << qLL[p] << "\t";

        for(int p = 0; p < 6; p++)
            myfile << qRL[p] << "\t";

        for(int p = 0; p < 7; p++)
            myfile << qLA[p] << "\t";

        for(int p = 0; p < 7; p++)
            myfile << qRA[p] << "\t";


        //myfile << ";" << "\t";	// for andy's matlab version
*/
   //     myfile << std::endl;




        // controlling actual robot
        // 1. safe way (constant accleration)
        hubo.setLeftArmAngles(qLA);
        hubo.setRightArmAngles(qRA);
        hubo.setLeftLegAngles(qLL);
        hubo.setRightLegAngles(qRL);


        /*
    // 2. do exactly as commanded (velocity needed)

// when traj is safe and accurate
hubo.setLeftArmTraj(qLA, (qLA - qLAprev)/dt);
hubo.setRightArmTraj(qRA, (qRA - qRAprev)/dt);
hubo.setLeftLegTraj(qLA, (qLL - qLLprev)/dt);
hubo.setRightLegTraj(qLA, (qRL - qRLprev)/dt);

*/	

        	hubo.sendControls();    // send to actual robot

        //qLAprev = qLA;
        //qRAprev = qRA;
        //qLLprev = qLL;
        //qRLprev = qRL;

    }

    fin.close();

    endTime = clock();

    std::cout << "Total time: " << (endTime - totalTime)/((double)CLOCKS_PER_SEC*k) << " : " <<
                 (double)CLOCKS_PER_SEC*k/(endTime-totalTime) << endl;

}
