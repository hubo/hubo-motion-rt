
#include "DrcHuboKin.h"
#include "manip.h"

using namespace RobotKin;
using namespace Eigen;
using namespace std;



int main(int argc, char **argv)
{
    DrcHuboKin kin;
    ArmVector qNull;
//    qNull <<  0.833292, 0.140585, -0.235877, -2.14833, 0.762452, -0.809249, -0.382926, 0, 0, 0;
//    qNull << -0.849666, -1.19113, 2.24967, -2.14833, -1.87977, 0.857494, -1.61893, 0, 0, 0;
    qNull << 1.02719, -0.49557, 0.463562, -1.7126, -0.70092, -2.62406, -1.17622, 0, 0, 0;
    kin.armConstraints.maxAttempts = 2;

    kin.updateArmJoints(RIGHT, qNull);
    cout << "Null Goal: " << endl << kin.armFK(RIGHT).matrix() << endl << endl;

    TRANSFORM goal = kin.armFK(RIGHT);
//    TRANSFORM goal = TRANSFORM::Identity();
//    goal.translate(Vector3d(0.201,-0.2295,-0.3257));
//    goal.rotate(AngleAxisd(90.0/180.0*M_PI, Vector3d::UnitX()));
//    goal.rotate(AngleAxisd(-90.0/180.0*M_PI, Vector3d::UnitY()));

    cout << "Goal: " << endl << goal.matrix() << endl << endl;

//    kin.joint("REB").value(-30.0/180.0*M_PI);

    ArmVector q; q.setZero();
//    q(EB) = -12.0/180*M_PI;
    kin.updateArmJoints(RIGHT, q);


    rk_result_t r = RK_SOLVED;



    r = kin.armIK(RIGHT, q, goal, qNull);
    ArmVector firstDiff = qNull-q;

    int iterations = 2000;
    if(argc > 1)
        iterations = atoi(argv[1]);
    for(int i=0; i<iterations; i++)
    {
        r = kin.armIK(RIGHT, q, goal, qNull);
    }
    cout << rk_result_to_string(r) << endl;

    cout << "dq: " << (qNull-q).transpose() << endl;
    cout << "Goal:   " << qNull.transpose() << endl;
    cout << "Actual: " << q.transpose() << endl << endl;

    cout << "First: " << firstDiff.transpose() << endl;
    cout << "Last:  " << (qNull-q).transpose() << endl;
    cout << "Improvement: " << firstDiff.norm() - (qNull-q).norm() << endl;
    cout << "Final Dist:  " << (qNull-q).norm() << endl;
    cout << "Iterations: (" << iterations << ")" << endl;

}
