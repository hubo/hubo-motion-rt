#include "Hubo_Tech.h"



int main(int argc, char **argv)
{
    printf("About to construct hubo\n");
    Hubo_Tech hubo;
    printf("About to home everything\n");
//    hubo.homeJoint( LSR, true );
    hubo.homeAllJoints(true);

    return 0;
}
