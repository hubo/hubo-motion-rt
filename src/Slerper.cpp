

#include "Slerper.h"

using namespace RobotKin;

Slerper::Slerper() :
    kin(),
    hubo()
{
#ifdef HAVE_REFLEX
    aa_mem_region_init(&reg, 1024*32);


#endif //HAVE_REFLEX


}

#ifdef HAVE_REFLEX
void Slerper::commenceSlerping(int side, hubo_manip_cmd_t &cmd)
{
    hubo.update();
    kin.updateHubo(hubo);


















    aa_mem_region_release(&reg);

    rfx_trajx_parablend_t T;
    rfx_trajx_splend_init(&T, &reg, 1);

    rfx_trajx_t *pT = (rfx_trajx_t*)&T;

    // NOTE: Amino quaternions are x, y, z, w

    double X[3];

    double R[4];


}

#else //HAVE_REFLEX
void Slerper::commenceSlerping(hubo_manip_cmd_t &cmd)
{
    std::cerr << "We do not currently have Eigen support" << std::endl;
}
#endif //HAVE_REFLEX
