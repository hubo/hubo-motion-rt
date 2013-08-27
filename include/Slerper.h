#ifndef SLERPER_H
#define SLERPER_H

#define HAVE_REFLEX // Uncomment this to get autocompletion
#ifdef HAVE_REFLEX
#include <amino.h>
#include <reflex.h>
#endif //HAVE_REFLEX


#include "DrcHuboKin.h"
#include "manip.h"



class Slerper
{
public:
    Slerper();
    ~Slerper();

    void commenceSlerping(int side, hubo_manip_cmd_t &cmd);

protected:

    Hubo_Control hubo;
    DrcHuboKin kin;

#ifdef HAVE_REFLEX
    aa_mem_region_t reg;

#endif //HAVE_REFLEX

};





















#endif // SLERPER_H
