
#include "MotionTrajectory.h"
#include <ctype.h>

int peek(FILE* fp) {
  int c = fgetc(fp);
  ungetc(c, fp);
  return c;
}

void skip_whitespace(FILE* fp)
{
    int c = peek(fp);
    if(isspace(c) && c != '\n')
    {
        fgetc(fp);
    }
}


bool parseNextElement(const char *filename, int line, FILE *fp, motion_element_t &elem)
{
    const int joint_order[] = {
      RHY, RHR, RHP, RKN, RAP, RAR,
      LHY, LHR, LHP, LKN, LAP, LAR,
      RSP, RSR, RSY, REB, RWY, RWR, RWP,
      LSP, LSR, LSY, LEB, LWY, LWR, LWP,
      NKY, NK1, NK2, WST,
      RF1, RF2, RF3, RF4, RF5,
      LF1, LF2, LF3, LF4, LF5,
      -1
    };

    int j;

    memset(&elem, 0, sizeof(elem));

    for(j=0; joint_order[j] >= 0; j++)
    {
        skip_whitespace(fp);

        int c = peek(fp);

        if( c == EOF || c == '\n' )
        {
            fprintf(stderr, "Line %d has ended abruptly!\n", line); fflush(stderr);
            return false;
        }

        if( joint_order[j] >= HUBO_JOINT_COUNT )
        {
            fprintf(stderr, "Joint %d (%d) is out of bounds!\n"
                    " -- See line %d of MotionTrajectory.cpp", joint_order[j], j, __LINE__ ); fflush(stderr);
            return false;
        }

        if( fscanf(fp, "%lf", &(elem.angles[j])) != 1 )
        {
            fprintf(stderr, "%s:%d: error parsing number\n", filename, line); fflush(stderr);
            return false;
        }
    }

    skip_whitespace(fp);

    int c = fgetc(fp);

    if(c != EOF && c != '\n')
    {
        fprintf(stderr, "%s:%d: too many tokens after numbers\n", filename, line);
    }

    return true;
}


#ifdef __cplusplus

bool fillMotionTrajectoryFromText(const char *filename, motion_trajectory_t &motion_trajectory)
{
    bool validFile = false;
    FILE* fp;
    if(fp = fopen(filename, "r"))
        validFile = true;
    else
        fprintf(stderr, "Cannot open trajectory file name %s!", filename);

    motion_element_t elem;

    motion_trajectory.resize(0);

    int line=0;
    while(!feof(fp) && validFile)
    {
        validFile = parseNextElement(filename, line, fp, elem);
        if(validFile)
            motion_trajectory.push_back(elem);
        line++;
    }

    return validFile;
}


#endif // __cplusplus
















