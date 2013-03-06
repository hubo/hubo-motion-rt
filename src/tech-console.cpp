/*
Copyright (c) 2012, Daniel M. Lofaro
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may 
      be used to endorse or promote products derived from this software 
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: Feb 03, 2013
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

#include <Hubo_Control.h>

#include <stdio.h>
#include <stdlib.h>
#include <readline/readline.h>
#include <readline/history.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include <iostream>
#include <sstream>
#include <string>
using namespace std;



static char** my_completion(const char*, int ,int);
char* my_generator(const char*,int);
char * dupstr (char*);
void *xmalloc (int);
void parse(char *s);
int test(char *s , struct hubo *h);
char* getArg(string s, int argNum);
int name2mot(char*s, struct hubo_param *h);
int name2sensor(char* name, struct hubo_param *h);
double hubo_set(char*s, struct hubo_param *p);
char* cmd [] ={ "initialize","fet","initializeAll","homeAll",
                "ctrl","enczero", "goto","get","test","update", "quit","beep", "home"," "}; //,


int main() {
	printf("\n");
	printf(" ******************* hubo-tech ***************** \n");
	printf("       Support: Grey (mxgrey@gatech.edu \n"         );
	printf(" *********************************************** \n");
    printf(" Note: This is a derived version of Dan Lofaro's\n"
           " hubo-console. It will be replaced in the near\n"
           " future with a GUI." ); fflush(stdout);
    
    Hubo_Control hubo; printf(" -- Hubo ready!\n"); fflush(stdout);

    hubo_param H_param;
    hubo_state H_state;

    setJointParams( &H_param, &H_state );
    setSensorDefaults( &H_param );

    char *buf;
    rl_attempted_completion_function = my_completion;
    printf("\n");
    while((buf = readline(">> hubo-ach: "))!=NULL) {
        //enable auto-complete
        rl_bind_key('\t',rl_complete);

        printf("   ");

        /* get update after every command */
        hubo.update();
        
        int tsleep = 0;
        char* buf0 = getArg(buf, 0);

        if (strcmp(buf0,"update")==0) {
            hubo.update();
            printf("--->Hubo Information Updated\n");
        }
        else if (strcmp(buf0,"get")==0) {
            int jnt = hubo_set(buf, &H_param);
            char* tmp = getArg(buf,1);
            printf(">> %s = %f rad \n",tmp,hubo.getJointAngle(jnt));
        }
        else if (strcmp(buf0,"goto")==0) {
            int jnt = hubo_set(buf, &H_param);
            float f = 0.0;
            char* str = getArg(buf,2);
            if(sscanf(str, "%f", &f) != 0){  //It's a float.
                hubo.setJointAngle( jnt, f, true );
                printf(">> %s = %f rad \n",getArg(buf,1),f);
            }
            else {
                printf(">> Bad input \n");
            }
        }
        else if (strcmp(buf0,"beep")==0) {
            int jnt = name2mot(getArg(buf, 1), &H_param);
            double etime = atof(getArg(buf,2));
            hubo.jointBeep( jnt, etime, true );
        }
        else if (strcmp(buf0,"home")==0) {
            hubo.homeJoint( hubo_set(buf, &H_param), true );
            printf("%s - Home \n",getArg(buf,1));
        }
        else if (strcmp(buf0,"homeAll")==0) {
            hubo.homeAllJoints( true );
        }
        else if (strcmp(buf0,"reset")==0) {
            int jnt = name2mot(getArg(buf, 1), &H_param);
            hubo.resetJoint( jnt, true );
            printf("%s - Resetting Encoder \n",getArg(buf,1));
        }
        else if (strcmp(buf0,"startup")==0) {
            hubo.startAllSensors( true );
            printf("Starting up Hubo\n");
            tsleep = 2;
        }
        else if (strcmp(buf0,"ctrl")==0) {
            int onOrOff = atof(getArg(buf,2));
            if(onOrOff == 0 | onOrOff == 1) {
                int jnt = name2mot(getArg(buf,1),&H_param);  // set motor num
                if(onOrOff==1)			// 1 = on, 0 = 0ff
                    hubo.motorCtrlOn( jnt, true );
                else if(onOrOff==0)
                    hubo.motorCtrlOff( jnt, true );
                if(onOrOff == 0) {
                    printf("%s - Turning Off CTRL\n",getArg(buf,1));}
                else {
                    printf("%s - Turning On CTRL\n",getArg(buf,1));}
            }
        }
        else if (strcmp(buf0,"fet")==0) {
            int onOrOff = atof(getArg(buf,2));
            if(onOrOff == 0 | onOrOff == 1) {
                int jnt = name2mot(getArg(buf,1),&H_param);  // set motor num
                if(onOrOff==1)
                    hubo.fetOn( jnt, true );
                else if(onOrOff==0)
                    hubo.fetOff( jnt, true );
                if(onOrOff == 0) {
                    printf("%s - Turning Off FET\n",getArg(buf,1));}
                else {
                    printf("%s - Turning On FET\n",getArg(buf,1));}
            }
        }
        else if (strcmp(buf0,"initialize")==0) {
            int jnt = name2mot(getArg(buf,1),&H_param);	// set motor num
            hubo.initializeBoard( jnt, true );
            printf("%s - Initialize \n",getArg(buf,1));
        }
        else if (strcmp(buf0,"initializeAll")==0) {
            hubo.initializeAll(true);
            printf("%s - Initialize All\n",getArg(buf,1));
            tsleep = 2;
        }
        else if (strcmp(buf0,"zero")==0) {
            int ft = name2sensor(getArg(buf,1), &H_param);
            hubo.startSensor( (hubo_sensor_index_t)ft, true );
        }
        else if (strcmp(buf0,"zeracc")==0) {
            int ft = name2sensor(getArg(buf,1), &H_param);
            hubo.zeroTilt( (hubo_sensor_index_t)ft, true );
        }
        else if (strcmp(buf0,"iniSensors")==0){
            printf("Nulling All Sensors\n");
            hubo.startAllSensors( true );
        }
        /* Quit */
        else if (strcmp(buf0,"quit")==0)
            break;
        if (buf[0]!=0)
        add_history(buf);
        sleep(tsleep);	// sleep for tsleep sec
    }
    free(buf);
    return 0;
}


double hubo_set(char*s, struct hubo_param *p) {

    /* get joint number */
    int jointNo = name2mot(getArg(s,1),p);
    return jointNo;
}



char* getArg(string s, int argNum) {
    istringstream iss(s);

    int i = 0;
    do
    {
        string sub;
        iss >> sub;
        if( i == argNum ) {
                return (char*)sub.c_str(); }
        i++;
    } while (iss);

    return NULL;
}

int test(char* s, struct hubo_param *h) {
    printf("\n dan test\n");

    char* mot = getArg(s,1);

    int tmp = name2mot(mot, h);
    printf("mot = %i \n",tmp);
    return 0;
}

int name2mot(char* name, struct hubo_param *h) {
    /* Returns the number of the requested joint */
    int i = 0;
    int iout = -1;
    for( i = 0; i < HUBO_JOINT_COUNT ; i++ ) {
        char *mot = h->joint[i].name;
        if (strcmp(name, mot) == 0) {
        	iout = i;
		}
    }
	return iout;
}


int name2sensor(char* name, struct hubo_param *h) {
	/* Returns the number of the requested joint */
	int i = 0;
	int iout = -1;
	for( i = 0; i < HUBO_SENSOR_COUNT ; i++ ) {
	    char *sens = h->sensor[i].name;
	    printf("i = %i, name = %s\n", i,sens);
	    if (strcmp(name, sens) == 0) {
		iout = i;}
	}
	return iout;
}


static char** my_completion( const char * text , int start,  int end) {
    char **matches;

    matches = (char **)NULL;

    if (start == 0)
            matches = rl_completion_matches ((char*)text, &my_generator);
    else
            rl_bind_key('\t',rl_abort);

    return (matches);
}

char* my_generator(const char* text, int state) {
    static int list_index, len;
    char *name;

    if (!state) {
            list_index = 0;
            len = strlen (text);
    }

    while (name = cmd[list_index]) {
            list_index++;

    if (strncmp (name, text, len) == 0)
            return (dupstr(name));
    }

    /* If no names matched, then return NULL. */
    return ((char *)NULL);
}

char * dupstr (char* s) {
    char *r;

    r = (char*) xmalloc ((strlen (s) + 1));
    strcpy (r, s);
    return (r);
}

void * xmalloc (int size) {
    void *buf;

    buf = malloc (size);
    if (!buf) {
            fprintf (stderr, "Error: Out of memory. Exiting.'n");
            exit (1);
    }
    return buf;
}
