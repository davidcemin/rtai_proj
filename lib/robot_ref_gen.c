/******************************************************************************/
/**
 * \file robotRefGeneration.c
 * \brief Functions to generate the references xref and yref to be used in
 * control's thread.
 */
/******************************************************************************/

#include "robotDefines.h"
#include "robotStructs.h"

/******************************************************************************/

inline void robotRefGen(st_robotControl *robot, double t)
{	
	int k = robot->generation_t.k;
	if (t < 0.0) {
		robot->generation_t.ref[XREF_POSITION][k] = 0;
		robot->generation_t.ref[YREF_POSITION][k] = 0;
	}
	else if(t < 10.0)  {
		robot->generation_t.ref[XREF_POSITION][k] = ( (5/M_PI) * cos(0.2 * M_PI * t) ); 
		robot->generation_t.ref[YREF_POSITION][k] = ( (5/M_PI) * sin(0.2 * M_PI * t) ); 
	}
	else { 
		/*t >=10 */
		robot->generation_t.ref[XREF_POSITION][k] = ( (5/M_PI) * cos(0.2 * M_PI * t) ); 
		robot->generation_t.ref[YREF_POSITION][k] = ( -(5/M_PI) * sin(0.2 * M_PI * t) ); 
	}
	//printf("%f %f %f\n\r", t, robot->generation_t.ref[0], robot->generation_t.ref[1]);
}

