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

void robotRefGen(st_robotControl *robot, double t)
{	
	if (t < 0.0) {
		robot->generation_t.xref = 0;
		robot->generation_t.yref = 0;
	}
	else if(t < 10.0)  {
		robot->generation_t.xref = ( (5/M_PI) * cos(0.2 * M_PI * t) ); 
		robot->generation_t.yref = ( (5/M_PI) * sin(0.2 * M_PI * t) ); 
	}
	else { 
		/*t >=10 */
		robot->generation_t.xref = ( (5/M_PI) * cos(0.2 * M_PI * t) ); 
		robot->generation_t.yref = ( 0 - (5/M_PI) * sin(0.2 * M_PI * t) ); 
	}
}

