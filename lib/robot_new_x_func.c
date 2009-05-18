/******************************************************************************/
/**
 * \file robotNewXFunc.c
 * \brief  It calculates the new x value from x' and x
 */
/******************************************************************************/

#include "robotDefines.h"
#include "libRobot.h"

/******************************************************************************/

void robotNewX(st_robotMainArrays *robotMain)
{
	int i;
	double ts;
	int k = robotMain->kIndex;

	ts = (robotMain->timeInstant[k] - robotMain->timeInstant[k-1]);
	/* dx/dt = (x[n] - x[n-1])Ts*/
	for (i = 0; i < 3; i++){
		robotMain->xVal[i][k] = robotMain->dxVal[i][k - 1] * ts + robotMain->xVal[i][k - 1];
	}
}

/******************************************************************************/
