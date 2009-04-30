/******************************************************************************/
/**
 * \file robotYFunc.c
 * \brief Functions to calculate the output y
 */
/******************************************************************************/

#include "robotDefines.h"
#include "libRobot.h"

/******************************************************************************/

void robotCalcYFromX(st_robotMainArrays *robotMain)
{
	int k = robotMain->kIndex;

	/* y[0] = x[0] + *0.5*D*cos(x[2]);
	 *  y[1] = x[1]; y[2] = x[2] */
	robotMain->yVal[0][k] = robotMain->xVal[0][k] + 0.5 * DIST * cos(robotMain->xVal[2][k]);	
	robotMain->yVal[1][k] = robotMain->xVal[1][k];	
	robotMain->yVal[2][k] = robotMain->xVal[2][k];	
}
/******************************************************************************/


