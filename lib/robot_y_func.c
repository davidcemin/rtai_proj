/******************************************************************************/
/**
 * \file robotYFunc.c
 * \brief Functions to calculate the output y
 */
/******************************************************************************/

#include "libRobot.h"
#include "robotDefines.h"
#include "robotStructs.h"

/******************************************************************************/

void robotCalcYFromX(st_robotMainArrays *robotMain)
{
	int k = robotMain->kIndex;

	/* y[0] = x[0] + RADIUS*cos(x[2]);
	 *  y[1] = x[1] + RADIUS*sin(x[2]); */
	robotMain->yVal[0][k] = robotMain->xVal[0][k] + RADIUS * cos(robotMain->xVal[2][k]);	
	robotMain->yVal[1][k] = robotMain->xVal[1][k] + RADIUS * sin(robotMain->xVal[2][k]);
}

/******************************************************************************/
