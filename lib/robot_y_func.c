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
	double x3 = robotMain->xVal[2][k];

	/* y[0] = x[0] + RADIUS*cos(x3);
	 *  y[1] = x[1] + RADIUS*sin(x3); */
	robotMain->yVal[0][k] = robotMain->xVal[0][k] + RADIUS * cos(x3);	
	robotMain->yVal[1][k] = robotMain->xVal[1][k] + RADIUS * sin(x3);
}

/******************************************************************************/
