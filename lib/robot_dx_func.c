/******************************************************************************/
/**
 * \file robotDxFunc.c
 * \brief This file has the x' functions
 */
/******************************************************************************/

#include <math.h>

#include "robotDefines.h"
#include "libRobot.h"

/******************************************************************************/

void robotDxSim(st_robotMainArrays *robotMain)
{
	int k = robotMain->kIndex;
	/* x1' = sin(x3) * v */
	robotMain->dxVal[0][k] = sin(robotMain->xVal[2][k]) * robotMain->uVal[0][k];
	/* x2' = cos(x3) * v; */
	robotMain->dxVal[1][k] = cos(robotMain->xVal[2][k]) * robotMain->uVal[0][k];
	/* x3' = w; */
	robotMain->dxVal[2][k] = robotMain->uVal[1][k];
}
/******************************************************************************/

