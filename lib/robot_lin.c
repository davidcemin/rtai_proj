/******************************************************************************/
/**
 * \file robot_lin.c
 * \brief Functions used in control linearization.
 */
/******************************************************************************/

/* libc */
#include <math.h>

/* robot */
#include "robotDefines.h"
#include "libRobot.h"

/******************************************************************************/

void robotGenU(st_robotLinPacket *linPacket)
{	
	double v1 = linPacket->v[0];
	double v2 = linPacket->v[1];
	double x3 = linPacket->x[2];

	linPacket->u[0] = ( (cos(x3) * v1) + (sin(x3) * v2) );
	linPacket->u[1] = ( (1/RADIUS) * ( (cos(x3) * v2) - (sin(x3) * v1) ) );
}

/******************************************************************************/

