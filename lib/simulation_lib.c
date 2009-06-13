/*****************************************************************************/
/**
 * \file  simulation_lib.c
 * \brief functions used in simulation thread
 */
/*****************************************************************************/

/*robot includes*/
#include "libRobot.h"
#include "robotStructs.h"

/******************************************************************************/

inline void robot_sim_calc_dx(st_robotSimulPacket *pack, int k)
{
	double x3 = pack->x[2];
	double v = pack->u[0];
	double w = pack->u[1];

	pack->xd[0] = cos(x3) * v;
	pack->xd[1] = sin(x3) * v;
	pack->xd[2] = w;
	//printf("%d %f %f %f\n\r", k, robotMain->dxVal[0][k], robotMain->dxVal[1][k], robotMain->dxVal[2][k]);
}

/******************************************************************************/

inline void robot_sim_calc_x(st_robotSimulPacket *pack, int k, double ts)
{
	int i;

	for (i = 0; i < 3; i++)
		pack->x[i] = pack->xd[i] * ts + pack->xl[i];
	//printf("%d %f %f %f\n\r", k, robotMain->xVal[0][k],robotMain->xVal[1][k],robotMain->xVal[2][k] );
}

/******************************************************************************/

inline void robot_sim_calc_y(st_robotSimulPacket *pack, int k)
{
	double x1 = pack->x[0];
	double x2 = pack->x[1];
	double x3 = pack->x[2];

	/* y[0] = x[0] + RADIUS*cos(x3);
	 *  y[1] = x[1] + RADIUS*sin(x3); */
	pack->y[0] = x1 + RADIUS * cos(x3);	
	pack->y[1] = x2 + RADIUS * sin(x3);
}

/******************************************************************************/
