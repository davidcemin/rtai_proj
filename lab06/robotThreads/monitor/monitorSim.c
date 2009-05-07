/******************************************************************************/
/**
 * \file monitorSim.c
 * \brief This file has the Simulation monitor functions
 *
 */
/******************************************************************************/

#include <pthread.h>

#include "monitorSim.h"
#include "simulCalcsUtils.h"

int monitorSimSet(st_robotMainArrays *robot, st_robotShared *shared)
{
	pthread_mutex_lock(&shared->mutexSim);

	/* Copy y values into shared memory */
	cpYIntoShared(robot, shared);

	pthread_mutex_unlock(&shared->mutexSim);
	return 0;
}


int monitorSimGet(st_robotMainArrays *robot, st_robotShared *shared)
{	
	pthread_mutex_lock(&shared->mutexCalc);

	/* Get u values from shared */
	getUFromShared(robot, shared);

	pthread_mutex_unlock(&shared->mutexCalc);

	return 0;
}


