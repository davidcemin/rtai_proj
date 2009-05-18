/******************************************************************************/
/**
 * \file monitorSim.c
 * \brief This file has the Simulation monitor functions
 *
 */
/******************************************************************************/

/* libc */
#include <pthread.h>

/* robot */
#include "monitorSim.h"
#include "simulCalcsUtils.h"

/******************************************************************************/

int monitorSimSet(st_robotMainArrays *robot, st_robotShared *shared)
{
	pthread_mutex_lock(&shared->mutex.mutexSim);

	/* Copy y values into shared memory */
	cpYIntoShared(robot, shared);

	pthread_mutex_unlock(&shared->mutex.mutexSim);
	return 0;
}

/******************************************************************************/

int monitorSimGet(st_robotShared *sharedCp, st_robotShared *shared)
{	
	pthread_mutex_lock(&shared->mutex.mutexCalc);

	memcpy(sharedCp, shared, sizeof(st_robotShared));

	pthread_mutex_unlock(&shared->mutex.mutexCalc);

	return 0;
}

/******************************************************************************/

