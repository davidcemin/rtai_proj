/******************************************************************************/
/**
 * \file monitorDisp.c
 * \brief This file has the Display monitor functions
 *
 */
/******************************************************************************/

#include <pthread.h>

#include "monitorDisp.h"
#include "simulCalcsUtils.h"

/******************************************************************************/

int monitorDispGet(st_robotShared *shared, double t)
{	
	pthread_mutex_lock(&shared->mutex.mutexSim);
	
	printDisplay(shared, t);
	
	pthread_mutex_unlock(&shared->mutex.mutexSim);

	return 0;
}
