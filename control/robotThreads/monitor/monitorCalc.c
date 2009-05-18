/******************************************************************************/
/**
 * \file monitorCalc.c
 * \brief This file has the Control monitor functions
 *
 */
/******************************************************************************/

/* libc */
#include <pthread.h>

/* robot */
#include "monitorCalc.h"
#include "simulCalcsUtils.h"

/******************************************************************************/

int monitorCalcSet(st_robotShared *shared, double t)
{
	pthread_mutex_lock(&shared->mutex.mutexCalc);

	/* Calculates the inputs: u[n] */
	robotInputCalc(shared, t);

	pthread_mutex_unlock(&shared->mutex.mutexCalc);
	return 0;
}

/******************************************************************************/

int monitorCalcGet(st_robotSample *sample, st_robotShared *shared, double t)
{	
	pthread_mutex_lock(&shared->mutex.mutexSim);
		
	/* Sample y and copy it into buffer */
	robotSampleYf(shared, sample, t);

	pthread_mutex_unlock(&shared->mutex.mutexSim);

	return 0;
}

/******************************************************************************/
