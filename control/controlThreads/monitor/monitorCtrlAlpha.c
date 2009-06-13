/******************************************************************************/
/**
 * \file monitorControl.c
 * \brief This file has the Control monitor functions
 *
 */
/******************************************************************************/

/* libc */
#include <pthread.h>

/* robot */
#include "monitorCtrlAlpha.h"

/******************************************************************************/

inline int monitorGetAlpha(st_robotControlShared *shared, st_robotControl *local)
{
	int i;
	
	pthread_mutex_lock(&shared->mutex.mutexAlpha);

	for (i = 0; i < ALPHA_DIMENSION; i++)
		local->alpha[i] = shared->control.alpha[i];

	pthread_mutex_unlock(&shared->mutex.mutexAlpha);

	return 0;
}

/******************************************************************************/

inline int monitorSetAlpha(st_robotControl *local, st_robotControlShared *shared)
{		
	int i;
	
	pthread_mutex_lock(&shared->mutex.mutexAlpha);
	
	for (i = 0; i < ALPHA_DIMENSION; i++)
		shared->control.alpha[i] = local->alpha[i];

	pthread_mutex_unlock(&shared->mutex.mutexAlpha);

	return 0;
}

/******************************************************************************/

