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
#include "monitorLin.h"

/******************************************************************************/

inline int monitorLinSetV(st_robotControlShared *shared, st_robotControl *local)
{
	int i;
	pthread_mutex_lock(&shared->mutex.mutexLin);

	for (i = 0; i < V_DIMENSION; i++)
		shared->control.lin_t.v[i] = local->lin_t.v[i];

	pthread_mutex_unlock(&shared->mutex.mutexLin);
	return 0;
}

/******************************************************************************/

inline int monitorLinGetV(st_robotControl *local, st_robotControlShared *shared)
{		
	int i;
	pthread_mutex_lock(&shared->mutex.mutexLin);

	for (i = 0; i < V_DIMENSION; i++)
		local->lin_t.v[i] = shared->control.lin_t.v[i];

	pthread_mutex_unlock(&shared->mutex.mutexLin);

	return 0;
}

/******************************************************************************/

inline int monitorLinGetY(st_robotControl *local, st_robotControlShared *shared)
{	
	int i;	
	pthread_mutex_lock(&shared->mutex.mutexLin);

	for (i = 0; i < Y_DIMENSION; i++)
		local->lin_t.y[i] = shared->control.lin_t.y[i];

	pthread_mutex_unlock(&shared->mutex.mutexLin);

	return 0;
}

/******************************************************************************/

inline int monitorLinSetY(st_robotControlShared *shared, st_robotControl *local)
{	
	int i;	
	pthread_mutex_lock(&shared->mutex.mutexLin);

	for (i = 0; i < Y_DIMENSION; i++)
		shared->control.lin_t.y[i] = local->lin_t.y[i];

	pthread_mutex_unlock(&shared->mutex.mutexLin);

	return 0;
}

/******************************************************************************/

