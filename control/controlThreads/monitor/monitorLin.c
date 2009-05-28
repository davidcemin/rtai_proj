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

inline int monitorLinSet(st_robotControlShared *shared, st_robotControl *local)
{
	pthread_mutex_lock(&shared->mutex.mutexLin);

	memcpy(&shared->control.lin_t, &local->lin_t, sizeof(st_robotLin_t));

	pthread_mutex_unlock(&shared->mutex.mutexLin);
	return 0;
}

/******************************************************************************/

inline int monitorLinGet(st_robotControl *local, st_robotControlShared *shared)
{		
	pthread_mutex_lock(&shared->mutex.mutexLin);

	//TODO: verify this code
	memcpy(&local->lin_t, &shared->control.lin_t, sizeof(st_robotLin_t));

	pthread_mutex_unlock(&shared->mutex.mutexLin);

	return 0;
}

/******************************************************************************/

inline int monitorLinGetU(st_robotControl *local, st_robotControlShared *shared)
{	
	int i;	
	pthread_mutex_lock(&shared->mutex.mutexLin);

	for (i = 0; i < U_DIMENSION; i++)
		local->lin_t.u[i] = shared->control.lin_t.u[i];

	pthread_mutex_unlock(&shared->mutex.mutexLin);

	return 0;
}

/******************************************************************************/

inline int monitorLinSetU(st_robotControlShared *shared, st_robotControl *local)
{	
	int i;	
	pthread_mutex_lock(&shared->mutex.mutexLin);

	for (i = 0; i < U_DIMENSION; i++)
		shared->control.lin_t.u[i] = local->lin_t.u[i];

	pthread_mutex_unlock(&shared->mutex.mutexLin);

	return 0;
}

/******************************************************************************/

inline int monitorLinGetX(st_robotControl *local, st_robotControlShared *shared)
{	
	int i;	
	pthread_mutex_lock(&shared->mutex.mutexLin);

	for (i = 0; i < U_DIMENSION; i++)
		local->lin_t.x[i] = shared->control.lin_t.x[i];

	pthread_mutex_unlock(&shared->mutex.mutexLin);

	return 0;
}

/******************************************************************************/

inline int monitorLinSetX(st_robotControlShared *shared, st_robotControl *local)
{	
	int i;	
	pthread_mutex_lock(&shared->mutex.mutexLin);

	for (i = 0; i < U_DIMENSION; i++)
		shared->control.lin_t.x[i] = local->lin_t.x[i];

	pthread_mutex_unlock(&shared->mutex.mutexLin);

	return 0;
}


