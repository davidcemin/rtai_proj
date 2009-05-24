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

inline int monitorLitGet(st_robotControl *local, st_robotControlShared *shared)
{		
	pthread_mutex_lock(&shared->mutex.mutexLin);

	memcpy(&local->lin_t, &shared->control.lin_t, sizeof(st_robotLin_t));

	pthread_mutex_unlock(&shared->mutex.mutexLin);

	return 0;
}

/******************************************************************************/

