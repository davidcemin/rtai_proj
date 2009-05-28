/******************************************************************************/
/**
 * \file monitorRefModels.c
 * \brief This file has the reference models monitor functions
 *
 */
/******************************************************************************/

/* libc */
#include <pthread.h>

/* robot */
#include "robotDefines.h"
#include "robotStructs.h"
#include "monitorControlMain.h"
#include "monitorRefModels.h"

/******************************************************************************/

inline int monitorRefXSet(st_robotControlShared *shared, st_robotControl *local)
{
	pthread_mutex_lock(&shared->mutex.mutexControl);

	shared->control.control_t.ym[XM_POSITION] = local->control_t.ym[XM_POSITION];
	shared->control.control_t.dym[XM_POSITION] = local->control_t.dym[XM_POSITION];

	pthread_mutex_unlock(&shared->mutex.mutexControl);

	return 0;
}

/******************************************************************************/

inline int monitorRefYSet(st_robotControlShared *shared, st_robotControl *local)
{
	pthread_mutex_lock(&shared->mutex.mutexControl);

	shared->control.control_t.ym[YM_POSITION] = local->control_t.ym[YM_POSITION];
	shared->control.control_t.dym[YM_POSITION] = local->control_t.dym[YM_POSITION];

	pthread_mutex_unlock(&shared->mutex.mutexControl);

	return 0;
}

/******************************************************************************/

inline int monitorRefXGet(st_robotControl *local, st_robotControlShared *shared)
{	
	pthread_mutex_lock(&shared->mutex.mutexControl);

	local->control_t.ym[XM_POSITION] = shared->control.control_t.ym[XM_POSITION];
	local->control_t.dym[XM_POSITION] = shared->control.control_t.dym[XM_POSITION]; 

	pthread_mutex_unlock(&shared->mutex.mutexControl);

	return 0;
}

/******************************************************************************/

inline int monitorRefYGet(st_robotControl *local, st_robotControlShared *shared)
{
	pthread_mutex_lock(&shared->mutex.mutexControl);

	local->control_t.ym[YM_POSITION] = shared->control.control_t.ym[YM_POSITION];
	local->control_t.dym[YM_POSITION] = shared->control.control_t.dym[YM_POSITION]; 

	pthread_mutex_unlock(&shared->mutex.mutexControl);

	return 0;
}

/******************************************************************************/

