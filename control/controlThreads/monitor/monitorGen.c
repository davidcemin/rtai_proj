/******************************************************************************/
/**
 * \file monitorGen.c
 * \brief This file has the Generation monitor functions
 *
 */
/******************************************************************************/

/* libc */
#include <pthread.h>

/* robot */
#include "robotDefines.h"
#include "monitorGen.h"

/******************************************************************************/

/**
 * \brief This function copy the local structure into shared
 * \param shared pointer to shared memory
 * \param local pointer to local memory
 * \return 0;
 */
inline int monitorGenSetX(st_robotControlShared *shared, st_robotControl *local)
{
	pthread_mutex_lock(&shared->mutex.mutexGen);

	memcpy(&shared->control.generation_t.xref, &local->generation_t.xref, sizeof(local->generation_t.xref));

	pthread_mutex_unlock(&shared->mutex.mutexGen);

	return 0;
}

/******************************************************************************/

/**
 * \brief This function copy the local structure into shared
 * \param shared pointer to shared memory
 * \param local pointer to local memory
 * \return 0;
 */
inline int monitorGenSetY(st_robotControlShared *shared, st_robotControl *local)
{
	pthread_mutex_lock(&shared->mutex.mutexGen);

	memcpy(&shared->control.generation_t.yref, &local->generation_t.yref, sizeof(local->generation_t.yref));

	pthread_mutex_unlock(&shared->mutex.mutexGen);

	return 0;
}

/******************************************************************************/

/**
 * \brief  Function that get the structure from shared
 * \param  local Pointer to st_robotGeneration structure
 * \param  shared Pointer to st_robotGenerationShared structure
 * \return 0
 */
inline int monitorGenGetX(st_robotControl *local, st_robotControlShared *shared)
{	
	pthread_mutex_lock(&shared->mutex.mutexGen);

	memcpy(&local->generation_t.xref, &shared->control.generation_t.xref, sizeof(local->generation_t.xref));

	pthread_mutex_unlock(&shared->mutex.mutexGen);

	return 0;
}

/******************************************************************************/

/**
 * \brief  Function that get the structure from shared
 * \param  local Pointer to st_robotGeneration structure
 * \param  shared Pointer to st_robotGenerationShared structure
 * \return 0
 */
inline int monitorGenGetY(st_robotControl *local, st_robotControlShared *shared)
{	
	pthread_mutex_lock(&shared->mutex.mutexGen);

	memcpy(&local->generation_t.yref, &shared->control.generation_t.yref, sizeof(local->generation_t.yref));

	pthread_mutex_unlock(&shared->mutex.mutexGen);

	return 0;
}

/******************************************************************************/
