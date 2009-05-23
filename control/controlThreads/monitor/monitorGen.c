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
inline int monitorGenSet(st_robotControlShared *shared, st_robotControl *local)
{
	pthread_mutex_lock(&shared->mutexGen);

	memcpy(&shared->control.generation_t, &local->generation_t, sizeof(st_robotGeneration_t));

	pthread_mutex_unlock(&shared->mutexGen);

	return 0;
}

/******************************************************************************/

/**
 * \brief  Function that get the structure from shared
 * \param  local Pointer to st_robotGeneration structure
 * \param  shared Pointer to st_robotGenerationShared structure
 * \return 0
 */
inline int monitorGenGet(st_robotControl *local, st_robotControlShared *shared)
{	
	pthread_mutex_lock(&shared->mutexGen);

	memcpy(&local->generation_t, &shared->control.generation_t, sizeof(st_robotGeneration_t));

	pthread_mutex_unlock(&shared->mutexGen);

	return 0;
}

/******************************************************************************/
