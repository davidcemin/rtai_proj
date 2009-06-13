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
	pthread_mutex_lock(&shared->mutex.mutexRefX);

	shared->control.generation_t.ref[XREF_POSITION] = local->generation_t.ref[XREF_POSITION];

	pthread_mutex_unlock(&shared->mutex.mutexRefX);

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
	pthread_mutex_lock(&shared->mutex.mutexRefY);
	
	shared->control.generation_t.ref[YREF_POSITION] = local->generation_t.ref[YREF_POSITION];

	pthread_mutex_unlock(&shared->mutex.mutexRefY);

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
	pthread_mutex_lock(&shared->mutex.mutexRefX);

	local->generation_t.ref[XREF_POSITION] = shared->control.generation_t.ref[XREF_POSITION];

	pthread_mutex_unlock(&shared->mutex.mutexRefX);

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
	pthread_mutex_lock(&shared->mutex.mutexRefY);

	local->generation_t.ref[YREF_POSITION] = shared->control.generation_t.ref[YREF_POSITION];

	pthread_mutex_unlock(&shared->mutex.mutexRefY);
	return 0;
}

/******************************************************************************/
