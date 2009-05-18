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
static int monitorGenSet(st_robotGenerationShared *shared, st_robotGeneration *local)
{
	pthread_mutex_lock(&shared->mutexGen);

	memcpy(&shared->genShared, local, sizeof(st_robotGeneration));

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
static int monitorGenGet(st_robotGeneration *local, st_robotGenerationShared *shared)
{	
	pthread_mutex_lock(&shared->mutexGen);

	memcpy(local, &shared->genShared, sizeof(st_robotGeneration));

	pthread_mutex_unlock(&shared->mutexGen);

	return 0;
}

/******************************************************************************/

int monitorGeneration(st_robotGenerationShared *shared, st_robotGeneration *local, int type)
{
	
	switch(type) {
		case MONITOR_GET:
			monitorGenGet(local, shared);
			break;

		case MONITOR_SET:
			monitorGenSet(shared, local);
			break;

		default:
			fprintf(stderr, "Monitor generation: I should not be here\n");
			return -1;			
	}

	return 0;
}
