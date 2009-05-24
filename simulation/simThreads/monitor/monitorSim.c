/******************************************************************************/
/**
 * \file monitorSim.c
 * \brief This file has the Simulation monitor functions
 *
 */
/******************************************************************************/

/* libc */
#include <pthread.h>

/* robot */
#include "monitorSim.h"
#include "simulCalcsUtils.h"

/******************************************************************************/

inline int monitorSimGet(st_robotSimulPacket *packet, st_robotSimulShared *shared)
{	
	pthread_mutex_lock(&shared->mutexSim);

	memcpy(packet, &shared->simul_t, sizeof(st_robotSimulPacket));

	pthread_mutex_unlock(&shared->mutexSim);

	return 0;
}


/******************************************************************************/

inline int monitorSimSet(st_robotSimulShared *shared, st_robotSimulPacket *packet)
{
	pthread_mutex_lock(&shared->mutexSim);

	memcpy(&shared->simul_t, packet, sizeof(st_robotSimulPacket));

	pthread_mutex_unlock(&shared->mutexSim);
	return 0;
}

/******************************************************************************/


