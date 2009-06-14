/******************************************************************************/
/**
 * \file monitorMain.c
 * \brief This file has the Main monitor functions
 *
 */
/******************************************************************************/

/* libc */
#include <stdio.h>
#include <semaphore.h>

/* robot */
#include "monitorSim.h"
#include "monitorSimMain.h"
#include "robotStructs.h"

/******************************************************************************/

//! Shared memory. Static global variable accessed via monitor
static st_robotSimulShared Shared;

/******************************************************************************/

void robotSimSharedInit(void)
{
	pthread_mutex_init(&Shared.mutexSim, NULL);
	memset(&Shared.simul_t, 0, sizeof(Shared.simul_t));

	Shared.simul_t.x[0] = 0;
	Shared.simul_t.x[1] = 1;
	Shared.simul_t.x[0] = 0;
}

/******************************************************************************/

void robotSimSharedFinish(void)
{
	pthread_mutex_destroy(&Shared.mutexSim);
}

/******************************************************************************/

inline void monitorSimMain(st_robotSimulPacket *local, int type)
{
	
	switch (type) {
		case MONITOR_GET_SIM_SHARED:
			monitorSimGet(local, &Shared);
			break;

		case MONITOR_SET_SIM_SHARED:
			monitorSimSet(&Shared, local);
			break;

		default:
			fprintf(stderr, "Error in monitor Main. I should not be here!\n\r");
			break;
	}

}

