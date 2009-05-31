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

static st_robotSimulShared Shared;

/******************************************************************************/

void robotSimSharedInit(void)
{
	pthread_mutex_init(&Shared.mutexSim, NULL);
	sem_init(&Shared.sm_disp, 0, 0);
	memset(&Shared.simul_t, 0, sizeof(Shared.simul_t));
}

/******************************************************************************/

void robotSimSharedFinish(void)
{
	pthread_mutex_destroy(&Shared.mutexSim);
	sem_destroy(&Shared.sm_disp);
}

/******************************************************************************/

inline void monitorSimMain(st_robotSimulPacket *local, int type)
{
	
	switch (type) {
		case MONITOR_GET_SIM_SHARED:
			while (sem_wait(&Shared.sm_disp) < 0)
				break;
			monitorSimGet(local, &Shared);
			break;

		case MONITOR_SET_SIM_SHARED:
			monitorSimSet(&Shared, local);
			sem_post(&Shared.sm_disp);
			break;

		default:
			fprintf(stderr, "Error in monitor Main. I should not be here!\n\r");
			break;
	}

}

