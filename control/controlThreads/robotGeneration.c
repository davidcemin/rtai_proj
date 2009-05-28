/*****************************************************************************/
/**
 * \file robotGeneration.c
 * \brief Generation Thread and auxiliary functions
 */
/*****************************************************************************/

/*libc Includes*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <semaphore.h>
#include <string.h> 
#include <sys/time.h> 

/*robot includes*/
#include "libRobot.h"
#include "monitorControlMain.h"
#include "robotStructs.h"
#include "robotThreads.h"
#include "simulCalcsUtils.h"
#include "robotGeneration.h"
#include "rtaiCtrl.h"

/*rtai includes*/
#include <rtai_lxrt.h>
#include <rtai_sem.h>

/*****************************************************************************/

void *robotGeneration(void *ptr)
{
	st_robotControlShared *shared = (st_robotControlShared*)ptr;
	st_robotControl *local;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	RT_TASK *gentask = NULL;
	int started_timer = 0;
	
	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}

	if( (started_timer = taskCreateRtai(gentask, GENTASK, GENPRIORITY, STEPTIMEGENERNANO, sizeof(st_robotControlShared)) ) < 0){
		fprintf(stderr, "Generation!\n");
		return NULL;
	}
	mkTaksRealTime(gentask, STEPTIMEGENERNANO, GENTASK);

	rt_sem_signal(shared->sem.sm_refx);
	printf("release x..\n\r");
	rt_sem_signal(shared->sem.sm_refy);
	printf("reelase y...");

	memset(local, 0, sizeof(local));

	rt_sem_wait(shared->sem.sm_gen);
	printf("GENERATION\n");
	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;

		/*Algorithm:
		 * 1) Generate xref and yref;
		 * 2) Put it inside shared memory
		 */
		robotRefGen(local, total);
		//printf("%f %f\n\r", local->generation_t.ref[0], local->generation_t.ref[1]);

		/*Set xref and yref into shared memory*/
		monitorControlMain(shared, local, MONITOR_SET_REFERENCE_X);
		monitorControlMain(shared, local, MONITOR_SET_REFERENCE_Y);

		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period();
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	printf("END GEN\n\r");
	taskFinishRtai(gentask, started_timer);
	return NULL;
}
