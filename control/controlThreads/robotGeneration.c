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

/*****************************************************************************/

void *robotGeneration(void *ptr)
{
	st_robotControlShared *shared = ptr;
	st_robotControl *local;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	RT_TASK *gentask = NULL;
	
	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}

	if(taskCreateRtai(gentask, GENTASK, GENPRIORITY, STEPTIMEGENERNANO, sizeof(st_robotControlShared)) < 0){
		fprintf(stderr, "Generation!\n");
		return NULL;
	}	
	
	rt_sem_signal(shared->sem.sm_refx);
	rt_sem_signal(shared->sem.sm_refy);

	memset(local, 0, sizeof(local));

	printf("GENERATION\n");
	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;

		/*Algorithm:
		 * 1) Generate xref and yref;
		 * 2) Put it inside shared memory
		 */

		robotRefGen(local, total);

		/*Set xref and yref into shared memory*/
		monitorControlMain(shared, local, MONITOR_SET_REFERENCE_X);
		monitorControlMain(shared, local, MONITOR_SET_REFERENCE_Y);

		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period();

	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	printf("g1\n\r");
	taskFinishRtai(gentask);
	printf("g2\n\r");
	free(local);
	return NULL;
}
