/*****************************************************************************/
/**
 * \file robotRefModSimY.c
 * \brief Thread used in reference model X simulation block
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
#include "robotRefModels.h"
#include "robotStructs.h"
#include "robotThreads.h"
#include "rtaiCtrl.h"
#include "simulCalcsUtils.h"

/*rtai includes*/
#include <rtai_lxrt.h>

/*****************************************************************************/

void *robotRefModSimY(void *ptr)
{	
	st_robotControlShared *shared = ptr;
	st_robotControl *local;
	st_robotRefMod *refmod;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	RT_TASK *simtask = NULL;

	if ( (refmod = (st_robotRefMod*)malloc(sizeof(st_robotRefMod))) == NULL ) {
		fprintf(stderr, "Error in refmod memory allocation!\n");
		return NULL;
	}
	
	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}
	

	if(taskCreateRtai(simtask, REFMODY, REFMODYPRIORITY, STEPTIMEREFMODELYNANO, sizeof(st_robotControlShared) < 0)) {
		fprintf(stderr, "Reference model Y\n");
		free(refmod);
		return NULL;
	}

	rt_sem_wait(shared->sem.sm_refy);
	rt_sem_signal(shared->sem.sm_control);

	memset(local, 0, sizeof(local));
	memset(refmod, 0, sizeof(st_robotRefMod));

	printf("REFMODY\n\r");
	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;
		
		/* Monitor get ref*/
		monitorControlMain(shared, local, MONITOR_GET_REFERENCE_Y);

		/* Copy reference from local memory */
		refmod->ref[refmod->kIndex] = local->generation_t.yref;

		/* Calculate ym */
		robotNewYm(refmod);

		/* Calculate ym' */
		robotDxYm(refmod);
		
		/* Copy ym and ym' into local shared */
		local->control_t.ym[YM_POSITION] = refmod->ym[refmod->kIndex];
		local->control_t.dym[YM_POSITION] = refmod->dRef[refmod->kIndex];
		
		/*monitor set ym and ym'*/
		monitorControlMain(shared, local, MONITOR_SET_YMY);

		rt_task_wait_period();
		refmod->kIndex++;
		refmod->timeInstant[refmod->kIndex] = currentT / SEC2NANO(1);	
		
		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);
	} while ( (fabs(total) <= (double)TOTAL_TIME) );

	taskFinishRtai(simtask);
	free(refmod);
	return NULL;
}

