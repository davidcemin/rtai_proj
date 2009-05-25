/*****************************************************************************/
/**
 * \file robotRefModSimX.c
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
#include "simulCalcsUtils.h"
#include "rtaiCtrl.h"

/*rtai includes*/
#include <rtai_lxrt.h>

/*****************************************************************************/

void *robotRefModSimX(void *ptr)
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
	

	if(taskCreateRtai(simtask, REFMODX, REFMODXPRIORITY, STEPTIMEREFMODELXNANO, sizeof(st_robotControlShared)) < 0) {
		fprintf(stderr, "reference model X!\n");
		free(refmod);
		return NULL;
	}
	
	rt_sem_wait(shared->sem.sm_refx);
	rt_sem_signal(shared->sem.sm_control);

	memset(local, 0, sizeof(local));
	memset(refmod, 0, sizeof(refmod));


	printf("REFMODX\n\r");
	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;

		/* Monitor get ref*/
		monitorControlMain(shared, local, MONITOR_GET_REFERENCE_X);

		/* Copy reference from local memory */
		refmod->ref[refmod->kIndex] = local->generation_t.xref;

		/* Calculate ym */
		robotNewYm(refmod);

		/* Calculate ym' */
		robotDxYm(refmod);
		
		/* Copy ym and ym' into local shared */
		local->control_t.ym[XM_POSITION] = refmod->ym[refmod->kIndex];
		local->control_t.dym[XM_POSITION] = refmod->dRef[refmod->kIndex];
		
		/*monitor set ym and ym'*/
		monitorControlMain(shared, local, MONITOR_SET_YMX);

		rt_task_wait_period();
		refmod->kIndex++;
		refmod->timeInstant[refmod->kIndex] = currentT / SEC2NANO(1);	
		
		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);
	} while ( (fabs(total) <= (double)TOTAL_TIME) );

	printf("x1\n\r");
	taskFinishRtai(simtask);
	printf("x2\n\r");
	free(refmod);
	printf("x3\n\r");
	return NULL;
}

