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
	st_robotControlShared *shared = (st_robotControlShared*)ptr;
	st_robotControl *local;
	st_robotRefMod *refmod;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	RT_TASK *simtask = NULL;
	int started_timer = 0;
	int stack = sizeof(st_robotControlShared);

	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}

	if ( (refmod = (st_robotRefMod*)malloc(sizeof(refmod))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}

	
	/*init task*/
	if( (started_timer = taskCreateRtai(simtask, REFMODX, REFMODXPRIORITY, STEPTIMEREFMODELXNANO, stack) ) < 0) {
		fprintf(stderr, "reference model X!\n");
		free(refmod);
		return NULL;
	}
	
	/*wait sync*/
	rt_sem_wait(shared->sem.sm_refx);
	rt_sem_signal(shared->sem.sm_control);

	/*make it real time*/
	mkTaksRealTime(simtask, STEPTIMEGENERNANO, REFMODX);

	/*init some pointers*/
	memset(local, 0, sizeof(local));
	memset(refmod, 0, sizeof(refmod));

	rt_sem_wait(shared->sem.sm_refx);
	printf("REFMODX\n\r");
	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;
		refmod->kIndex++;

		/*get alpha*/
		local->alpha[XREF_POSITION] = 1;

		/* Monitor get ref*/
		monitorControlMain(shared, local, MONITOR_GET_REFERENCE_X);
		
		/* Copy reference from local memory */
		refmod->ref[refmod->kIndex] = local->generation_t.ref[XREF_POSITION];
	
		/* Calculate ym' */
		//robotDxYm(refmod, local, XREF_POSITION);

		/* Calculate ym */
		//robotNewYm(refmod);
				
		/* Copy ym and ym' into local shared */
		local->control_t.ym[XM_POSITION] = refmod->ym[refmod->kIndex];
		local->control_t.dym[XM_POSITION] = refmod->dRef[refmod->kIndex];
		
		/*monitor set ym and ym'*/
		//monitorControlMain(shared, local, MONITOR_SET_YMX);

		refmod->timeInstant[refmod->kIndex] = currentT / SEC2NANO(1);	
		
		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);

		rt_task_wait_period();
	} while ( (fabs(total) <= (double)TOTAL_TIME) );

	printf("finish x\n\r");
	taskFinishRtai(simtask, started_timer);
	return NULL;
}

