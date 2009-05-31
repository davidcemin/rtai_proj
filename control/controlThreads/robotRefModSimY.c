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

/*rtai includes*/
#include <rtai_lxrt.h>

/*****************************************************************************/

void *robotRefModSimY(void *ptr)
{	
	st_robotControlStack *stack = ptr;
	st_robotControl *local;
	st_robotRefMod *refmod;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	RT_TASK *task = NULL;

	if ( (refmod = (st_robotRefMod*)malloc(sizeof(st_robotRefMod))) == NULL ) {
		fprintf(stderr, "Error in refmod memory allocation!\n");
		return NULL;
	}
	
	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}
	
	mlockall(MCL_CURRENT | MCL_FUTURE);	

	/*init some pointers..*/
	memset(local, 0, sizeof(local));
	memset(refmod, 0, sizeof(st_robotRefMod));

	/*registering real time task*/
	if((task = rt_thread_init(nam2num(REFMODY), REFMODYPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"refY task init error!\n\r");
		return NULL;
	}

	printf("Y: waiting control\n\r");
	/*wait sync*/
	rt_sem_wait(stack->sem.sm_refy);
	
	/*make it hard real time*/
	rt_make_hard_real_time();
	
	/*and finally make it periodic*/
	rt_task_make_periodic(task, nano2count(stack->time), TIMEMODY*stack->tick);

	printf("REFMODY\n\r");
	do {
		currentT = rt_get_time_ns() - stack->time;
		refmod->kIndex++;
		
#if 0	
		/*get alpha*/
		local->alpha[YREF_POSITION] = 1;

		/* Monitor get ref*/
		monitorControlMain(local, MONITOR_GET_REFERENCE_Y);

		/* Copy reference from local memory */
		refmod->ref[refmod->kIndex] = local->generation_t.ref[YREF_POSITION];

		/* Calculate ym' */
		//robotDxYm(refmod, local, YREF_POSITION);

		/* Calculate ym */
		//robotNewYm(refmod);

		/* Copy ym and ym' into local shared */
		local->control_t.ym[YM_POSITION] = refmod->ym[refmod->kIndex];
		local->control_t.dym[YM_POSITION] = refmod->dRef[refmod->kIndex];
		
		/*monitor set ym and ym'*/
		//monitorControlMain(local, MONITOR_SET_YMY);

		refmod->timeInstant[refmod->kIndex] = currentT / SEC2NANO(1);	
#endif
		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);

		rt_task_wait_period();
	} while ( (fabs(total) <= (double)TOTAL_TIME) );

	munlockall();
	printf("Y: waiting control signal\n\r");
	rt_sem_wait(stack->sem.sm_refy);
	printf("finish ya\n\r");
	rt_make_soft_real_time();
	rt_task_delete(task);
	printf("finish yb\n\r");
	return NULL;
}

