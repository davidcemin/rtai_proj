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
#include "rtaiCtrl.h"

/*rtai includes*/
#include <rtai_lxrt.h>

/*****************************************************************************/

void *robotRefModSimX(void *ptr)
{	
	st_robotControlStack *stack = ptr;
	st_robotControl *local;
	st_robotRefMod *refmod;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	RT_TASK *task = NULL;

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
	
	mlockall(MCL_CURRENT | MCL_FUTURE);	
	/*init some pointers*/
	memset(local, 0, sizeof(local));
	memset(refmod, 0, sizeof(refmod));
	
	/*registering real time task*/
	if((task = rt_thread_init(nam2num(REFMODX), REFMODXPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"refX task init error!\n\r");
		return NULL;
	}

	/*wait sync*/
	printf("X: waiting control\n\r");
	rt_sem_wait(stack->sem.sm_refx);

	/*make it hard real time*/
	rt_make_hard_real_time();
	
	/*and finally make it periodic*/
	rt_task_make_periodic(task, nano2count(stack->time), TIMEMODX*stack->tick);

	printf("REFMODX\n\r");
	do {
		currentT = rt_get_time_ns() - stack->time;
		refmod->kIndex++;
		
		/*get alpha*/
		local->alpha[XREF_POSITION] = 1;

		/* Monitor get ref*/
		monitorControlMain(local, MONITOR_GET_REFERENCE_X);
		
		/* Copy reference from local memory */
		refmod->ref[refmod->kIndex] = local->generation_t.ref[XREF_POSITION];
	
		/* Calculate ym' */
		robotDxYm(refmod, local, XREF_POSITION);

		/* Calculate ym */
		robotNewYm(refmod);
				
		/* Copy ym and ym' into local shared */
		local->control_t.ym[XM_POSITION] = refmod->ym[refmod->kIndex];
		local->control_t.dym[XM_POSITION] = refmod->dRef[refmod->kIndex];
		
		/*monitor set ym and ym'*/
		monitorControlMain(local, MONITOR_SET_YMX);

		refmod->timeInstant[refmod->kIndex] = currentT / SEC2NANO(1);	
		
		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);

		rt_task_wait_period();
	} while ( (fabs(total) <= (double)TOTAL_TIME) );

	printf("X: waiting control signal\n\r");
	rt_sem_wait(stack->sem.sm_refx);
	
	rt_make_soft_real_time();
	rt_task_delete(task);

	return NULL;
}

