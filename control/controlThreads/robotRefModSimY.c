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
#include "simulCalcsUtils.h"

/*rtai includes*/
#include <rtai_lxrt.h>

/*****************************************************************************/

/**
 * \brief  It creates a rtai task
 * \param  task Pointer to task 
 * \param  taskName Task's name
 * \param  priority Task's priority
 * \param  stepTick Step timer
 * \return -1 error, 0 ok.
 */
static inline int taskCreateRtaiRefY(RT_TASK *task, unsigned long taskName, char priority, double stepTick) 
{
	int period;
	int stkSize;
	int msgSize;

	/*set root permissions to user space*/
	rt_allow_nonroot_hrt();

	/*Es ist nicht noetig um die priority im sched_param structure anzusetzen, da 
	 * es bereits im rt_task_init_schmod Function angesetzt ist.
	 */
	/*It Prevents the memory to be paged*/
    mlockall(MCL_CURRENT | MCL_FUTURE);
	
	msgSize = sizeof(st_robotControlShared);
	stkSize = msgSize + 10000;

	if(!(task = rt_task_init_schmod(taskName, priority, stkSize, msgSize, SCHED_FIFO, 0xff) ) ) {	
		fprintf(stderr, "Cannot Init Task: ");
		return -1;
	}

	/*make it hard real time*/	
    rt_make_hard_real_time();

	/*set the period according to our desired tick*/
	period = (int)nano2count((RTIME)stepTick);

	/*start the task*/
	rt_task_make_periodic(task, rt_get_time()+period, period);

	return 0;
}

/*****************************************************************************/

/**
 * \brief  It destroys a rtai task
 * \param  task pointer to task to be destroyed
 * \return void
 */
static inline void taskFinishRtaiRefSim(RT_TASK *task)
{
	/* Here also is not necessary to use rt_make_soft_real_time because rt_task_delete 
	 * already use it(base/include/rtai_lxrt.h:842)
	 */
	rt_task_delete(task);
}

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
	unsigned long simtask_name = nam2num("REFMODY");

	if ( (refmod = (st_robotRefMod*)malloc(sizeof(st_robotRefMod))) == NULL ) {
		fprintf(stderr, "Error in refmod memory allocation!\n");
		return NULL;
	}
	
	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}

	memset(local, 0, sizeof(local));
	memset(refmod, 0, sizeof(st_robotRefMod));

	if(taskCreateRtaiRefY(simtask, simtask_name, REFMODYPRIORITY, STEPTIMEREFMODELYNANO) < 0) {
		fprintf(stderr, "Reference model Y\n");
		free(refmod);
		return NULL;
	}

	printf("REFMODY\n\r");
	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;
		
		/* Monitor get ref*/
		monitorControlMain(shared, local, MONITOR_GET_REFERENCE);

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

	taskFinishRtaiRefSim(simtask);
	free(refmod);
	return NULL;
}

