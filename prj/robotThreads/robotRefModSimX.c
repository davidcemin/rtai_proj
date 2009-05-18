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
#include "monitorGen.h"
#include "robotRefModels.h"
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
static inline int taskCreateRtaiRefX(RT_TASK *task, unsigned long taskName, char priority, double stepTick) 
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
	
	msgSize = sizeof(st_robotShared);
	stkSize = msgSize + sizeof(st_robotMainArrays) + sizeof(st_robotSample) + 10000;

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

void *robotRefModSimX(void *ptr)
{	
	st_robotGenerationShared *shared = ptr;
	st_robotRefMod *refmod;
	st_robotGeneration *local;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;

	RT_TASK *simtask = NULL;
	unsigned long simtask_name = nam2num("REFMODX");

	if ( (refmod = (st_robotRefMod*)malloc(sizeof(st_robotRefMod))) == NULL ) {
		fprintf(stderr, "Error in refmod memory allocation!\n");
		return NULL;
	}

	if ( (local = (st_robotGeneration*)malloc(sizeof(st_robotGeneration))) == NULL ) {
		fprintf(stderr, "Error in generation local memory allocation!\n");
		return NULL;
	}

	memset(local, 0, sizeof(st_robotGeneration));
	memset(refmod, 0, sizeof(st_robotRefMod));

	if(taskCreateRtaiRefX(simtask, simtask_name, REFMODXPRIORITY, STEPTIMEREFMODELXNANO) < 0) {
		fprintf(stderr, "reference model X!\n");
		free(refmod);
		return NULL;
	}

	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;

		/* Monitor get ref*/
		monitorGeneration(shared, local, MONITOR_GET);

		/* Copy reference from local memory */
		refmod->ref[refmod->kIndex] = local->xref;

		/* Calculate ym */
		robotNewYm(refmod);

		/* Calculate ym' */
		robotDxYm(refmod);
		
		/* Copy ym and ym' into local shared */
		local->ym = refmod->ym[refmod->kIndex];
		local->dym = refmod->dRef[refmod->kIndex];
		
		/*monitor set ym and ym'*/
		monitorGeneration(shared, local, MONITOR_SET);

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

