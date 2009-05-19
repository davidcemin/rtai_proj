/*****************************************************************************/
/**
 * \file robotSimulation.c
 * \brief Thread used in simulation's block
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
#include "simThreads.h"

/*rtai includes*/
#include <rtai_netrpc.h>
#include <rtai_lxrt.h>

//! Defined used in jitter calculations and the like.
#define CALC_DATA

/*****************************************************************************/

/**
 * \brief  It creates a rtai task
 * \param  task Pointer to task 
 * \param  taskName Task's name
 * \param  priority Task's priority
 * \param  stepTick Step timer
 * \return -1 error, 0 ok.
 */
static inline int taskCreateRtaiSim(RT_TASK *task, unsigned long taskName, char priority, double stepTick) 
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
static inline void taskFinishRtaiSim(RT_TASK *task)
{
	/* Here also is not necessary to use rt_make_soft_real_time because rt_task_delete 
	 * already use it(base/include/rtai_lxrt.h:842)
	 */
	rt_task_delete(task);
}

/*****************************************************************************/

void *robotSimulation(void *ptr)
{	
	st_robotShared *shared = ptr;
	st_robotShared *sharedCp;
	st_robotMainArrays *robot;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	unsigned long node = rt_set_this_node("127.0.0.1", 0, 1);

	RT_TASK *simtask = NULL;
	unsigned long simtask_name = nam2num("SIMULATION");

	/* Allocates memory to robot structure */
	if ( (robot = (st_robotMainArrays*) malloc(sizeof(st_robotMainArrays)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to main!\n\r");
		return NULL;
	}
	
	/* Allocates memory to shared's copy structure */
	if ( (sharedCp = (st_robotShared*) malloc(sizeof(st_robotShared)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to shared copy!\n\r");
		return NULL;
	}

	//robotInit(robot);
	memset(sharedCp, 0, sizeof(st_robotShared));

	if(taskCreateRtaiSim(simtask, simtask_name, SIMPRIORITY, STEPTIMESIMNANO) < 0) {
		fprintf(stderr, "Simulation!\n");
		free(robot);
		return NULL;
	}

	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;
		
		/* New X value */
		robotNewX(robot);

		/* wait for semaphore release from control thread */
		rt_sem_wait(shared->sem.rt_sem);

		/* Monitor get u*/
		RT_receive(node, 0, simtask, 0);
		
		/* Calculates x' from x and u*/
		robotDxSim(robot);

		/* Calculates y from x and the inputs */
		robotCalcYFromX(robot);

		/*monitor set y*/
		RT_send(node, 0, simtask, 0);

		/* release the semaphore to display thread */
	//	sem_post(&shared->sem.disp_sem);
	
		rt_task_wait_period();
		robot->kIndex++;
		robot->timeInstant[robot->kIndex] = currentT / SEC2NANO(1);	
		
		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);

	} while ( (fabs(total) <= (double)TOTAL_TIME) );

//#ifdef CALC_DATA
//	if ( robotCalcData(robot) < 0 ) {
//		free(robot);
//		free(sharedCp);
//		taskFinishRtaiSim(simtask);
//		return NULL;
//	}
//#endif /*CALC_DATA*/

	taskFinishRtaiSim(simtask);
	free(sharedCp);
	free(robot);
	return NULL;
}

