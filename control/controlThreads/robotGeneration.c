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
	st_robotControlStack *stack = ptr;
	st_robotControl *local;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	RT_TASK *gentask = NULL;
	
	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}
	
	mlockall(MCL_CURRENT | MCL_FUTURE);	
	
	/*registering real time task*/
	if((gentask = rt_thread_init(nam2num(GENTASK), GENPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"gentask init error!\n\r");
		return NULL;
	}
	
	/*wait sync*/
	printf("G: waiting control\n\r");
	rt_sem_wait(stack->sem.sm_gen);
	
	/*make it hard real time*/
	rt_make_hard_real_time();
	
	/*and finally make it periodic*/
	rt_task_make_periodic(gentask, nano2count(stack->time), TIMEGEN*stack->tick);
	
	printf("GENERATION\n");
	do {
		currentT = rt_get_time_ns() - stack->time;

		/*Algorithm:
		 * 1) Generate xref and yref;
		 * 2) Put it inside shared memory
		 */
		//robotRefGen(local, total);

		/*Set xref and yref into shared memory*/
	//	monitorControlMain(local, MONITOR_SET_REFERENCE_X);
	//	monitorControlMain(local, MONITOR_SET_REFERENCE_Y);

		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period();
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	printf("G: waiting control signal\n\r");
	rt_sem_wait(stack->sem.sm_gen);

	rt_make_soft_real_time();
	rt_task_delete(gentask);

	munlockall();
	
	printf("END GENb\n\r");
	return NULL;
}
