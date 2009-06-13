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
	st_robotControlStack *stack = ptr;
	st_robotControl local;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	RT_TASK *task = NULL;
	int i = 0;

	mlockall(MCL_CURRENT | MCL_FUTURE);	

	/*init some pointers..*/
	memset(&local, 0, sizeof(local));

	/*registering real time task*/
	if((task = rt_thread_init(nam2num(REFMODY), REFMODYPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"refY task init error!\n\r");
		return NULL;
	}

	printf("Y: waiting control\n\r");
	/*wait sync*/
	rt_sem_wait(stack->sem.sm_refy);
	
	rtaiMakeHard(task, REFMODY, STEPTIMEREFMODELYNANO);
	
	printf("REFMODY\n\r");
	double diff = 0;
	lastT = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns();
		diff = currentT - lastT;
		i++;
		
		/*get alpha*/
		monitorControlMain(&local, MONITOR_GET_ALPHA);

		/* Monitor get ref*/
		monitorControlMain(&local, MONITOR_GET_REFERENCE_Y);

		/* Calculate ym */
		robotSetYm(&local, (diff/SEC2NANO(1)), YREF_POSITION);
		
		/*save last*/
		local.refmod_t.ymLast = local.refmod_t.ym;

		/* Calculate ym' */
		robotDxYm(&local, YREF_POSITION);
	
		/*monitor set ym and ym'*/
		monitorControlMain(&local, MONITOR_SET_YMY);

		/*Timers procedure*/
		total += diff / SEC2NANO(1);
		lastT = currentT;
		local.refmod_t.k = i;
		local.refmod_t.time[i] = total;
		local.refmod_t.tc[i] = rt_get_time_ns() - currentT;
		//local.refmod_t.period[i] = diff;
		rt_task_wait_period();
	} while ( (fabs(total) <= (double)TOTAL_TIME) );

	printf("Y: waiting control signal\n\r");
	rt_sem_wait(stack->sem.sm_refy);
	
	rt_make_soft_real_time();
	rt_task_delete(task);

#ifdef CALC_DATA
	robotCalcData(local.refmod_t.tc, local.refmod_t.k, "results_refy_te.dat");
	//robotCalcData(local.refmod_t.period, local.refmod_t.k, "results_refy_period.dat");
	saveToFileGeneric(FILE_REFY, &local.refmod_t);
#endif /*CALC_DATA*/
	
	return NULL;
}

