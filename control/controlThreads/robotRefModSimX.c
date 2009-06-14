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
#include "simulCalcsUtils.h"

/*rtai includes*/
#include <rtai_lxrt.h>

/*****************************************************************************/

void *robotRefModSimX(void *ptr)
{	
	st_robotControlStack *stack = ptr;
	st_robotControl local_x;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	RT_TASK *task = NULL;
	int i = 0;

	mlockall(MCL_CURRENT | MCL_FUTURE);	

	/*init some pointers*/
	memset(&local_x, 0, sizeof(local_x));
	
	/*registering real time task*/
	if((task = rt_thread_init(nam2num(REFMODX), REFMODXPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"refX task init error!\n\r");
		return NULL;
	}

	/*wait sync*/
	printf("X: waiting control\n\r");
	rt_sem_wait(stack->sem.sm_refx);

	rtaiMakeHard(task, REFMODX, STEPTIMEREFMODELXNANO);

	double diff = 0;
	lastT = rt_get_time_ns();
	printf("REFMODX\n\r");
	do {
		currentT = rt_get_time_ns();
		diff = currentT - lastT;
		i++;
		
		/*get alpha*/
		monitorControlMain(&local_x, MONITOR_GET_ALPHA);

		/* Monitor get ref*/
		monitorControlMain(&local_x, MONITOR_GET_REFERENCE_X);
		
		/* Calculate ym */
		robotSetYm(&local_x, (diff/SEC2NANO(1)), XREF_POSITION);

		/*save last*/
		local_x.refmod_t.ymLast = local_x.refmod_t.ym;

		/* Calculate ym' */
		robotDxYm(&local_x, XREF_POSITION);
						
		/*monitor set ym and ym'*/
		monitorControlMain(&local_x, MONITOR_SET_YMX);

		/*Timers procedure*/
		total += diff / SEC2NANO(1);
		lastT = currentT;
		local_x.refmod_t.k = i;
		local_x.refmod_t.time[i] = total;
		local_x.refmod_t.tc[i] = rt_get_time_ns() - currentT;
		local_x.refmod_t.period[i] = diff;
		rt_task_wait_period();
	} while ( (fabs(total) <= (double)TOTAL_TIME) );

	printf("X: waiting control signal\n\r");
	
	rt_sem_wait(stack->sem.sm_refx);
	rt_make_soft_real_time();
	rt_task_delete(task);
	
#ifdef CALC_DATA
	robotCalcData(local_x.refmod_t.tc, local_x.refmod_t.k, "results_refx_te.dat");
	robotCalcData(local_x.refmod_t.period, local_x.refmod_t.k, "results_refx_pd.dat");
	saveToFileGeneric(FILE_REFX, &local_x.refmod_t);
#endif /*CALC_DATA*/
	
	return NULL;
}

