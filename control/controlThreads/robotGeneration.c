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
	st_robotControl local;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	RT_TASK *gentask = NULL;
	int i =0;
	
	mlockall(MCL_CURRENT | MCL_FUTURE);	
	
	/*registering real time task*/
	if((gentask = rt_thread_init(nam2num(GENTASK), GENPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"gentask init error!\n\r");
		return NULL;
	}
	
	memset(&local, 0, sizeof(local));
	/*sync*/
	rt_sem_wait(stack->sem.sm_gen);
	
	rtaiMakeHard(gentask, GENTASK, STEPTIMEGENERNANO);
	
	double diff =0;
	//double tInit = stack->time;
	//lastT = stack->time;
	lastT = rt_get_time_ns();
	printf("GENERATION\n");
	do {
		currentT = rt_get_time_ns();
		diff = currentT - lastT;
		i++;

		/*Algorithm:
		 * 1) Generate xref and yref;
		 * 2) Put it inside shared memory
		 */
		robotRefGen(&local, total);

		/*Set xref and yref into shared memory*/
		monitorControlMain(&local, MONITOR_SET_REFERENCE_X);
		monitorControlMain(&local, MONITOR_SET_REFERENCE_Y);

		total += diff/SEC2NANO(1);
		lastT = currentT;
		local.generation_t.tc[i] = rt_get_time_ns() - currentT;
		local.generation_t.k = i;
		local.generation_t.time[i] = total;
		local.generation_t.period[i] = diff;
		rt_task_wait_period();
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	printf("G: waiting control signal\n\r");
	rt_sem_wait(stack->sem.sm_gen);

	rt_make_soft_real_time();
	rt_task_delete(gentask);
	
#ifdef CALC_DATA
	robotCalcData(local.generation_t.tc, local.generation_t.k, "results_gen_te.dat");
	robotCalcData(local.generation_t.period, local.generation_t.k, "results_gen_pd.dat");
	saveToFileGeneric(FILE_GEN, &local.generation_t);
#endif /*CALC_DATA*/

	return NULL;
}
