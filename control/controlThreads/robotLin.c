/*****************************************************************************/
/**
 * \file robotLin.c
 * \brief Linearization Thread and auxiliary functions
 */
/*****************************************************************************/

/*libc Includes*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <arpa/inet.h>
#include <semaphore.h>
#include <string.h> 
#include <sys/time.h> 

/*robot includes*/
#include "libRobot.h"
#include "robotStructs.h"
#include "robotThreads.h"
#include "monitorControlMain.h"
#include "robotLin.h"
#include "rtnetAPI.h"
#include "rtaiCtrl.h"

/*rtai includes*/
#include <rtai_lxrt.h>
#include <rtai_netrpc.h>

/*****************************************************************************/

void *robotLin(void *ptr)
{
	st_robotControlStack *stack = ptr;
	st_robotControl *local;
	st_robotLinPacket *linPacket;
	RT_TASK *task = NULL;
	
	double currentT = 0;
	double lastT = 0;
	double total = 0;

	if ( (linPacket = (st_robotLinPacket*)malloc(sizeof(linPacket)) ) == NULL ) {
		fprintf(stderr, "Error in linPacket malloc\n");
		return NULL;
	}	
	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}

	mlockall(MCL_CURRENT | MCL_FUTURE);	

	memset(linPacket, 0, sizeof(linPacket));
	memset(local, 0, sizeof(local));
	
	/*registering real time task*/
	if((task = rt_thread_init(nam2num(LINEARTSK), LINPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"refY task init error!\n\r");
		return NULL;
	}

	/*sync*/
	printf("L: waiting control\n\r");
	rt_sem_wait(stack->sem.sm_lin);
		
	/*make it hard real time*/
	rt_make_hard_real_time();
	
	/*and finally make it periodic*/
	rt_task_make_periodic(task, nano2count(stack->time), TIMELIN*stack->tick);

	printf("LIN\n\r");
	do {
		currentT = rt_get_time_ns() - stack->time;
		/*Algorithm:
		 * 1) Get x;
		 * 2) Get v;
		 * 3) Generate u;
		 * 4) Send u;
		 */

		/* get  x*/
		//monitorControlMain(local, MONITOR_GET_X);

		/* Get v */
		//monitorControlMain(local, MONITOR_GET_V);

		/* Copy v into packet */
		//memcpy(linPacket->v, local->lin_t.v, sizeof(st_robotLin_t));

		/* Generate u*/
		//robotGenU(linPacket);

		/* send u */
		//monitorControlMain(local, MONITOR_SET_U);
		
		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period();
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	munlockall();
	printf("L: waiting control signal\n\r");
	rt_sem_wait(stack->sem.sm_lin);
	printf("finish Lina\n\r");
	
	rt_make_soft_real_time();
	rt_task_delete(task);
	printf("finish Linb\n\r");
	return NULL;
}

