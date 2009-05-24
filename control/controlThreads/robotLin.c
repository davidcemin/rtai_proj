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

/*rtai includes*/
#include <rtai_lxrt.h>
#include <rtai_netrpc.h>

/*****************************************************************************/

/**
 * \brief  It creates a rtai task
 * \param  task Pointer to task 
 * \param  taskName Task's name
 * \param  priority Task's priority
 * \param  stepTick Step timer
 * \return -1 error, 0 ok.
 */
static inline int taskCreateRtaiLin(RT_TASK *task, unsigned long taskName, char priority, double stepTick) 
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
static inline void taskFinishRtaiLin(RT_TASK *task)
{
	/* Here also is not necessary to use rt_make_soft_real_time because rt_task_delete 
	 * already use it(base/include/rtai_lxrt.h:842)
	 */
	rt_task_delete(task);
}

/*****************************************************************************/

void *robotLin(void *ptr)
{
	st_robotControlShared *shared = ptr;
	st_robotControl *local;
	st_robotLinPacket *linPacket;
	st_rtnetSend *sendSim; 
	st_rtnetReceive *recvSim;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	
	unsigned long lintask_name = nam2num("LINTASK");
	RT_TASK *lintask = NULL;

	if(taskCreateRtaiLin(lintask, lintask_name, LINPRIORITY, STEPTIMELINNANO) < 0){
		fprintf(stderr, "Linearization!\n");
		return NULL;
	}

	if ( (linPacket = (st_robotLinPacket*)malloc(sizeof(linPacket)) ) == NULL ) {
		fprintf(stderr, "Error in linPacket malloc\n");
		return NULL;
	}
	
	if ( (sendSim = (st_rtnetSend*)malloc(sizeof(sendSim))) == NULL ) {
		fprintf(stderr, "Not possible to allocate memroy to sendSim packet\n\r");
		return NULL;
	}
	
	if ( (recvSim = (st_rtnetReceive*)malloc(sizeof(recvSim))) == NULL ) {
		fprintf(stderr, "Not possible to allocate memory to recvSim packet\n\r");
		return NULL;
	}
	
	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}

	/* Pointers init*/
	memset(local, 0, sizeof(local));
	memset(linPacket, 0, sizeof(linPacket));
	rtnetSendPacketInit(sendSim, "SIMTASK");
	rtnetRecvPacketInit(recvSim, "SIMTASK");

	printf("LIN\n\r");
	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;

		/*Algorithm:
		 * 1) Get x;
		 * 2) Get v;
		 * 3) Generate u;
		 * 4) Send u;
		 */

		/* Get x from simulation thread*/
		if (robotGetPacket(recvSim, (void*)linPacket->x) < 0)  {
			linPacket->x[0] = 0;
			linPacket->x[1] = 0;
			linPacket->x[2] = 0;
		}
	
		/* Get v */
		monitorControlMain(shared, local, MONITOR_GET_V);

		/* Copy v into packet */
		memcpy(linPacket->v, local->lin_t.v, sizeof(st_robotLin_t));

		/* Generate u*/
		robotGenU(linPacket);

		/* send u */
		robotSendPacket(sendSim, (void*)linPacket->u);

		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period();

	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	printf("l1\n\r");
	rtnetSendPacketFinish(sendSim);
	printf("l2\n\r");
	rtnetRecvPacketFinish(recvSim);
	printf("l3\n\r");
	taskFinishRtaiLin(lintask);
	printf("l4\n\r");
	return NULL;
}

