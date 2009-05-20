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
#include "robotThreads.h"
#include "robotControl.h"
#include "robotLin.h"

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
static inline void taskFinishRtaiLin(RT_TASK *task)
{
	/* Here also is not necessary to use rt_make_soft_real_time because rt_task_delete 
	 * already use it(base/include/rtai_lxrt.h:842)
	 */
	rt_task_delete(task);
}

/*****************************************************************************/

/**
 * \brief  
 */
static inline void robotLinGetPacket(void *msg)
{
	RT_TASK *recvfrom = NULL;
	unsigned long recvnode;
	unsigned long recvport;
	struct sockaddr_in addr;
	long len = 0;
	
	inet_aton("127.0.0.1", &addr.sin_addr);
	recvnode = addr.sin_addr.s_addr;
	recvport = rt_request_hard_port(recvnode);

	recvfrom = RT_get_adr(recvnode, recvport, "SIMTASK");

	/*TODO: Treat errors..*/
	RT_receivex(recvnode, recvport, recvfrom, msg, X_DIMENSION * sizeof(double), &len);

	RT_release_port(recvnode, recvport);
}

/*****************************************************************************/

/**
 * \brief  
 */
static inline void robotLinSendPacket(void *msg)
{
	RT_TASK *sendto = NULL;
	unsigned int sendnode;
	unsigned int sendport;
	struct sockaddr_in addr;
	
	inet_aton("127.0.0.1", &addr.sin_addr);
	sendnode = addr.sin_addr.s_addr;
	sendport = rt_request_hard_port(sendnode);

	sendto = RT_get_adr(sendnode, sendport, "SIMTASK");

	/*TODO: Treat errors..*/
	RT_sendx(sendnode, sendport, sendto, msg, V_DIMENSION * sizeof(double) );

	RT_release_port(sendnode, sendport);
}

/*****************************************************************************/

void *robotLin(void *ptr)
{
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	st_robotLinPacket *linPacket;
	
	unsigned long lintask_name = nam2num("LINTSK");
	RT_TASK *lintask = NULL;

	if(taskCreateRtaiLin(lintask, lintask_name, LINPRIORITY, STEPTIMELINNANO) < 0){
		fprintf(stderr, "Linearization!\n");
		return NULL;
	}

	if ( (linPacket = (st_robotLinPacket*)malloc(sizeof(st_robotLinPacket)) ) == NULL ) {
		fprintf(stderr, "Error in linPacket malloc\n");
		return NULL;
	}

	memset(linPacket, 0, sizeof(st_robotLinPacket));

	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;

		/*Algorithm:
		 * 1) Get x;
		 * 2) Get v;
		 * 3) Generate u;
		 * 4) Send u;
		 */

		/* Get x */
		robotLinGetPacket((void*)linPacket->x);
	
		/* Get v */

		/* Generate u*/
		robotGenU(linPacket);

		/* send u */
		robotLinSendPacket((void*)linPacket->u);

		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period();

	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	taskFinishRtaiLin(lintask);
	return NULL;
}

