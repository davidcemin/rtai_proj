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
	st_robotControlShared *shared = ptr;
	st_robotControl *local;
	st_robotLinPacket *linPacket;
	st_rtnetSend *sendSim; 
	st_rtnetReceive *recvSim;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	RT_TASK *lintask = NULL;

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

	if(taskCreateRtai(lintask, LINEARTSK, LINPRIORITY, STEPTIMELINNANO, sizeof(st_robotControlShared)) < 0) {
		fprintf(stderr, "Linearization!\n");
		return NULL;
	}

	/* Pointers init*/
	memset(local, 0, sizeof(local));
	memset(linPacket, 0, sizeof(linPacket));
	rt_sem_wait(shared->sem.sm_lin);
	rtnetSendPacketInit(sendSim, SIMTSK);
	rtnetRecvPacketInit(recvSim, SIMTSK);

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
	taskFinishRtai(lintask);
	printf("l4\n\r");
	return NULL;
}

