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
	st_robotLinPacket *linRemote;
	RT_TASK *sendSim = NULL; 
	RT_TASK *recvSim = NULL;
	int stack = sizeof(st_robotControl) + 2*sizeof(st_robotLinPacket);
	int started_timer = 0;
		
	long len = 0;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	RT_TASK *lintask = NULL;

	if ( (linPacket = (st_robotLinPacket*)malloc(sizeof(linPacket)) ) == NULL ) {
		fprintf(stderr, "Error in linPacket malloc\n");
		return NULL;
	}	
	if ( (linRemote = (st_robotLinPacket*)malloc(sizeof(linRemote)) ) == NULL ) {
		fprintf(stderr, "Error in linPacket malloc\n");
		return NULL;
	}
	
	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}

	/*sync*/
	rt_sem_wait(shared->sem.sm_lin);
	
	/*init task*/
	if( (started_timer = taskCreateRtai(lintask, LINEARTSK, LINPRIORITY, STEPTIMELINNANO, stack)) < 0) {
		fprintf(stderr, "Linearization!\n");
		return NULL;
	}

	/*wait handler, it was alredy initialized...*/
	rtnetTaskWait(&shared->rtnet, recvSim, SIMTSK);
	
	/*make task real time*/
	mkTaksRealTime(lintask, STEPTIMELINNANO, LINEARTSK);

	/* Pointers init*/
	memset(local, 0, sizeof(local));
	memset(linPacket, 0, sizeof(linPacket));

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
		//robotGetPacket(&shared->rtnet, recvSim, (void*)linPacket->x);	

		printf("RCTRL: node: 0x%lx port: %d\n\r", shared->rtnet.node, shared->rtnet.port);
		RT_receivex(shared->rtnet.node, shared->rtnet.port, recvSim, (void*)linRemote->x, sizeof(linRemote->x), &len);

		if( len != sizeof(linRemote->x) ) {
			printf("len: %ld msg: %d\n\r", len, sizeof(linRemote->x));
		}
		else
			memcpy(&linPacket->x, &linRemote->x, sizeof(linRemote->x));


		/* Get v */
		monitorControlMain(shared, local, MONITOR_GET_V);

		/* Copy v into packet */
		memcpy(linPacket->v, local->lin_t.v, sizeof(st_robotLin_t));

		/* Generate u*/
		robotGenU(linPacket);

		/* send u */
		//robotSendPacket(&shared->rtnet, sendSim, (void*)linPacket->u);
		RT_sendx(shared->rtnet.node, shared->rtnet.port, sendSim, (void*)linPacket->u, sizeof(linPacket->u));

		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period();

	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	rtnetPacketFinish(&shared->rtnet);
	taskFinishRtai(lintask, started_timer);
	free(local);
	free(linPacket);
	free(linRemote);
	return NULL;
}

