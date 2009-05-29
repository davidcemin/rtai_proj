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
	
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;

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

	memset(linPacket, 0, sizeof(linPacket));
	memset(linRemote, 0, sizeof(linRemote));
	memset(local, 0, sizeof(local));

	RT_TASK *lintask = NULL;
	int started_timer = 0;	
	
	/*init task*/
	if( (started_timer = taskCreateRtai(lintask, LINEARTSK, LINPRIORITY, STEPTIMELINNANO, 0)) < 0) {
		fprintf(stderr, "Linearization!\n");
		return NULL;
	}
	
	/*sync*/
	rt_sem_wait(shared->sem.sm_lin);
	
	/*make task real time*/
	mkTaksRealTime(lintask, STEPTIMELINNANO, LINEARTSK);
	
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

		/* get  x*/
		//monitorControlMain(shared, local, MONITOR_GET_X);

		/* Get v */
		//monitorControlMain(shared, local, MONITOR_GET_V);

		/* Copy v into packet */
		//memcpy(linPacket->v, local->lin_t.v, sizeof(st_robotLin_t));

		/* Generate u*/
		//robotGenU(linPacket);

		/* send u */
		//monitorControlMain(shared, local, MONITOR_SET_U);
		
		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period();
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	taskFinishRtai(lintask, started_timer);
	return NULL;
}

