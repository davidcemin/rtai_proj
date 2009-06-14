/*****************************************************************************/
/**
 * \file robotDisplay.c
 * \brief Display Thread and auxiliary functions
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
#include "monitorSimMain.h"
#include "robotStructs.h"
#include "simulCalcsUtils.h"

/*rtai includes*/
#include <rtai_lxrt.h>


/*****************************************************************************/

/**
 * \brief Print data on display
 * \param packet pointer to simulation packet
 * \param t Current time
 */
static inline void printDisplay(st_robotSimulPacket *packet, double t)
{
	struct {
		double y1;
		double y2; 
		double x3; 
		double u1; 
		double u2;
	} disp = {packet->y[0], packet->y[1], packet->x[2], packet->u[0], packet->u[1]};

	fprintf(stdout, "Y1:%03f\tY2:%03f\tX3:%03f\tU1:%03f\tU2:%03f\tT:%03f\n", disp.y1, disp.y2, disp.x3, disp.u1, disp.u2, t);

	return;
}

/*****************************************************************************/

void *robotThreadDisplay(void *ptr)
{
	st_robotSimulStack *stack = ptr;
	st_robotSimulPacket packet;
	double tInit = 0;
	double lastT = 0;
	double currentT = 0;
	double t = 0;

	//mlockall(MCL_CURRENT | MCL_FUTURE);

	memset(&packet, 0, sizeof(packet));

	sem_wait(&stack->sm_disp);

	printf("display\n");
	
	/*time init*/
	tInit = getTimeMilisec();
	do {
		/* update the current Time */
		currentT = getTimeMilisec() - tInit;
		if ( ((currentT - lastT) >= (STEPTIMESIM * 1000) ) ){

			t = currentT / 1000.0;

			monitorSimMain(&packet, MONITOR_GET_SIM_SHARED);

			printDisplay(&packet, t);

			/* saves the last time */
			lastT = currentT;
		}
	} while (currentT < (double)TOTAL_TIME * 1000);
	
	//munlockall();
	pthread_exit(NULL);
	//return NULL;
}

