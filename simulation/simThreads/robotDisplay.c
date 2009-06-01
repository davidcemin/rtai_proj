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

static inline void printDisplay(st_robotSimulPacket *packet, double t)
{
	struct {
		double y1;
		double y2; 
		double u1; 
		double u2;
	} disp = {packet->y[0], packet->y[1], packet->u[0], packet->u[1]};

	fprintf(stdout, "%f\t%f\t%f\t%f\t%f\n", disp.y1, disp.y2, disp.u1, disp.u2, t);

	return;
}

/*****************************************************************************/

/**
 * \brief  Thread used to show data on the screen. It is not a RTAI thread.
 * \param  ptr Pointer to shared memory
 * \return void
 */
void *robotThreadDisplay(void *ptr)
{
	st_robotSimulStack *stack = ptr;
	st_robotSimulPacket *packet;
	double tInit = 0;
	double lastT = 0;
	double currentT = 0;
	double t = 0;

	mlockall(MCL_CURRENT | MCL_FUTURE);

	/* Allocates memory to shared's copy structure */
	if ( (packet = (st_robotSimulPacket*) malloc(sizeof(packet)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to display packet!\n\r");
		return NULL;
	}
	
	memset(packet, 0, sizeof(packet));

	sem_wait(&stack->sm_disp);

	printf("display\n");
	
	/*time init*/
	tInit = stack->time;
	do {
		/* update the current Time */
		currentT = getTimeMilisec() - tInit;
		if ( ((currentT - lastT) >= (STEPTIMESIM * 1000) ) ){

			t = currentT / 1000.0;

			monitorSimMain(packet, MONITOR_GET_SIM_SHARED);

			printDisplay(packet, t);

			/* saves the last time */
			lastT = currentT;
		}
	} while (currentT < (double)TOTAL_TIME * 1000);
	
	munlockall();
	return NULL;
}

