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
#include "simulCalcsUtils.h"
#include "monitorControlMain.h"
#include "robotStructs.h"

/*rtai includes*/
#include <rtai_lxrt.h>


/*****************************************************************************/

/**
 * \brief  Print data on display
 * \param  local Pointer to control structure
 * \param  t Current time
 * \return void
 */
static inline void printDisplayCtrl(st_robotControl *local, double t)
{
	struct {
		double y1;
		double y2; 
		double x3;
		double refx;
		double refy;
		double alpha1;
		double alpha2;
	} disp = {local->lin_t.y[0], local->lin_t.y[1], local->lin_t.x[2],
	   	local->generation_t.ref[0][local->generation_t.k], local->generation_t.ref[1][local->generation_t.k],
	   	local->alpha[0], local->alpha[1]};

	fprintf(stdout, "Y1:%03f\tY2:%03f\tX3:%03f\tRefx:%03f\tRefy:%03f\tT:%03f\tA1:%03f\tA2:%03f\n",
		   	disp.y1, disp.y2, disp.x3, disp.refx, disp.refy, t, disp.alpha1, disp.alpha2);

	return;
}

/*****************************************************************************/

void *robotDispCtrl(void *ptr)
{
	st_robotControlStack *stack = ptr;
	st_robotControl local;
	double tInit = 0;
	double lastT = 0;
	double currentT = 0;
	double t = 0;

	//mlockall(MCL_CURRENT | MCL_FUTURE);

	memset(&local, 0, sizeof(local));

	sem_wait(&stack->sem.sm_disp);

	printf("display\n");
	
	/*time init*/
	tInit = getTimeMilisec();
	do {
		/* update the current Time */
		currentT = getTimeMilisec() - tInit;
		if ( ((currentT - lastT) >= (STEPTIMELIN * 1000) ) ){

			t = currentT / 1000.0;

			monitorControlMain(&local, MONITOR_GET_Y);
			monitorControlMain(&local, MONITOR_GET_REFERENCE_X);
			monitorControlMain(&local, MONITOR_GET_REFERENCE_Y);
			monitorControlMain(&local, MONITOR_GET_ALPHA);

			printDisplayCtrl(&local, t);

			/* saves the last time */
			lastT = currentT;
		}
	} while (currentT < (double)TOTAL_TIME * 1000);
	
	//munlockall();
	pthread_exit(NULL);
}

