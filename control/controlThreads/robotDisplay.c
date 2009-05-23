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
#include "libRobot.h"
#include "monitorSim.h"
#include "robotStructs.h"
#include "robotGeneration.h"
#include "robotThreads.h"
#include "simulCalcsUtils.h"

/*rtai includes*/
#include <rtai_lxrt.h>

/*****************************************************************************/

/**
 * \brief  Thread used to show data on the screen. It is not a RTAI thread.
 * \param  ptr Pointer to shared memory
 * \return void
 */
void *robotThreadDisplay(void *ptr)
{
	st_robotShared *shared = ptr;
	st_robotShared *sharedCp;
	double tInit;
	double lastT;
	double currentT;
	double t;

	/* Allocates memory to shared's copy structure */
	if ( (sharedCp = (st_robotShared*) malloc(sizeof(st_robotShared)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to shared copy!\n\r");
		return NULL;
	}

	memset(sharedCp, 0, sizeof(st_robotShared));

	/*time init*/
	tInit = getTimeMilisec();
	lastT = 0;
	currentT = 0;
	t = 0;

	do {
		/* update the current Time */
		currentT = getTimeMilisec() - tInit;
		if ( ((currentT - lastT) >= (STEPTIMESIM * 1000) ) ){

			t = currentT / 1000.0;

			monitorSimGet(sharedCp, shared);

			printDisplay(sharedCp, t);

			/* saves the last time */
			lastT = currentT;
		}
	} while (currentT < (double)TOTAL_TIME * 1000);
	
	free(sharedCp);
	return NULL;
}

