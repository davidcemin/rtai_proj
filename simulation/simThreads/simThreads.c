/*****************************************************************************/
/**
 * \file robotThreads.c
 * \brief Threads used on simulation
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
#include "robotStructs.h"
#include "robotSimulation.h"
#include "simThreads.h"
#include "robotDisplay.h"
#include "rtnetAPI.h"

/*rtai includes*/
#include <rtai_lxrt.h>
#include <rtai_netrpc.h>

/*****************************************************************************/

/**
 * \brief  Initializes shared memory
 * \param  shared Void pointer to shared memory
 * \return 0 Ok, -1 Error. 
 */
static int robotSimSharedInit(void *ptr)
{
	st_robotSimulShared *shared = ptr;

	pthread_mutex_init(&shared->mutexSim, NULL);

	if ( (sem_init(&shared->disp_sem, 0, 0) < 0) ) {
		fprintf(stderr, "Error in sem_init: %d\n", errno);
		return -1;
	}

	return 0;
}

/*****************************************************************************/

/**
 * \brief Cleans up shared memory.
 * \param shared void pointer to shared memory
 * \return void
 */
static void robotSimSharedCleanUp(void *ptr)
{
	st_robotSimulShared *shared = ptr;

	pthread_mutex_destroy(&shared->mutexSim);
	sem_destroy(&shared->disp_sem);
	stop_rt_timer();
	free(shared);
}

/*****************************************************************************/

void robotSimThreadsMain(void)
{
	void *shared; 
	
	int rt_simTask_thread;
	int stkSize;

	//pthread_t threadDisplay;
	//pthread_attr_t attr;
	//int ret;
	
	if ( (shared = (st_robotSimulShared*) malloc(sizeof(st_robotSimulShared)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to shared!\n\r");
		return;
	}
	
	/* shared init */
	memset(shared, 0, sizeof(st_robotSimulShared) );
	if( robotSimSharedInit(shared) < 0){
		free(shared);
		return;
	}

	/*Start timer*/
    rt_set_oneshot_mode(); 	
	start_rt_timer(0);

	stkSize = sizeof(st_robotSimulShared) + 10000;


	/*rtai simulation thread*/
	if(!(rt_simTask_thread = rt_thread_create(robotSimulation, shared, stkSize))) {
		fprintf(stderr, "Error Creating Simulation Thread!!\n");
		robotSimSharedCleanUp(shared);
		return;
	}	
	
	/*Create display thread*/
	
	/* For portability, explicitly create threads in a joinable state */
	//pthread_attr_init(&attr);
	//pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	//if ( (ret = pthread_create(&threadDisplay, &attr, robotThreadDisplay, shared) ) ) {
	//	fprintf(stderr, "Error Creating Display Thread: %d\n", ret);
	//	pthread_attr_destroy(&attr);
	//	robotSimSharedCleanUp(shared);
	//	return;
	//}

	/* Wait for all threads to complete */
	rt_thread_join(rt_simTask_thread);
	//pthread_join(threadDisplay, NULL);

	/* Clean up and exit */
	//pthread_attr_destroy(&attr);
	robotSimSharedCleanUp(shared);
	return; 
}
/*****************************************************************************/

