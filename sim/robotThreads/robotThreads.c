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
#include "libRobot.h"
#include "monitorCalc.h"
#include "monitorSim.h"
#include "robotThreads.h"
#include "simulCalcsUtils.h"
#include "robotSimulation.h"
#include "robotControl.h"
#include "robotGeneration.h"
#include "robotDisplay.h"
#include "robotRefModels.h"

/*rtai includes*/
#include <rtai_lxrt.h>

/*****************************************************************************/

/**
 * \brief  Initializes shared memory
 * \param  shared Void pointer to shared memory
 * \return 0 Ok, -1 Error. 
 */
static int robotSharedInit(void *shared)
{
	st_robotShared *robotShared = shared;

	pthread_mutex_init(&robotShared->mutex.mutexSim, NULL);
	pthread_mutex_init(&robotShared->mutex.mutexCalc, NULL);

	robotShared->sem.rt_sem = rt_sem_init(nam2num("SEM_RT"), 0);

	if ( (sem_init(&robotShared->sem.disp_sem, 0, 0) < 0) ) {
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
static void robotSharedCleanUp(void *shared)
{
	st_robotShared *robotShared = shared;

	pthread_mutex_destroy(&robotShared->mutex.mutexSim);
	pthread_mutex_destroy(&robotShared->mutex.mutexCalc);
	rt_sem_delete(robotShared->sem.rt_sem);
	sem_destroy(&robotShared->sem.disp_sem);
	stop_rt_timer();
	free(shared);
}

/*****************************************************************************/

void robotThreadsMain(void)
{
	void *shared; 
	
	int rt_simTask_thread;
	int rt_calcTask_thread;
	int stkSize;

	pthread_t threadDisplay;
	pthread_attr_t attr;
	int ret;
	
	if ( (shared = (st_robotShared*) malloc(sizeof(st_robotShared)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to shared!\n\r");
		return;
	}
	
	/* shared init */
	memset(shared, 0, sizeof(st_robotShared) );
	if( robotSharedInit(shared) < 0){
		free(shared);
		return;
	}

	/*Start timer*/
    rt_set_oneshot_mode(); 	
	start_rt_timer(0);

	stkSize = 2*sizeof(st_robotShared) + sizeof(st_robotMainArrays) + sizeof(st_robotSample) + 10000;

	/*rtai simulation thread*/
	if(!(rt_simTask_thread = rt_thread_create(robotSimulation, shared, stkSize))) {
		fprintf(stderr, "Error Creating Simulation Thread!!\n");
		robotSharedCleanUp(shared);
		return;
	}	
	
	/*rtai control task*/
	if(!(rt_calcTask_thread = rt_thread_create(robotControl, shared, stkSize))) {
		fprintf(stderr, "Error Creating Calculation Thread!!\n");
		robotSharedCleanUp(shared);
		return;
	}	
	
	/*Create display thread*/
	
	/* For portability, explicitly create threads in a joinable state */
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	if ( (ret = pthread_create(&threadDisplay, &attr, robotThreadDisplay, shared) ) ) {
		fprintf(stderr, "Error Creating Display Thread: %d\n", ret);
		pthread_attr_destroy(&attr);
		robotSharedCleanUp(shared);
		return;
	}

	/* Wait for all threads to complete */
	rt_thread_join(rt_calcTask_thread);
	rt_thread_join(rt_simTask_thread);
	pthread_join(threadDisplay, NULL);

	/* Clean up and exit */
	pthread_attr_destroy(&attr);
	robotSharedCleanUp(shared);
	return; 
}
/*****************************************************************************/

