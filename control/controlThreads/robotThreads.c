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
#include "robotThreads.h"
#include "simulCalcsUtils.h"
#include "robotControl.h"
#include "robotGeneration.h"
#include "robotLin.h"
#include "robotRefModels.h"
#include "robotStructs.h"

/*rtai includes*/
#include <rtai_lxrt.h>

/*****************************************************************************/

/**
 * \brief  Initializes shared memory
 * \param  shared Void pointer to shared memory
 * \return 0 Ok, -1 Error. 
 */
static int robotSharedInit(void *ptr)
{
	st_robotControlShared *shared = ptr;

	pthread_mutex_init(&shared->mutex.mutexControl, NULL);
	pthread_mutex_init(&shared->mutex.mutexLin, NULL);
	pthread_mutex_init(&shared->mutex.mutexGen, NULL);

	//robotShared->sem.rt_sem = rt_sem_init(nam2num("SEM_RT"), 0);

	//if ( (sem_init(&robotShared->sem.disp_sem, 0, 0) < 0) ) {
	//	fprintf(stderr, "Error in sem_init: %d\n", errno);
	//	return -1;
	//}

	return 0;
}

/*****************************************************************************/

/**
 * \brief Cleans up shared memory.
 * \param shared void pointer to shared memory
 * \return void
 */
static void robotSharedCleanUp(void *ptr)
{
	st_robotControlShared *shared = ptr;

	pthread_mutex_destroy(&shared->mutex.mutexControl);
	pthread_mutex_destroy(&shared->mutex.mutexLin);
	pthread_mutex_destroy(&shared->mutex.mutexGen);
	//rt_sem_delete(robotShared->sem.rt_sem);
	stop_rt_timer();
	free(shared);
}

/*****************************************************************************/

void robotControlThreadsMain(void)
{
	void *shared; 
	
	int rt_controlTask_thread;
	int rt_genTask_thread;
	int rt_linTask_thread;
	int rt_refXTask_thread;
	int rt_refYTask_thread;
	int stkSize;

	if ( (shared = (st_robotControlShared*) malloc(sizeof(st_robotControlShared)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to shared!\n\r");
		return;
	}
	
	/* shared init */
	memset(shared, 0, sizeof(st_robotControlShared) );
	if( robotSharedInit(shared) < 0){
		free(shared);
		return;
	}

	/*Start timer*/
    rt_set_oneshot_mode(); 	
	start_rt_timer(0);

	stkSize = sizeof(st_robotControlShared) + 10000;

	printf("a\n\r");
	if(!(rt_genTask_thread = rt_thread_create(robotGeneration, shared, stkSize))) {
		fprintf(stderr, "Error Creating generation Thread!!\n");
		robotSharedCleanUp(shared);
		return;
	}	

	printf("b\n\r");
	if(!(rt_controlTask_thread = rt_thread_create(robotControl, shared, stkSize))) {
		fprintf(stderr, "Error Creating control Thread!!\n");
		robotSharedCleanUp(shared);
		return;
	}	
	
	printf("c\n\r");
	if(!(rt_linTask_thread = rt_thread_create(robotLin, shared, stkSize))) {
		fprintf(stderr, "Error Creating linearization Thread!!\n");
		robotSharedCleanUp(shared);
		return;
	}
	
	printf("d\n\r");
	if(!(rt_refYTask_thread = rt_thread_create(robotRefModSimY, shared, stkSize))) {
		fprintf(stderr, "Error Creating refY Thread!!\n");
		robotSharedCleanUp(shared);
		return;
	}	

	printf("e\n\r");
	if(!(rt_refXTask_thread = rt_thread_create(robotRefModSimX, shared, stkSize))) {
		fprintf(stderr, "Error Creating refX Thread!!\n");
		robotSharedCleanUp(shared);
		return;
	}	
	
	/* Wait for all threads to complete */
	rt_thread_join(rt_controlTask_thread);
	rt_thread_join(rt_genTask_thread);
	rt_thread_join(rt_linTask_thread);
	rt_thread_join(rt_refXTask_thread);
	rt_thread_join(rt_refYTask_thread);

	/* Clean up and exit */
	robotSharedCleanUp(shared);
	return; 
}
/*****************************************************************************/

