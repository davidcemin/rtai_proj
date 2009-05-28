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
#include "rtnetAPI.h"

/*rtai includes*/
#include <rtai_sem.h>
#include <rtai_lxrt.h>

/*****************************************************************************/

/**
 * \brief  Initializes shared memory
 * \param  shared Void pointer to shared memory
 * \return 
 */
static int robotSharedInit(void *ptr)
{
	//st_robotControlShared *shared = (st_robotControlShared*)ptr;
	//int ret = 0;

//	ret += pthread_mutex_init(&shared->mutex.mutexControl, NULL);
//	ret += pthread_mutex_init(&shared->mutex.mutexLin, NULL);
//	ret += pthread_mutex_init(&shared->mutex.mutexGen, NULL);
//	ret += pthread_mutex_init(&shared->mutex.mutexPorts, NULL);
//
//	shared->sem.sm_refx = rt_sem_init(nam2num("1g"), 0);	
//	shared->sem.sm_refy = rt_sem_init(nam2num("2g"), 0);
//	shared->sem.sm_control = rt_sem_init(nam2num("3g"), 0);
//	shared->sem.sm_lin = rt_sem_init(nam2num("4g"), 0);
//	shared->sem.sm_gen = rt_sem_init(nam2num("5g"), 0);

//	if(shared->sem.sm_refx == NULL)
//		printf("sem x null\n\r");

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
	st_robotControlShared *shared = (st_robotControlShared*)ptr;

//	pthread_mutex_destroy(&shared->mutex.mutexGen);
//	pthread_mutex_destroy(&shared->mutex.mutexControl);
//	pthread_mutex_destroy(&shared->mutex.mutexLin);
//	pthread_mutex_destroy(&shared->mutex.mutexPorts);
//	rt_sem_delete(shared->sem.sm_refx);
//	rt_sem_delete(shared->sem.sm_refy);
//	rt_sem_delete(shared->sem.sm_control);
//	rt_sem_delete(shared->sem.sm_lin);
//	rt_sem_delete(shared->sem.sm_gen);
	//stop_rt_timer();
	free(shared);
}

/*****************************************************************************/

void robotControlThreadsMain(void)
{
	void *shared; 
	
	//int rt_genTask_thread;
	//int rt_refXTask_thread;
	//int rt_refYTask_thread;
	int rt_controlTask_thread;
	//int rt_linTask_thread;
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

	//if(!(rt_genTask_thread = rt_thread_create(robotGeneration, shared, stkSize))) {
	//	fprintf(stderr, "Error Creating generation Thread!!\n");
	//	robotSharedCleanUp(shared);
	//	return;
	//}	

	//if(!(rt_refXTask_thread = rt_thread_create(robotRefModSimX, shared, stkSize))) {
	//	fprintf(stderr, "Error Creating refX Thread!!\n");
	//	robotSharedCleanUp(shared);
	//	return;
	//}		
	
	//if(!(rt_refYTask_thread = rt_thread_create(robotRefModSimY, shared, stkSize))) {
	//	fprintf(stderr, "Error Creating refY Thread!!\n");
	//	robotSharedCleanUp(shared);
	//	return;
	//}	

	if(!(rt_controlTask_thread = rt_thread_create(robotControl, shared, stkSize))) {
		fprintf(stderr, "Error Creating control Thread!!\n");
		robotSharedCleanUp(shared);
		return;
	}	
	
	//if(!(rt_linTask_thread = rt_thread_create(robotLin, shared, stkSize))) {
	//	fprintf(stderr, "Error Creating linearization Thread!!\n");
	//	robotSharedCleanUp(shared);
	//	return;
	//}
	
	/* Wait for all threads to complete */
	//rt_thread_join(rt_genTask_thread);
	//rt_thread_join(rt_refXTask_thread);
	//rt_thread_join(rt_refYTask_thread);
	rt_thread_join(rt_controlTask_thread);
	//rt_thread_join(rt_linTask_thread);
	
	/* Clean up and exit */
	robotSharedCleanUp(shared);
	return; 
}
/*****************************************************************************/

