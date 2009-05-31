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
#include "monitorSimMain.h"
#include "robotStructs.h"
#include "robotSimulation.h"
#include "simThreads.h"
#include "robotDisplay.h"
#include "rtaiCtrl.h"
#include "rtnetAPI.h"

/*rtai includes*/
#include <rtai_lxrt.h>
#include <rtai_netrpc.h>

/*****************************************************************************/

void robotSimThreadsMain(void)
{
	st_robotSimulStack stack;
	int rt_simTask_thread = 0;
	int stkSize = sizeof(stack);
	int i;

	struct {
		char *name;
		int thread;
		void *(*func)(void*);
	} rt_threads[] = {
		{"simulation", rt_simTask_thread, robotSimulation},
	};

	//pthread_t threadDisplay;
	//pthread_attr_t attr;
	//int ret;
	
	/* shared init */
	robotSimSharedInit();
	memset(&stack, 0, sizeof(stack) );

	RT_TASK *task = NULL;
	int tick = (int)nano2count((RTIME)TICK);
	int started_timer = 0;

	if ( !(started_timer = taskCreateRtai(task, "tssi", 1, tick))) {
		printf("asdg\n\r");
		return;
	}
	stack.tick = tick;
	stack.time = rt_get_time_ns() + (RTIME)TICK;

	/*rt threads init*/
	for (i = 0; i < NMEMB(rt_threads); i++) {
		printf("Creating thread: %s\n\r", rt_threads[i].name);
		if( (rt_threads[i].thread = rt_thread_create(rt_threads[i].func, &stack, stkSize)) == 0) {
			fprintf(stderr, "Error creating %s thread\n\r", rt_threads[i].name);
			return;
		}
	}
	
	/*rt threads join*/
	for (i = 0; i < NMEMB(rt_threads); i++) {
		printf("Joining thread: %s\n\r", rt_threads[i].name);
		rt_thread_join(rt_threads[i].thread);
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
	//pthread_join(threadDisplay, NULL);

	/* Clean up and exit */
	//pthread_attr_destroy(&attr);
	robotSimSharedFinish();
	taskFinishRtai(task, started_timer);
	return; 
}
/*****************************************************************************/

