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

void robotSimThreadsMain(char *ip)
{
	st_robotSimulStack stack;
	int rt_simTask_thread = 0;
	pthread_t threadDisplay;
	pthread_attr_t attrd;
	int retd = 0;

	int stkSize = sizeof(stack) + 100000;
	int i;

	struct {
		char *name;
		int thread;
		void *(*func)(void*);
	} rt_threads[] = {
		{"simulation", rt_simTask_thread, robotSimulation},
	};

	/* shared init */
	robotSimSharedInit();
	memset(&stack, 0, sizeof(stack) );

	stack.ip = ip;
	
	rt_allow_nonroot_hrt();
	sem_init(&stack.sm_disp, 0, 0);

	/*rt threads init*/
	for (i = 0; i < NMEMB(rt_threads); i++) {
		printf("Creating thread: %s\n\r", rt_threads[i].name);
		if( (rt_threads[i].thread = rt_thread_create(rt_threads[i].func, &stack, stkSize)) == 0) {
			fprintf(stderr, "Error creating %s thread\n\r", rt_threads[i].name);
			robotSimSharedFinish();
			return;
		}
	}

	/*Create posix threads*/
	pthread_attr_init(&attrd);
	pthread_attr_setdetachstate(&attrd, PTHREAD_CREATE_JOINABLE);
		
	if ( (retd = pthread_create(&threadDisplay, &attrd, robotThreadDisplay , &stack))) {
		fprintf(stderr, "Error Creating display Thread: %d\n", retd);
		pthread_attr_destroy(&attrd);
		robotSimSharedFinish();
		return;
	}
	
	/*rt threads join*/
	for (i = 0; i < NMEMB(rt_threads); i++) {
		printf("Joining thread: %s\n\r", rt_threads[i].name);
		rt_thread_join(rt_threads[i].thread);
	}

	/*posix threads join*/
	printf("Joining thread display\n\r");
	pthread_join(threadDisplay, NULL);

	/* Clean up and exit */
	pthread_attr_destroy(&attrd);

	robotSimSharedFinish();
	sem_destroy(&stack.sm_disp);
	return; 
}
/*****************************************************************************/

