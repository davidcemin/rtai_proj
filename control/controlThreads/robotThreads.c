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
#include "monitorControlMain.h"
#include "robotThreads.h"
#include "simulCalcsUtils.h"
#include "robotControl.h"
#include "robotGeneration.h"
#include "robotLin.h"
#include "robotRefModels.h"
#include "robotStructs.h"
#include "rtnetAPI.h"
#include "rtaiCtrl.h"

/*rtai includes*/
#include <rtai_sem.h>
#include <rtai_lxrt.h>

/*****************************************************************************/

/**
 * \brief  Initialize the stack pointer
 * \param  *stack pointer to stack
 * \return void
 */
static void robotStackInit(st_robotControlStack *stack)
{
	stack->sem.sm_refx = rt_sem_init(nam2num("1g"), 0);	
	stack->sem.sm_refy = rt_sem_init(nam2num("2g"), 0);
	stack->sem.sm_control = rt_sem_init(nam2num("3g"), 0);
	stack->sem.sm_lin = rt_sem_init(nam2num("4g"), 0);
	stack->sem.sm_gen = rt_sem_init(nam2num("5g"), 0);

//	if(shared->sem.sm_refx == NULL)
//		printf("sem x null\n\r");

	//if ( (sem_init(&robotShared->sem.disp_sem, 0, 0) < 0) ) {
	//	fprintf(stderr, "Error in sem_init: %d\n", errno);
	//	return -1;
	//}
}

/*****************************************************************************/

/**
 * \brief Cleans up shared memory.
 * \param shared void pointer to shared memory
 * \return void
 */
static void robotStackCleanUp(st_robotControlStack *stack)
{
	rt_sem_delete(stack->sem.sm_refx);
	rt_sem_delete(stack->sem.sm_refy);
	rt_sem_delete(stack->sem.sm_control);
	rt_sem_delete(stack->sem.sm_lin);
	rt_sem_delete(stack->sem.sm_gen);
}

/*****************************************************************************/

void robotControlThreadsMain(void)
{
	st_robotControlStack stack;
	int rt_genTask_thread = 0;
	int rt_refXTask_thread = 0;   
	int rt_refYTask_thread = 0;
	int rt_controlTask_thread = 0;
	int rt_linTask_thread = 0;
	int i;
	int stkSize = sizeof(stack);

	struct {
		char *name;
		int thread;
		void *(*func)(void*);
	} rt_threads[] = {
		{"generation", rt_genTask_thread    , robotGeneration },
		{"refModX"   , rt_refXTask_thread   , robotRefModSimX },
		{"refModY"   , rt_refYTask_thread   , robotRefModSimY },
		{"control"   , rt_controlTask_thread, robotControl    },
		{"linear"    , rt_linTask_thread    , robotLin        },
	};
	
	/*shared init*/
	robotControlSharedInit();
	
	/* stack init */
	memset(&stack, 0, sizeof(stack) );
	robotStackInit(&stack);

	RT_TASK *mainTask = NULL;
	int tick = (int)nano2count((RTIME)TICK);
	int started_timer = 0;
	
	if( !(started_timer = taskCreateRtai(mainTask, "tsct", 1, tick)) ) {
		printf("aaz\n\r");
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

	/*stop timer, delete task, shared finish and permit pagination noch einmal*/
	robotControlSharedFinish();
	robotStackCleanUp(&stack);
	taskFinishRtai(mainTask, started_timer);
	return; 
}
/*****************************************************************************/

