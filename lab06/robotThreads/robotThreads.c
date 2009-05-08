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

/*rtai includes*/
#include <rtai_lxrt.h>

//! Defined used in jitter calculations and the like.
#define CALC_DATA

/*****************************************************************************/

/**
 * \brief  It logs the sampled data into a file
 * \param  sample Pointer to st_robotSample memory
 * \return void
 */
static inline int robotLogData(st_robotSample *sample)
{
	FILE *fd;
	int i;

	/* Opens a file to write */
	if ( (fd = fopen("dados.dat","w")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
		return -1;
	}
	
	fprintf(stdout, "Writing dados.dat\n"); 
	for(i = 2; i < sample->kIndex; i++)
		fprintf(fd, "%f\t%f\t%f\t%f\t%d\n", sample->yVal[0][i], sample->yVal[1][i], sample->yVal[2][i], 
				sample->timeInstant[i], i - 2);

	fclose(fd);
	return 0;
}

/*****************************************************************************/

/**
 * \brief  Function that creates a task in rtai
 * \param task Pointer to task 
 * \param taskName Number that corresponts to task name
 * \param priority priority of the task
 * \param stepTick Tick value, used on period.
 * \return 0 if ok, -1 error.
 */
static inline int taskCreateRtai(RT_TASK *task, unsigned long taskName, char priority, double stepTick) 
{
	int period;
	int stkSize;
	int msgSize;

	/*set root permissions to user space*/
	rt_allow_nonroot_hrt();

	/*Es ist nicht noetig um die priority im sched_param structure anzusetzen, da 
	 * es bereits im rt_task_init_schmod Function angesetzt ist.
	 */
	
	/*It Prevents the memory to be paged*/
    mlockall(MCL_CURRENT | MCL_FUTURE);
	
	msgSize = sizeof(st_robotShared);
	stkSize = msgSize + sizeof(st_robotMainArrays) + sizeof(st_robotSample) + 10000;

	if(!(task = rt_task_init_schmod(taskName, priority, stkSize, msgSize, SCHED_FIFO, 0xff) ) ) {	
		fprintf(stderr, "Cannot Init Task: ");
		return -1;
	}

	/*make it hard real time*/	
    rt_make_hard_real_time();

	/*set the period according to our desired tick*/
	period = (int)nano2count((RTIME)stepTick);

	/*start the task*/
	rt_task_make_periodic(task, rt_get_time()+period, period);

	return 0;
}

/*****************************************************************************/

/**
 * \brief Function to finish a rt task
 * \param task Pointer to task to be finished
 * \return void
 */
static inline void taskFinishRtai(RT_TASK *task)
{
	/* Here also is not necessary to use rt_make_soft_real_time because rt_task_delete 
	 * already use it(base/include/rtai_lxrt.h:842)
	 */
	rt_task_delete(task);
}

/*****************************************************************************/

/**
 * \brief  Thread to simulate the robot 
 * \param  ptr Pointer to shared data
 * \return void
 */
static void *robotSimulation(void *ptr)
{	
	st_robotShared *shared = ptr;
	st_robotShared *sharedCp;
	st_robotMainArrays *robot;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;

	RT_TASK *simtask = NULL;
	unsigned long simtask_name = nam2num("SIMULATION");

	/* Allocates memory to robot structure */
	if ( (robot = (st_robotMainArrays*) malloc(sizeof(st_robotMainArrays)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to main!\n\r");
		return NULL;
	}
	
	/* Allocates memory to shared's copy structure */
	if ( (sharedCp = (st_robotShared*) malloc(sizeof(st_robotShared)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to shared copy!\n\r");
		return NULL;
	}

	robotInit(robot);
	memset(sharedCp, 0, sizeof(st_robotShared));

	if(taskCreateRtai(simtask, simtask_name, SIMPRIORITY, STEPTIMESIMNANO) < 0) {
		fprintf(stderr, "Simulation!\n");
		free(robot);
		return NULL;
	}

	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;
		
		/* New X value */
		robotNewX(robot);

		/* wait for semaphore release from control thread */
		rt_sem_wait(shared->sem.rt_sem);

		/* Monitor get u*/
		monitorSimGet(sharedCp, shared);
		
		/* Get u values from shared */
		getUFromShared(robot, sharedCp);

		/* Calculates x' from x and u*/
		robotDxSim(robot);

		/* Calculates y from x and the inputs */
		robotCalcYFromX(robot);

		/*monitor set y*/
		monitorSimSet(robot, shared);

		/* release the semaphore to display thread */
		sem_post(&shared->sem.disp_sem);
	
		rt_task_wait_period();
		robot->kIndex++;
		robot->timeInstant[robot->kIndex] = currentT / SEC2NANO(1);	
		
		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);

	} while ( (fabs(total) <= (double)TOTAL_TIME) );

#ifdef CALC_DATA
	if ( robotCalcData(robot) < 0 ) {
		free(robot);
		free(sharedCp);
		taskFinishRtai(simtask);
		return NULL;
	}
#endif /*CALC_DATA*/

	taskFinishRtai(simtask);
	free(sharedCp);
	free(robot);
	return NULL;
}

/*****************************************************************************/

/**
* \brief Thread used to generate u and sample yf
* \param ptr Pointer to shared data
* \return void
*/
static void *robotGeneration(void *ptr)
{
	st_robotShared *shared = ptr;
	st_robotSample *sample;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;

	RT_TASK *calctask = NULL;
	unsigned long calctask_name = nam2num("CONTROL");
	
	/* Allocates memory to robot structure */
	if ( (sample = (st_robotSample*) malloc(sizeof(st_robotSample)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to sample!\n\r");
		return NULL;
	}

	/*sample init*/
	memset(sample, 0, sizeof(st_robotSample) );

	if(taskCreateRtai(calctask, calctask_name, CALCPRIORITY, STEPTIMECALCNANO) < 0){
		fprintf(stderr, "Calculation!\n");
		free(sample);
		exit(1);
	}

	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;

		/*monitor set u*/
		monitorCalcSet(shared, total);
		
		/* release semaphore to simulation thread */
		rt_sem_signal(shared->sem.rt_sem);

		/*monitor get y*/
		monitorCalcGet(sample, shared, total);

		sample->kIndex++;

		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period();

	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	/*log data*/
	if(	robotLogData(sample) < 0) 
		fprintf(stderr, "Error! It was not possible to log data!\n\r");

	taskFinishRtai(calctask);
	free(sample);
	return NULL;
}

/*****************************************************************************/

/**
 * \brief  Thread used to show data on the screen. It is not a RTAI thread.
 * \param  ptr Pointer to shared memory
 * \return void
 */
static void *robotThreadDisplay(void *ptr)
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

			/* Wait simulation thread release the semaphore */
			sem_wait(&shared->sem.disp_sem);
			
			monitorSimGet(sharedCp, shared);

			printDisplay(sharedCp, t);

			/* saves the last time */
			lastT = currentT;
		}
	} while (currentT < (double)TOTAL_TIME * 1000);
	
	free(sharedCp);
	return NULL;
}

/*****************************************************************************/

/**
 * \brief  
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
 * \brief  
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
	if(!(rt_calcTask_thread = rt_thread_create(robotGeneration, shared, stkSize))) {
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

