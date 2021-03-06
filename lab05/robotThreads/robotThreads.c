/*****************************************************************************/
/**
 * \file robotThreads.c
 * \brief Threads used on simulation
 */
/*****************************************************************************/

/*libc Includes*/
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h> 
#include <sys/time.h> 

/*robot includes*/
#include "libRobot.h"
#include "robotThreads.h"
#include "simulCalcsUtils.h"

/*rtai includes*/
#include <rtai_lxrt.h>

//! Defined used in jitter calculations and the like.
#define CALC_DATA

/*****************************************************************************/

//! Shared memory's mutex
pthread_mutex_t mutexShared = PTHREAD_MUTEX_INITIALIZER;

/*****************************************************************************/

/**
 * \brief  Gets u array from shared memory
 * \param  robot Pointer to st_robotMainArrays memory
 * \param  shared Pointer to st_robotShared memory
 * \return void
 */
static inline void getUFromShared(st_robotMainArrays *robot, st_robotShared *shared)
{
	int k = robot->kIndex;
	int i;

	for (i = 0; i < U_DIMENSION; i++)
		robot->uVal[i][k] = shared->u[i];
}

/*****************************************************************************/

/**
 * \brief  Copy yf array into shared memory
 * \param  robot Pointer to st_robotMainArrays memory
 * \param  shared Pointer to st_robotShared memory
 * \return void
 */
static inline void cpYIntoShared(st_robotMainArrays *robot, st_robotShared *shared)
{
	int k = robot->kIndex;
	int i;
	
	for (i = 0; i < XY_DIMENSION; i++)
		shared->yf[i] = robot->yVal[i][k];
}

/*****************************************************************************/

/**
 * \brief  It samples yf array to later save it in a file
 * \param  shared Pointer to st_robotShared memory
 * \param  sample Pointer to st_robotSample memory
 * \param  t Current simulation time
 * \return void
 */
static inline void robotSampleYf(st_robotShared *shared, st_robotSample *sample, double t)
{
	int i;
	sample->timeInstant[sample->kIndex] = t;

	for(i = 0; i < XY_DIMENSION; i++)
		sample->yVal[i][sample->kIndex] = shared->yf[i];
}

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
	
	for(i = 1; i < sample->kIndex; i++)
		fprintf(fd, "%f\t%f\t%f\t%f\t%d\n", sample->yVal[0][i], sample->yVal[1][i], sample->yVal[2][i], 
				sample->timeInstant[i], i - 1);

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

	robotInit(robot);

	if(taskCreateRtai(simtask, simtask_name, SIMPRIORITY, STEPTIMESIMNANO) < 0) {
		fprintf(stderr, "Simulation!\n");
		free(robot);
		return NULL;
	}

	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;
		
		/* Entering in crictical section */
		pthread_mutex_lock(&mutexShared);

		/* New X value */
		robotNewX(robot);

		/* Get u values from shared */
		getUFromShared(robot, shared);

		/* Calculates x' from x and u*/
		robotDxSim(robot);

		/* Calculates y from x and the inputs */
		robotCalcYFromX(robot);

		/* Copy y values into shared memory */
		cpYIntoShared(robot, shared);

		/* Leaving crictical section */
		pthread_mutex_unlock(&mutexShared);

		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);
		robot->kIndex++;

		rt_task_wait_period();
		robot->timeInstant[robot->kIndex] = currentT / SEC2NANO(1);
	} while ( (fabs(total) <= (double)TOTAL_TIME) );

#ifdef CALC_DATA
	if ( robotCalcData(robot) < 0 ) {
		free(robot);
		taskFinishRtai(simtask);
		return NULL;
	}
#endif /*CALC_DATA*/

	taskFinishRtai(simtask);
	free(robot);
	pthread_exit(NULL);
	//return NULL;
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
		pthread_exit(NULL);
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

		/* Entering in crictical section */
		pthread_mutex_lock(&mutexShared);

		/* Calculates the inputs: u[n] */
		robotInputCalc(shared, total);

		/* Sample y and copy it into buffer */
		robotSampleYf(shared, sample, total);

		/* Leaving crictical section */
		pthread_mutex_unlock(&mutexShared);

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
 * \brief Function used to print the information on the screen.
 * \param shared Pointer to shared memory.
 * \param t Current time of simulation
 * \return void
 */
static inline void printDisplay(st_robotShared *shared, double t)
{
	struct {
		double y1;
		double y2; 
		double y3;
		double u1; 
		double u2;
	} disp = {shared->yf[0], shared->yf[1], shared->yf[2], shared->u[0], shared->u[1]};

	fprintf(stdout, "%f\t%f\t%f\t%f\t%f\t%f\n", disp.y1, disp.y2, disp.y3, disp.u1, disp.u2, t);

	return;
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
	double tInit;
	double lastT;
	double currentT;
	double t;

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

			pthread_mutex_lock(&mutexShared);
			printDisplay(shared, t);
			pthread_mutex_unlock(&mutexShared);

			/* saves the last time */
			lastT = currentT;
		}
	} while (currentT < (double)TOTAL_TIME * 1000);
	
	return NULL;
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

	/*Start timer*/
    rt_set_oneshot_mode(); 	
	start_rt_timer(0);

	stkSize = sizeof(st_robotShared) + sizeof(st_robotMainArrays) + sizeof(st_robotSample) + 10000;

	/*rtai simulation thread*/
	if(!(rt_simTask_thread = rt_thread_create(robotSimulation, shared, stkSize))) {
		fprintf(stderr, "Error Creating Simulation Thread!!\n");
		pthread_mutex_destroy(&mutexShared);
		stop_rt_timer();
		free(shared);
		return;
	}	
	
	/*rtai control task*/
	if(!(rt_calcTask_thread = rt_thread_create(robotGeneration, shared, stkSize))) {
		fprintf(stderr, "Error Creating Calculation Thread!!\n");
		pthread_mutex_destroy(&mutexShared);
		stop_rt_timer();
		free(shared);
		return;
	}	
	
	/*Create display thread*/
	
	/* For portability, explicitly create threads in a joinable state */
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	if ( (ret = pthread_create(&threadDisplay, &attr, robotThreadDisplay, shared) ) ) {
		fprintf(stderr, "Error Creating Display Thread: %d\n", ret);
		pthread_mutex_destroy(&mutexShared);
		pthread_attr_destroy(&attr);
		stop_rt_timer();
		free(shared);
		return;
	}

	/* Wait for all threads to complete */
	rt_thread_join(rt_calcTask_thread);
	rt_thread_join(rt_simTask_thread);
	pthread_join(threadDisplay, NULL);

	/* Clean up and exit */
	pthread_mutex_destroy(&mutexShared);
	pthread_attr_destroy(&attr);
	free(shared);
	stop_rt_timer();
}
/*****************************************************************************/

