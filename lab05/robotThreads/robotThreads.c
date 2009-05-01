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

#include <rtai_sched.h>
#include <rtai_lxrt.h>

//! Defined used in jitter calculations and the like.
#define CALC_DATA

/*****************************************************************************/

//! Shared memory's mutex
pthread_mutex_t mutexShared;

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
	
	for(i = 0; i < sample->kIndex; i++)
		fprintf(fd, "%f\t%f\t%f\t%f\t%d\n", sample->yVal[0][i], sample->yVal[1][i], sample->yVal[2][i], 
				sample->timeInstant[i], i);

	return 0;
}

/*****************************************************************************/

/**
 * \brief  
 */
static inline int rtai_taskCreate(RT_TASK *task, unsigned long taskName, char priority, double stepTick) 
{
	int period;
	struct sched_param sched;
	/*set root permissions to user space*/
	rt_allow_nonroot_hrt();

	/*set priority*/
    sched.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;

    sched_setscheduler(0, SCHED_FIFO, &sched);

	/*It Prevents the memory to be paged*/
    mlockall(MCL_CURRENT | MCL_FUTURE);

 	period = (int)nano2count((RTIME)stepTick);
    rt_make_hard_real_time();

	if(!(task = rt_task_init(taskName, priority, 0, 0))) {
		fprintf(stderr, "Cannot Init Task: ");
		return -1;
	}
    
	rt_task_make_periodic(task, rt_get_time()+period, period);

	return 0;
}

/*****************************************************************************/

/**
 * \brief  
 */
static inline void rtai_taskFinish(RT_TASK *task)
{
	rt_make_soft_real_time();
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
	//int period;

	/* Allocates memory to robot structure */
	if ( (robot = (st_robotMainArrays*) malloc(sizeof(st_robotMainArrays)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to main!\n\r");
		pthread_exit(NULL);
	}

	robotInit(robot);

	if(rtai_taskCreate(simtask, simtask_name, SIMPRIORITY, 30000000) < 0) {
		fprintf(stderr, "Simulation!\n");
		exit(1);
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
	} while ( (fabs(total) - (double)TOTAL_TIME) < CALCERROR );

#ifdef CALC_DATA
	if ( robotCalcData(robot) < 0 ) {
		rtai_taskFinish(simtask);
		pthread_exit(NULL);
	}
#endif /*CALC_DATA*/

	rtai_taskFinish(simtask);
	pthread_exit(NULL);
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
	double t = 0;
	double currentT = 0;
	double lastT = 0;
	double diff = 0;
	double total = 0;
	double tInit = 0;

	RT_TASK *calctask = NULL;
	unsigned long calctask_name = nam2num("CONTROL");
	
	/* Allocates memory to robot structure */
	if ( (sample = (st_robotSample*) malloc(sizeof(st_robotSample)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to main!\n\r");
		pthread_exit(NULL);
	}

	/*sample init*/
	memset(sample, 0, sizeof(st_robotSample) );

	if(rtai_taskCreate(calctask, calctask_name, CALCPRIORITY, 50000000) < 0){
		fprintf(stderr, "Calculation!\n");
		exit(1);
	}

	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;
		diff = currentT - lastT;

		/* Entering in crictical section */
		pthread_mutex_lock(&mutexShared);

		/* Calculates the inputs: u[n] */
		robotInputCalc(shared, t);

		/* Sample y and copy it into buffer */
		robotSampleYf(shared, sample, t);

		/* Leaving crictical section */
		pthread_mutex_unlock(&mutexShared);

		sample->kIndex++;

		lastT = currentT;
		t = currentT / SEC2NANO(1); 
	
		total = currentT / SEC2NANO(1);	
		rt_task_wait_period();

	} while ( (fabs(total) - (double)TOTAL_TIME) < CALCERROR);
	
	/*log data*/
	if(	robotLogData(sample) < 0) 
		fprintf(stderr, "Error! It was not possible to log data!\n\r");

	rtai_taskFinish(calctask);
	pthread_exit(NULL);
}
/*****************************************************************************/

void robotThreadsMain(void)
{
	void *shared; 
	
	int rt_simTask_thread;
	int rt_calcTask_thread;
	int period;
	
	if ( (shared = (st_robotShared*) malloc(sizeof(st_robotShared)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to shared!\n\r");
		return;
	}
	/* shared init */
	memset(shared, 0, sizeof(st_robotShared) );

	/*Start timer*/
    rt_set_oneshot_mode(); 
	period = (int) nano2count((RTIME) SEC2NANO(1));
    start_rt_timer(period);
	start_rt_timer(0);

	pthread_mutex_init(&mutexShared, NULL);

	if(!(rt_simTask_thread = rt_thread_create(robotSimulation, shared, 10000))) {
		fprintf(stderr, "Error Creating Simulation Thread!!\n");
		exit(1);
	}
	
	if(!(rt_calcTask_thread = rt_thread_create(robotGeneration, shared, 10000))) {
		fprintf(stderr, "Error Creating Calculation Thread!!\n\r");
		exit(1);
	}

	//! TODO: third thread to print the y and u values on the screen

	rt_thread_join(rt_simTask_thread);
	rt_thread_join(rt_calcTask_thread);
	pthread_mutex_destroy(&mutexShared);
	stop_rt_timer();
}
/*****************************************************************************/

