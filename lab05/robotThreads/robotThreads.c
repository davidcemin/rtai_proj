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

#define USE_RTAI

#ifdef USE_RTAI

#include <rtai_sched.h>
#include <rtai_lxrt.h>

#define DESIRED_TICK 1000000

#endif //USE_RTAI

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
	
	for(i = 0; i < sample->kIndex; i++)
		fprintf(fd, "%f\t%f\t%f\t%f\t%d\n", sample->yVal[0][i], sample->yVal[1][i], sample->yVal[2][i], 
				sample->timeInstant[i], i);

	return 0;
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
#ifdef USE_RTAI
	RT_TASK *simtask;
	unsigned long simtask_name = nam2num("SIMULATION");
	int period;
#else	
	/*Other variables*/
	double tInit;
	double lastT;
#endif

	/* Allocates memory to robot structure */
	if ( (robot = (st_robotMainArrays*) malloc(sizeof(st_robotMainArrays)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to main!\n\r");
		pthread_exit(NULL);
	}

	robotInit(robot);

#ifdef USE_RTAI 

	/*It Prevents the memory to be paged*/
    mlockall(MCL_CURRENT | MCL_FUTURE);

 	period = (int)nano2count((RTIME)STEPTIMESIMNANO);
    rt_make_hard_real_time();

	if(!(simtask = rt_task_init(simtask_name, SIMPRIORITY, 0, 0))) {
		fprintf(stderr, "Cannot Init simulation Task!\n");
		exit(1);
	}
    
	rt_task_make_periodic(simtask, rt_get_time()+period, period);

	currentT = count2nano(rt_get_time());

	do {

#else
	/*time init*/
	tInit = getTimeMilisec();
	lastT = 0;
	do {
		/* update the current Time */
		currentT = getTimeMilisec() - tInit;
		
		if ( ((currentT - lastT) >= (STEPTIMESIM * 1000) ) ){
#endif /*USE_RTAI*/
	
			/*common section(RTAI and Non-RTAI)*/			

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
			
			robot->kIndex++;
#ifdef USE_RTAI
			rt_task_wait_period();
			robot->timeInstant[robot->kIndex] = currentT / 1000000;
#else	
			/* saves the last time */
			lastT = currentT;
			robot->timeInstant[robot->kIndex] = currentT / 1000;
		}
#endif
	} while (currentT < (double)TOTAL_TIME * 1000);

#ifdef CALC_DATA
	if ( robotCalcData(robot) < 0 ) {
		free(robot);
		pthread_exit(NULL);
	}
#endif /*CALC_DATA*/

#ifdef USE_RTAI
	rt_task_delete(simtask);
#endif

	free(robot);
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
#ifdef USE_RTAI
	RT_TASK *calctask;
	unsigned long calctask_name = nam2num("CONTROL");
	int period;
#else	
	double tInit;
	double lastT;
#endif
	
	/* Allocates memory to robot structure */
	if ( (sample = (st_robotSample*) malloc(sizeof(st_robotSample)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to main!\n\r");
		pthread_exit(NULL);
	}

	/*sample init*/
	memset(sample, 0, sizeof(st_robotSample) );

#ifdef USE_RTAI

	/*It Prevents the memory to be paged*/
    mlockall(MCL_CURRENT | MCL_FUTURE);

 	period = (int)nano2count((RTIME)STEPTIMECALCNANO);
    rt_make_hard_real_time();

	if(!(calctask = rt_task_init(calctask_name, CALCPRIORITY, 0, 0))) {
		fprintf(stderr, "Cannot Init simulation Task!\n");
		exit(1);
	}
    
	rt_task_make_periodic(calctask, rt_get_time()+period, period);
	do {

#else
	/*time init*/
	tInit = getTimeMilisec();
	lastT = 0;

	do {
		/* update the current Time */
		currentT = getTimeMilisec() - tInit;
		
		if ( ((currentT - lastT) >= (STEPTIMECALC * 1000) ) ){
#endif /*USE_RTAI*/
	
			/* Entering in crictical section */
			pthread_mutex_lock(&mutexShared);
		
			/* Calculates the inputs: u[n] */
			robotInputCalc(shared, t);
			
			/* Sample y and copy it into buffer */
			robotSampleYf(shared, sample, t);
			
			/* Leaving crictical section */
			pthread_mutex_unlock(&mutexShared);
			
			sample->kIndex++;
			
#ifdef USE_RTAI
			currentT = count2nano(rt_get_time()) / 1000;
			t = currentT / 1000;
			rt_task_wait_period();
#else
			/* saves the last time */
			lastT = currentT;
			
			t = currentT / 1000;
		}
#endif
	
	} while (currentT < (double)TOTAL_TIME * 1000);
	
	/*log data*/
	if(	robotLogData(sample) < 0) 
		fprintf(stderr, "Error! It was not possible to log data!\n\r");

#ifdef USE_RTAI
	rt_task_delete(calctask);
#endif

	pthread_exit(NULL);
}
/*****************************************************************************/

void robotThreadsMain(void)
{
	void *shared; 
	
#ifdef USE_RTAI
	RT_TASK *maintask;
	int rt_simTask_thread;
	int rt_calcTask_thread;
	unsigned long maintask_name = nam2num("MAIN");
	struct sched_param mainsched;
	int period;
#else
	pthread_t threadSimulation;
	pthread_t threadGeneration;
	pthread_attr_t attr;
#endif 
	
	if ( (shared = (st_robotShared*) malloc(sizeof(st_robotShared)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to shared!\n\r");
		return;
	}
	/* shared init */
	memset(shared, 0, sizeof(st_robotShared) );

#ifdef USE_RTAI
	
	/*set root permissions to user space*/
	rt_allow_nonroot_hrt();

	/*set priority*/
    mainsched.sched_priority = sched_get_priority_max(SCHED_FIFO)-1;

    sched_setscheduler(0,SCHED_FIFO, &mainsched);


	/*It Prevents the memory to be paged*/
    mlockall(MCL_CURRENT | MCL_FUTURE);


	if(!(maintask = rt_task_init(maintask_name, MAINPRIORITY, 0, 0))) {
		fprintf(stderr, "Cannot Init Main Task!!\n");
		fprintf(stderr, "Please, try to load the modules first!(./run)\n");
		exit(1);
	}

    rt_set_oneshot_mode();
     
	period=(int)nano2count((RTIME)DESIRED_TICK);
	
	start_rt_timer(period);
    
	rt_make_hard_real_time();
	
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
	stop_rt_timer();
	rt_task_delete(maintask);

#else
	/* For portability, explicitly create threads in a joinable state */
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	/*Create both threads*/
	pthread_create(&threadSimulation, &attr, robotSimulation, shared);
	pthread_create(&threadGeneration, &attr, robotGeneration, shared);

	/* Wait for all threads to complete */
	pthread_join(threadSimulation, NULL);
	pthread_join(threadGeneration, NULL);

	/* Clean up and exit */
	pthread_mutex_destroy(&mutexShared);
	pthread_attr_destroy(&attr);

#endif
}
/*****************************************************************************/

