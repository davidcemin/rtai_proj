/*****************************************************************************/
/**
 * \file robotSimulation.c
 * \brief Thread used in simulation's block
 */
/*****************************************************************************/

/*libc Includes*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <arpa/inet.h>
#include <semaphore.h>
#include <string.h> 
#include <sys/time.h> 

/*robot includes*/
#include "monitorSimMain.h"
#include "libRobot.h"
#include "robotStructs.h"
#include "rtnetAPI.h"
#include "simThreads.h"
#include "rtaiCtrl.h"

/*rtai includes*/
#include <rtai_netrpc.h>
#include <rtai_lxrt.h>

//! Defined used in jitter calculations and the like.
#define CALC_DATA

/*****************************************************************************/

/**
 * \brief  It copies u array into u array in robot main structure
 * \param  robot Pointer to robot structure
 * \param  u Pointer to u array
 * \return void
 */
static inline void cpUIntoRobot(st_robotMainArrays *robot, double *u)
{
	int i;
	int k = robot->kIndex;
	for (i = 0; i < U_DIMENSION; i++)
		robot->uVal[i][k] = u[i]; 
}

/*****************************************************************************/

/**
 * \brief  It copies x and y arrays from robot structure to packet
 * \param  packet Pointer to the packet
 * \param  robot Pointer to robot structure
 * \return void
 */
static inline void cpXYIntoPacket(st_robotSimulPacket *packet, st_robotMainArrays *robot)
{
	int i;
	int k = robot->kIndex;

	for (i = 0; i < X_DIMENSION; i++)
		packet->x[i] = robot->xVal[i][k];
	
	for (i = 0; i < Y_DIMENSION; i++)
		packet->y[i] = robot->yVal[i][k];
}

/*****************************************************************************/

void *robotSimulation(void *ptr)
{	
	st_robotSimulStack *stack = ptr;
	st_robotSimulPacket *simulPack;
	st_robotSimulPacket *simRemote;
	st_robotMainArrays *robot;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	long len;
	double u[U_DIMENSION];
	double xy[XY_DIMENSION];
		
	/* Allocates memory to robot structure */
	if ( (robot = (st_robotMainArrays*) malloc(sizeof(robot)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to main!\n\r");
		return NULL;
	}
	
	if ( (simulPack = (st_robotSimulPacket*) malloc(sizeof(simulPack)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to simul packet!\n\r");
		return NULL;
	}	

	if ( (simRemote = (st_robotSimulPacket*) malloc(sizeof(simRemote)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to simul packet!\n\r");
		return NULL;
	}	

	mlockall(MCL_CURRENT | MCL_FUTURE);	

	/*pointers init*/
	memset(robot, 0, sizeof(robot));
	memset(simulPack, 0, sizeof(simulPack));

	RT_TASK *planttsk = NULL;
	/*init task*/
	int started_timer=0;
	/*registering realtime task*/
	if((planttsk = rt_thread_init(nam2num(SIMTSK), SIMPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0) {
		printf("sim task init error\n\r");
		return NULL;
	}

	//if( (started_timer = taskCreateRtai(planttsk, SIMTSK, SIMPRIORITY, STEPTIMESIMNANO ) ) < 0) {
	//	fprintf(stderr, "Simulation!\n");
//		free(robot);
	//	return NULL;
	//}

	st_rtnetRobot rtnet;
	unsigned long ctrlnode = 0;
	int ctrlport=0;	
	
	/*init rtnet*/
	rtnetPacketInit(&rtnet);
	ctrlport = rtnet.port;
	ctrlnode = rtnet.node;
	
	RT_TASK *ctrltsk=NULL;
	
	while((ctrltsk=(RT_TASK *)RT_get_adr(ctrlnode,ctrlport,CONTROLTSK))==NULL) {
		usleep(100000);
	//	printf("Cant find task %s\n\r", CONTROLTSK);	
	}

	rt_make_hard_real_time();

	rt_task_make_periodic(planttsk, nano2count(stack->time), TIMESIM*stack->tick);

	/*make it real time*/
	//mkTaksRealTime(planttsk, STEPTIMESIMNANO, SIMTSK);

	stack->time = rt_get_time_ns();
	printf("SIMULATION\n\r");
	do {
		currentT = rt_get_time_ns() - stack->time;
		
		/* New X value */
	//	robotNewX(robot);

		/* get u from lin thread*/
			
		/* get y */
		//printf("Receiving..\n\r");
		RT_receivex(ctrlnode,ctrlport,ctrltsk,u,sizeof(u),&len);

		if(len != sizeof(u))
			printf("%ld != %d\n\r", len, sizeof(u));
		else if (u[0] == DUMMYPACK || u[1] == DUMMYPACK) {
			printf("Dummy packet received. Breaking now..\n\r");
			break;
		}
		else{
		//	printf("%f %f \n\r", u[0], u[1]);
			/*Copy u into robot structure*/
			//cpUIntoRobot(robot, u);
		}

		/* Calculates x' from x and u*/
	//	robotDxSim(robot);

		/* Calculates y from x and the inputs */
	//	robotCalcYFromX(robot);

	
		/*copy x and y into packet structure*/
	//	cpXYIntoPacket(simulPack, robot);
		xy[0] = robot->kIndex*0.1;
		xy[1] = robot->kIndex*0.2;
		xy[2] = robot->kIndex*0.3;
		xy[3] = robot->kIndex*0.4;

		//printf("Sending\n\r");
		/* send xy to control thread*/
		RT_sendx(ctrlnode,-ctrlport,ctrltsk,xy,sizeof(xy));
	
		/*monitor set shared to display*/
		//monitorSimMain(simulPack, MONITOR_SET_SIM_SHARED);
		printf("k: %d\n\r", robot->kIndex);

		robot->kIndex++;
		robot->timeInstant[robot->kIndex] = currentT / SEC2NANO(1);	
		
		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);
		rt_task_wait_period(); 
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	xy[0] = DUMMYPACK;
	xy[1] = DUMMYPACK;
	xy[2] = DUMMYPACK;
	xy[3] = DUMMYPACK;

	printf("Sending dummy\n\r");
	/* send xy to control thread*/
	RT_sendx(ctrlnode,ctrlport,ctrltsk,xy,sizeof(xy));

//#ifdef CALC_DATA
//	if ( robotCalcData(robot) < 0 ) {
//		free(robot);
//		free(sharedCp);
//		taskFinishRtaiSim(simtask);
//		return NULL;
//	}
//#endif /*CALC_DATA*/

	rtnetPacketFinish(&rtnet);
	taskFinishRtai(planttsk, started_timer);

	return 0;
}
