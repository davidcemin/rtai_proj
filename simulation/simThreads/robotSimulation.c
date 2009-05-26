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
	st_robotSimulShared *shared = ptr;
	st_robotSimulPacket *simulPack;
	st_robotMainArrays *robot;
	RT_TASK *sendCtrl = NULL;
	RT_TASK *sendLin = NULL; 
	RT_TASK *recvLin = NULL;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;

	RT_TASK *simtask = NULL;

	printf("rtnet init\n\r");
	/*init rtnet*/
	rtnetPacketInit(&shared->rtnet);
	
	/* Allocates memory to robot structure */
	if ( (robot = (st_robotMainArrays*) malloc(sizeof(robot)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to main!\n\r");
		return NULL;
	}
	
	if ( (simulPack = (st_robotSimulPacket*) malloc(sizeof(simulPack)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to simul packet!\n\r");
		return NULL;
	}	
	
	printf("Simulation!\n\r");
	if(taskCreateRtai(simtask, SIMTSK, SIMPRIORITY, STEPTIMESIMNANO, sizeof(st_robotSimulShared) ) < 0) {
		fprintf(stderr, "Simulation!\n");
		free(robot);
		return NULL;
	}

	memset(robot, 0, sizeof(robot));
	memset(simulPack, 0, sizeof(simulPack));
	rtnetTaskWait(&shared->rtnet, recvLin, LINEARTSK);
	rtnetTaskWait(&shared->rtnet, sendLin, LINEARTSK);
	rtnetTaskWait(&shared->rtnet, sendCtrl, CONTROLTSK);

	printf("SIMULATION\n\r");
	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;
		
		/* New X value */
		robotNewX(robot);

		/* get u from lin thread*/
		if(robotGetPacket(&shared->rtnet, recvLin, (void*)simulPack->u) == 0) {
			/*We need to keep the last packet when kein packet is received*/
			simulPack->u[0] = 0;
			simulPack->u[1] = 0;
		}
		
		/*Copy u into robot structure*/
		cpUIntoRobot(robot, simulPack->u);
		
		/* Calculates x' from x and u*/
		robotDxSim(robot);

		/* Calculates y from x and the inputs */
		robotCalcYFromX(robot);

		/*copy x and y into packet structure*/
		cpXYIntoPacket(simulPack, robot);
	
		/* send y to control thread*/
		robotSendPacket(&shared->rtnet, sendCtrl, (void*)simulPack->y);

		/* send x to lin thread */
		robotSendPacket(&shared->rtnet, sendLin, (void*)simulPack->x);
	
		/*monitor set shared to display*/
		monitorSimMain(shared, simulPack, MONITOR_SET_SIM_SHARED);

		rt_task_wait_period();
		robot->kIndex++;
		robot->timeInstant[robot->kIndex] = currentT / SEC2NANO(1);	
		
		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);

	} while ( (fabs(total) <= (double)TOTAL_TIME) );

//#ifdef CALC_DATA
//	if ( robotCalcData(robot) < 0 ) {
//		free(robot);
//		free(sharedCp);
//		taskFinishRtaiSim(simtask);
//		return NULL;
//	}
//#endif /*CALC_DATA*/

	taskFinishRtai(simtask);
	rtnetPacketFinish(&shared->rtnet);
	return NULL;
}

