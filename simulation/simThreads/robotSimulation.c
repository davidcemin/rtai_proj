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
	st_robotSimulShared *shared = (st_robotSimulShared*)ptr;
	st_robotSimulPacket *simulPack;
	st_robotSimulPacket *simRemote;
	st_robotMainArrays *robot;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
		
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

	/*pointers init*/
	memset(robot, 0, sizeof(robot));
	memset(simulPack, 0, sizeof(simulPack));

	RT_TASK *planttsk = NULL;
	/*init task*/
	int started_timer=0;
	if( (started_timer = taskCreateRtai(planttsk, SIMTSK, SIMPRIORITY, STEPTIMESIMNANO, 0 ) ) < 0) {
		fprintf(stderr, "Simulation!\n");
//		free(robot);
		return NULL;
	}

	unsigned long ctrlnode = 0;
	int ctrlport=0;	
	
	/*init rtnet*/
	rtnetPacketInit(&shared->rtnet);
	ctrlport = shared->rtnet.port;
	ctrlnode = shared->rtnet.node;
	
	RT_TASK *ctrltsk=NULL;
	//RT_TASK *recvLin = NULL;
	
	while((ctrltsk=(RT_TASK *)RT_get_adr(ctrlnode,ctrlport,CONTROLTSK))==NULL) {
		usleep(100000);
	//	printf("Cant find task %s\n\r", CONTROLTSK);	
	}

	/*make it real time*/
	mkTaksRealTime(planttsk, STEPTIMESIMNANO, SIMTSK);

	printf("SIMULATION\n\r");
	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;
		
		/* New X value */
	//	robotNewX(robot);

		/* get u from lin thread*/
		long len;
		double u[2];
	
		/* get y */
		printf("Receiving..\n\r");
		RT_receivex(ctrlnode,ctrlport,ctrltsk,u,sizeof(u),&len);

		if(len != sizeof(u))
			printf("%ld != %d\n\r", len, sizeof(u));
		else{
			printf("%f %f \n\r", u[0], u[1]);
			/*Copy u into robot structure*/
			//cpUIntoRobot(robot, u);
		}

		/* Calculates x' from x and u*/
	//	robotDxSim(robot);

		/* Calculates y from x and the inputs */
	//	robotCalcYFromX(robot);

		/*copy x and y into packet structure*/
	//	cpXYIntoPacket(simulPack, robot);
		double xy[4];
		xy[0] = robot->kIndex*0.1;
		xy[1] = robot->kIndex*0.2;
		xy[2] = robot->kIndex*0.3;
		xy[3] = robot->kIndex*0.4;

		printf("Sending\n\r");
		/* send xy to control thread*/
		RT_sendx(ctrlnode,ctrlport,ctrltsk,xy,sizeof(xy));
	
		/*monitor set shared to display*/
		//monitorSimMain(shared, simulPack, MONITOR_SET_SIM_SHARED);

		robot->kIndex++;
		robot->timeInstant[robot->kIndex] = currentT / SEC2NANO(1);	
		
		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);
		rt_task_wait_period(); 
	} while ( (fabs(total) <= (double)TOTAL_TIME) );

//#ifdef CALC_DATA
//	if ( robotCalcData(robot) < 0 ) {
//		free(robot);
//		free(sharedCp);
//		taskFinishRtaiSim(simtask);
//		return NULL;
//	}
//#endif /*CALC_DATA*/

	rtnetPacketFinish(&shared->rtnet);
	taskFinishRtai(planttsk, started_timer);

	return 0;
}
