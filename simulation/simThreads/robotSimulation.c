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
	st_robotMainArrays *robot;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	long len;
	double u[U_DIMENSION];
	double xy[XY_DIM_PACKET];
		
	/* Allocates memory to robot structure */
	if ( (robot = (st_robotMainArrays*) malloc(sizeof(robot)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to main!\n\r");
		return NULL;
	}
	
	if ( (simulPack = (st_robotSimulPacket*) malloc(sizeof(simulPack)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to simul packet!\n\r");
		return NULL;
	}	

	mlockall(MCL_CURRENT | MCL_FUTURE);	

	/*pointers init*/
	memset(robot, 0, sizeof(robot));
	memset(simulPack, 0, sizeof(simulPack));

	RT_TASK *planttsk = NULL;
	
	/*registering realtime task*/
	if((planttsk = rt_thread_init(nam2num(SIMTSK), SIMPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0) {
		printf("sim task init error\n\r");
		return NULL;
	}

	st_rtnetRobot rtnet;
	unsigned long ctrlnode = 0;
	int ctrlport=0;	
	
	/*init rtnet*/
	rtnetPacketInit(&rtnet);
	ctrlport = rtnet.port;
	ctrlnode = rtnet.node;
	
	RT_TASK *ctrltsk=NULL;
	
	while((ctrltsk=(RT_TASK *)RT_get_adr(ctrlnode,ctrlport,LINEARTSK))==NULL) {
		usleep(100000);
		printf("Waiting %s to be up...\n\r", LINEARTSK);
	}

	rt_make_hard_real_time();

	rt_task_make_periodic(planttsk, nano2count(stack->time), TIMESIM*stack->tick);
	
	simulPack->x[0] = 0;
	simulPack->x[1] = 1;
	simulPack->x[2] = 0;
	stack->time = rt_get_time_ns();
	sem_post(&stack->sm_disp);

	printf("SIMULATION\n\r");
	do {
		currentT = rt_get_time_ns() - stack->time;
		robot->kIndex++;
		robot->timeInstant[robot->kIndex] = currentT / SEC2NANO(1);	

		/* Algorithm:
		 * 1) get u from lin thread;
		 * 2) calculate new x value;
		 * 3) calculate x' from x and u;
		 * 4) calculate y from x and u
		 * 5) send x and y to control
		 */
		
		/* get u from lin thread*/
		RT_receivex(ctrlnode,ctrlport,ctrltsk,u,sizeof(u),&len);

		if(len != sizeof(u))
			printf("%ld != %d\n\r", len, sizeof(u));
		else if (u[0] == DUMMYPACK || u[1] == DUMMYPACK) {
			printf("Dummy packet received. Breaking now..\n\r");
			break;
		}
		else{
			//printf("%f %f \n\r", u[0], u[1]);
			/*Copy u into robot structure*/
			cpUIntoRobot(robot, u);
		}
	
		/* Calculates x' from x and u*/
		robotDxSim(robot);
	
		/* New X value */
		robotNewX(robot);
	
		/* Calculates y from x*/
		robotCalcYFromX(robot);
			
		/*copy x and y into packet structure*/
		//cpXYIntoPacket(simulPack, robot);
			
		/*monitor set shared to display*/
		//monitorSimMain(simulPack, MONITOR_SET_SIM_SHARED);

		xy[0] = robot->xVal[2][robot->kIndex];
		xy[1] = robot->yVal[0][robot->kIndex];
		xy[2] = robot->yVal[1][robot->kIndex];
		printf("%d %f %f %f\n\r", robot->kIndex, xy[0], xy[1], xy[2]);
		/* send xy to control thread*/
		RT_sendx(ctrlnode,-ctrlport,ctrltsk,xy,sizeof(xy));

		
		/*Timers procedure*/
		lastT = currentT;
		total = currentT / SEC2NANO(1);
		rt_task_wait_period(); 
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	xy[0] = DUMMYPACK;
	xy[1] = DUMMYPACK;
	xy[2] = DUMMYPACK;
	printf("Sending dummy\n\r");
	RT_sendx(ctrlnode,ctrlport,ctrltsk,xy,sizeof(xy));

//#ifdef CALC_DATA
//	if ( robotCalcData(robot) < 0 ) {
//		free(robot);
//		free(sharedCp);
//		taskFinishRtaiSim(simtask);
//		return NULL;
//	}
//#endif /*CALC_DATA*/

	munlockall();
	rtnetPacketFinish(&rtnet);
	rt_make_soft_real_time();
	rt_task_delete(planttsk);
	return 0;
}
