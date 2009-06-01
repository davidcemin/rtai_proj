/*****************************************************************************/
/**
 * \file robotLin.c
 * \brief Linearization Thread and auxiliary functions
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
#include "libRobot.h"
#include "robotStructs.h"
#include "robotThreads.h"
#include "monitorControlMain.h"
#include "robotLin.h"
#include "rtnetAPI.h"
#include "rtaiCtrl.h"

/*rtai includes*/
#include <rtai_lxrt.h>
#include <rtai_netrpc.h>

/*****************************************************************************/

void *robotLin(void *ptr)
{
	st_robotControlStack *stack = ptr;
	st_robotControl *local;
	st_robotLinPacket *linPacket;
	RT_TASK *task = NULL;
	st_rtnetRobot rtnet;
	
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	
	double u[U_DIMENSION];
	double xy[XY_DIMENSION];
	long len;
	int i =0;



	if ( (linPacket = (st_robotLinPacket*)malloc(sizeof(linPacket)) ) == NULL ) {
		fprintf(stderr, "Error in linPacket malloc\n");
		return NULL;
	}	
	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}

	mlockall(MCL_CURRENT | MCL_FUTURE);	

	memset(linPacket, 0, sizeof(linPacket));
	memset(local, 0, sizeof(local));
	
	/*registering real time task*/
	if((task = rt_thread_init(nam2num(LINEARTSK), LINPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"refY task init error!\n\r");
		return NULL;
	}

	unsigned long plantnode=0;
	int plantport=0;
	
	/*init rtnet*/
	rtnetPacketInit(&rtnet);
	plantport = rtnet.port;
	plantnode = rtnet.node;
	
	RT_TASK *planttsk=NULL;
	while((planttsk=(RT_TASK *)RT_get_adr(plantnode,plantport,SIMTSK))==NULL) {
		usleep(100000);
		printf("Waiting %s to be up...\n\r", SIMTSK);
	}

	/*make it hard real time*/
	rt_make_hard_real_time();
	
	/*and finally make it periodic*/
	rt_task_make_periodic(task, nano2count(stack->time), TIMELIN*stack->tick);

	/*sync*/
	//printf("L: waiting control\n\r");
	//rt_sem_wait(stack->sem.sm_lin);
	stack->time = rt_get_time_ns();
	printf("release gen\n\r");
	rt_sem_signal(stack->sem.sm_gen);
	printf("release x\n\r");
	rt_sem_signal(stack->sem.sm_refx);
	printf("release y\n\r");
	rt_sem_signal(stack->sem.sm_refy);
	printf("release ctrl\n\r");
	rt_sem_signal(stack->sem.sm_control);

	tInit = stack->time;
	printf("LIN\n\r");
	do {
		currentT = rt_get_time_ns() - tInit;
		i++;
		/*Algorithm:
		 * 4) Send u;
		 * 1) Get x;
		 * 2) Get v;
		 * 3) Generate u;
		 */
		
		/* send u */
		u[0] = linPacket->u[0];
		u[1] = linPacket->u[1];
		printf("%f %f\n\r", u[0], u[1]);
		RT_sendx(plantnode,-plantport,planttsk,u,sizeof(u));

		/* get  x and y*/
		RT_receivex(plantnode,plantport,planttsk,xy,sizeof(xy),&len);

		if(len != sizeof(xy))
			printf("%ld != %d\n\r", len, sizeof(xy));
		else if (xy[0] == DUMMYPACK || xy[1] == DUMMYPACK || xy[2] == DUMMYPACK || xy[3] == DUMMYPACK) {
			printf("Dummy pack received. Breaking now..\n\r");
			break;
		}
		else{
			//printf("%d %f %f %f %f %f\n\r", i, xy[0], xy[1], xy[2], xy[3], xy[4]);
			linPacket->x[0] = xy[0];
			linPacket->x[1] = xy[1];
			linPacket->x[2] = xy[2];
			local->lin_t.y[0] = xy[3];
			local->lin_t.y[1] = xy[4];
		}
	
		/* set y*/
		monitorControlMain(local, MONITOR_SET_Y);
	
		/* Get v */
		monitorControlMain(local, MONITOR_GET_V);
		linPacket->v[0] = local->lin_t.v[0];
		linPacket->v[1] = local->lin_t.v[1];

		/* Generate u*/
		robotGenU(linPacket);
		//printf("%f %f %f\n\r", linPacket->u[0], linPacket->u[1], linPacket->x[2]);
				
		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period();
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	u[0] = DUMMYPACK;
	u[1] = DUMMYPACK;
	printf("Sending dummy\n\r");
	/* send v to control thread*/
	RT_sendx(plantnode,plantport,planttsk,u,sizeof(u));

	printf("finishing gen\n\r");
	rt_sem_signal(stack->sem.sm_gen);
	printf("finishing x\n\r");
	rt_sem_signal(stack->sem.sm_refx);
	printf("finishing y\n\r");
	rt_sem_signal(stack->sem.sm_refy);
	printf("finishing lin\n\r");
	rt_sem_signal(stack->sem.sm_control);

	munlockall();
	rtnetPacketFinish(&rtnet);
	
	rt_make_soft_real_time();
	rt_task_delete(task);
	return NULL;
}

