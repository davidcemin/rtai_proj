/*****************************************************************************/
/**
 * \file robotControl.c
 * \brief Control Thread and auxiliary functions
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
#include "monitorControlMain.h"
#include "robotDefines.h"
#include "robotStructs.h"
#include "robotThreads.h"
#include "simulCalcsUtils.h"
#include "robotControl.h"
#include "rtnetAPI.h"
#include "rtaiCtrl.h"

/*rtai includes*/
#include <rtai_lxrt.h>
#include <rtai_netrpc.h>

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
 * \brief  Calculates v and save into local memory
 * \param local Local memory
 * \param y Pointer to y packet
 * \return void
 */
static inline void robotCalcV(st_robotControl *local, double *y)
{
	double ymx = local->control_t.ym[XM_POSITION];
	double ymy = local->control_t.ym[YM_POSITION];
	double dymx = local->control_t.dym[XM_POSITION];
	double dymy = local->control_t.dym[YM_POSITION];
	unsigned char alpha1 = local->alpha[ALPHA_1];
	unsigned char alpha2 = local->alpha[ALPHA_2];

	local->lin_t.v[0] = dymx + alpha1*(ymx - y[0]);
	local->lin_t.v[1] = dymy + alpha2*(ymy - y[1]);
}

/*****************************************************************************/

void *robotControl(void *ptr)
{	
	st_robotControlStack *stack = ptr;
	st_robotControl *local;
	st_robotSample *sample;
	st_robotSimulPacket *simulPacket;
	st_rtnetRobot rtnet;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double u[U_DIMENSION];
	double xy[XY_DIMENSION];
	long len;

	
	/* Allocates memory to robot structure */
	if ( (sample = (st_robotSample*)malloc(sizeof(sample)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to sample!\n\r");
		return NULL;
	}

	if ( (simulPacket = (st_robotSimulPacket*)malloc(sizeof(simulPacket))) == NULL) {
		fprintf(stderr, "Error in simulPack malloc!\n");
		return NULL;
	}

	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}

	mlockall(MCL_CURRENT | MCL_FUTURE);	
	
	/* Pointers init*/
	memset(local, 0, sizeof(local));
	memset(sample, 0, sizeof(sample) );
	memset(simulPacket, 0, sizeof(simulPacket));
	
	RT_TASK *ctrltsk = NULL;
	/*registering realtime task*/
	if((ctrltsk = rt_thread_init(nam2num(CONTROLTSK), CTRLPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"gentask init error!\n\r");
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
		//printf("Cant find task %s\n", SIMTSK);
	}

	/*make it hard real time*/
	rt_make_hard_real_time();
	
	/*and finally make it periodic*/
	rt_task_make_periodic(ctrltsk, nano2count(stack->time), TIMECTRL*stack->tick);

	stack->time = rt_get_time_ns();
	printf("CTRL\n\r");
	printf("release gen\n\r");
	rt_sem_signal(stack->sem.sm_gen);
	printf("release x\n\r");
	rt_sem_signal(stack->sem.sm_refx);
	printf("release y\n\r");
	rt_sem_signal(stack->sem.sm_refy);
	printf("release lin\n\r");
	rt_sem_signal(stack->sem.sm_lin);

	do {
		currentT = rt_get_time_ns() - stack->time;

		/*Algorithm:
		 * 6) get u vector;
		 * 7) send u to sim
		 * 1) get ymx and y'mx;
		 * 2) get ymy and y'my;
		 * 3) get y vector;
		 * 4) calculate v vector;
		 * 5) send v vector;
		*/
		
		/*get u*/
		monitorControlMain(local, MONITOR_GET_U);
	
		/*send u*/
		u[0] = local->lin_t.u[0];
		u[1] = local->lin_t.u[1];
		//printf("Sending\n\r");
		/* send v to control thread*/
		RT_sendx(plantnode,-plantport,planttsk,u,sizeof(u));

		/* get xy */
		//printf("Receiving..\n\r");
		RT_receivex(plantnode,plantport,planttsk,xy,sizeof(xy),&len);

		if(len != sizeof(xy))
			printf("%ld != %d\n\r", len, sizeof(xy));
		else if (xy[0] == DUMMYPACK || xy[1] == DUMMYPACK || xy[2] == DUMMYPACK || xy[3] == DUMMYPACK) {
			printf("Dummy pack received. Breaking now..\n\r");
			break;
		}
		else{
			//printf("%f %f %f %f\n\r", xy[0], xy[1], xy[2], xy[3]);
			simulPacket->x[0] = xy[0];
			simulPacket->x[1] = xy[1];
			simulPacket->y[0] = xy[2];
			simulPacket->y[1] = xy[3];
		}
		
		/* get references */
		monitorControlMain(local, MONITOR_GET_YMX);
		monitorControlMain(local, MONITOR_GET_YMY);

		printf("%f %f\n\r", local->control_t.ym[XM_POSITION], local->control_t.ym[YM_POSITION]);

		monitorControlMain(local, MONITOR_SET_X);
		
		//TODO: Get alpha
		local->alpha[ALPHA_1] = 1;
		local->alpha[ALPHA_2] = 1;
		
		/*then calculate v */
		robotCalcV(local, simulPacket->y);
		
		/*send v*/
		monitorControlMain(local, MONITOR_SET_V);
	
		
		//printf("k: %d\n\r", sample->kIndex);
		sample->kIndex++;

		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period(); 
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	//	/*log data*/
	//if(	robotLogData(sample) < 0) 
	//	fprintf(stderr, "Error! It was not possible to log data!\n\r");
	/*send u*/

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
	rt_sem_signal(stack->sem.sm_lin);

	munlockall();
	rtnetPacketFinish(&rtnet);
	/*Not real time anymore*/
	rt_make_soft_real_time();
	rt_task_delete(ctrltsk);
	return 0;
}
