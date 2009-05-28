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
	st_robotControlShared *shared = (st_robotControlShared*)ptr;
	st_robotControl *local;
	st_robotSample *sample;
	st_robotSimulPacket *simulPacket;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	
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

	/* Pointers init*/
	memset(local, 0, sizeof(local));
	memset(sample, 0, sizeof(sample) );
	memset(simulPacket, 0, sizeof(simulPacket));

 	RT_TASK *ctrltsk = NULL;
	int started_timer = 0;

	/*init task*/
	if( (started_timer = taskCreateRtai(ctrltsk, CONTROLTSK, CALCPRIORITY, STEPTIMECALCNANO, 0) ) < 0) {
		fprintf(stderr, "Calculation!\n");
	//	free(sample);
		return NULL;
	}	

	unsigned long plantnode=0;
	int plantport=0;
	/*sync*/
//	//rt_sem_wait(shared->sem.sm_control);
//	//rt_sem_wait(shared->sem.sm_control);

	/*init rtnet*/
	rtnetPacketInit(&shared->rtnet);
	plantport = shared->rtnet.port;
	plantnode = shared->rtnet.node;
	
	RT_TASK *planttsk=NULL;
	while((planttsk=(RT_TASK *)RT_get_adr(plantnode,plantport,SIMTSK))==NULL) {
		usleep(100000);
		//printf("Cant find task %s\n", SIMTSK);
	}

	/*make it real time*/
	mkTaksRealTime(ctrltsk, STEPTIMECALCNANO, CONTROLTSK);

	//rt_sem_signal(shared->sem.sm_gen);
	//rt_sem_signal(shared->sem.sm_refx);
	//rt_sem_signal(shared->sem.sm_refy);
	//rt_sem_signal(shared->sem.sm_lin);

	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;

		/*Algorithm:
		 * 1) get ymx and y'mx;
		 * 2) get ymy and y'my;
		 * 3) get y vector;
		 * 4) calculate v vector;
		 * 5) send v vector;
		 * 6) get u vector;
		 * 7) send u to sim
		 */
		/* get references */
		//monitorControlMain(shared, local, MONITOR_GET_YMX);
		//monitorControlMain(shared, local, MONITOR_GET_YMY);
		long len;
		double xy[4];
	
		/* get xy */
		printf("Receiving..\n\r");
		RT_receivex(plantnode,plantport,planttsk,xy,sizeof(xy),&len);

		if(len != sizeof(xy))
			printf("%ld != %d\n\r", len, sizeof(xy));
		else{
			printf("%f %f %f %f\n\r", xy[0], xy[1], xy[2], xy[3]);
			simulPacket->x[0] = xy[0];
			simulPacket->x[1] = xy[1];
			simulPacket->y[0] = xy[2];
			simulPacket->y[1] = xy[3];
		}
		
		//monitorControlMain(shared, local, MONITOR_SET_X);
		
		//TODO: Get alpha
		//local->alpha[ALPHA_1] = shared->control.alpha[ALPHA_1];
		//local->alpha[ALPHA_2] = shared->control.alpha[ALPHA_2];
		local->alpha[ALPHA_1] = 1;
		local->alpha[ALPHA_2] = 1;
		
		/*then calculate v */
		robotCalcV(local, simulPacket->y);
		
		/*send v*/
		//monitorControlMain(shared, local, MONITOR_SET_V);
	
		/*get u*/
		//monitorControlMain(shared, local, MONITOR_GET_U);
	
		/*send u*/
		double u[2];
		
		printf("Sending\n\r");
		/* send v to control thread*/
		RT_sendx(plantnode,plantport,planttsk,u,sizeof(u));

		sample->kIndex++;

		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period(); 
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	//	/*log data*/
	//if(	robotLogData(sample) < 0) 
	//	fprintf(stderr, "Error! It was not possible to log data!\n\r");

	rtnetPacketFinish(&shared->rtnet);
	taskFinishRtai(ctrltsk, started_timer);

	return 0;
}
