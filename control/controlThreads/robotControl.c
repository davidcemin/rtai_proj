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
	st_robotControlShared *shared = ptr;
	st_robotControl *local;
	st_robotSample *sample;
	st_robotSimulPacket *simulPacket;
	st_robotSimulPacket *simRemote;
	//RT_TASK *recvSim = NULL;
	int started_timer = 0;

	int stack = sizeof(st_robotControl) + sizeof(st_robotSample) + 2 * sizeof(st_robotSimulPacket);

	//long len = 0;
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	RT_TASK *calctask = NULL;
	
	
	/* Allocates memory to robot structure */
	if ( (sample = (st_robotSample*)malloc(sizeof(sample)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to sample!\n\r");
		return NULL;
	}

	if ( (simulPacket = (st_robotSimulPacket*)malloc(sizeof(simulPacket))) == NULL) {
		fprintf(stderr, "Error in simulPack malloc!\n");
		return NULL;
	}

	if ( (simRemote = (st_robotSimulPacket*)malloc(sizeof(simRemote))) == NULL) {
		fprintf(stderr, "Error in simulPack malloc!\n");
		return NULL;
	}

	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}
	
	/*init task*/
	if( (started_timer = taskCreateRtai(calctask, CONTROLTSK, CALCPRIORITY, STEPTIMECALCNANO, stack) ) < 0) {
		fprintf(stderr, "Calculation!\n");
		free(sample);
		return NULL;
	}	
	/*sync*/
	rt_sem_wait(shared->sem.sm_control);
	rt_sem_wait(shared->sem.sm_control);
	rt_sem_signal(shared->sem.sm_lin);

	/*init rtnet*/
	//rtnetPacketInit(&shared->rtnet);

	/*init handler*/
	//rtnetTaskWait(&shared->rtnet, recvSim, SIMTSK);

	/*make it real time*/
	mkTaksRealTime(calctask, STEPTIMECALCNANO, CONTROLTSK);

	/* Pointers init*/
	memset(local, 0, sizeof(local));
	memset(sample, 0, sizeof(sample) );
	memset(simulPacket, 0, sizeof(simulPacket));

	printf("CONTROL\n\r");
	tInit = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns() - tInit;

		/*Algorithm:
		 * 1) get ymx and y'mx;
		 * 2) get ymy and y'my;
		 * 3) get y vector;
		 * 4) calculate v vector;
		 * 5) send v vector;
		 */
		/* get references */
		monitorControlMain(shared, local, MONITOR_GET_YMX);
		monitorControlMain(shared, local, MONITOR_GET_YMY);

		/* get y */
		//printf("RCTRL: node: 0x%lx port: %d\n\r", shared->rtnet.node, shared->rtnet.port);
		//RT_receivex(shared->rtnet.node, shared->rtnet.port, recvSim, (void*)simRemote->y, sizeof(simRemote->y), &len);
		//robotGetPacket(&shared->rtnet, recvSim, (void*)simulPacket->y);

		simulPacket->y[0] = 1;
		simulPacket->y[1] = 1;

		//if( len != sizeof(simRemote->y) ) {
		//	printf("len: %ld msg: %d\n\r", len, sizeof(simRemote->y));
		//}
		//else
		//	memcpy(&simulPacket->y, &simRemote->y, sizeof(simRemote->y));


		//printf("%f\t%f\n\r", simulPacket->y[0], simulPacket->y[1]);
		/*first we get alpha values, not in crictical session */
		//local->alpha[ALPHA_1] = shared->control.alpha[ALPHA_1];
		//local->alpha[ALPHA_2] = shared->control.alpha[ALPHA_2];
		local->alpha[ALPHA_1] = 1;
		local->alpha[ALPHA_2] = 1;
		
		/*then calculate v */
		robotCalcV(local, simulPacket->y);
		
		/*send v*/
		monitorControlMain(shared, local, MONITOR_SET_V);

		sample->kIndex++;

		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period();

	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	/*log data*/
	//if(	robotLogData(sample) < 0) 
	//	fprintf(stderr, "Error! It was not possible to log data!\n\r");

	taskFinishRtai(calctask, started_timer);
	return NULL;
}
