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
#include "robotStructs.h"
#include "robotThreads.h"
#include "simulCalcsUtils.h"
#include "robotControl.h"
#include "rtnetAPI.h"

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
 * \brief  It creates a rtai task
 * \param  task Pointer to task 
 * \param  taskName Task's name
 * \param  priority Task's priority
 * \param  stepTick Step timer
 * \return -1 error, 0 ok.
 */
static inline int taskCreateRtai(RT_TASK *task, unsigned long taskName, char priority, double stepTick) 
{
	int period;
	int stkSize;
	int msgSize;

	/*set root permissions to user space*/
	rt_allow_nonroot_hrt();

	/*Es ist nicht noetig um die priority im sched_param structure anzusetzen, da 
	 * es bereits im rt_task_init_schmod Function angesetzt ist.
	 */
	
	/*It Prevents the memory to be paged*/
    mlockall(MCL_CURRENT | MCL_FUTURE);
	
	msgSize = sizeof(st_robotControlShared);
	stkSize = msgSize + 10000;

	if(!(task = rt_task_init_schmod(taskName, priority, stkSize, msgSize, SCHED_FIFO, 0xff) ) ) {	
		fprintf(stderr, "Cannot Init Task: ");
		return -1;
	}

	/*make it hard real time*/	
    rt_make_hard_real_time();

	/*set the period according to our desired tick*/
	period = (int)nano2count((RTIME)stepTick);

	/*start the task*/
	rt_task_make_periodic(task, rt_get_time()+period, period);

	return 0;
}

/*****************************************************************************/

/**
 * \brief  It destroys a rtai task
 * \param  task pointer to task to be destroyed
 * \return void
 */
static inline void taskFinishRtai(RT_TASK *task)
{
	/* Here also is not necessary to use rt_make_soft_real_time because rt_task_delete 
	 * already use it(base/include/rtai_lxrt.h:842)
	 */
	rt_task_delete(task);
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
	st_rtnetReceive *recvSim;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;

	RT_TASK *calctask = NULL;
	unsigned long calctask_name = nam2num("CTRLTASK");
	
	/* Allocates memory to robot structure */
	if ( (sample = (st_robotSample*)malloc(sizeof(sample)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to sample!\n\r");
		return NULL;
	}

	if ( (simulPacket = (st_robotSimulPacket*)malloc(sizeof(simulPacket))) == NULL) {
		fprintf(stderr, "Error in simulPack malloc!\n");
		return NULL;
	}

	if ( (recvSim = (st_rtnetReceive*)malloc(sizeof(recvSim))) == NULL ) {
		fprintf(stderr, "Not possible to allocate memory to recvSim packet\n\r");
		return NULL;
	}	
	
	if ( (local = (st_robotControl*)malloc(sizeof(local))) == NULL ) {
		fprintf(stderr, "Error in generation structure memory allocation\n");
		free(local);
		return NULL;
	}

	if(taskCreateRtai(calctask, calctask_name, CALCPRIORITY, STEPTIMECALCNANO) < 0){
		fprintf(stderr, "Calculation!\n");
		free(sample);
		return NULL;
	}

	/* Pointers init*/
	memset(local, 0, sizeof(local));
	memset(sample, 0, sizeof(sample) );
	memset(simulPacket, 0, sizeof(simulPacket));
	rtnetRecvPacketInit(recvSim, "SIMTASK");


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
		if (robotGetPacket(recvSim, (void*)simulPacket->y) < 0) {
			simulPacket->y[XM_POSITION] = 0;
			simulPacket->y[YM_POSITION] = 0;
		}

		/*first we get alpha values, not in crictical session */
		local->alpha[ALPHA_1] = shared->control.alpha[ALPHA_1];
		local->alpha[ALPHA_2] = shared->control.alpha[ALPHA_2];
		
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

	printf("c1\n\r");
	rtnetRecvPacketFinish(recvSim);
	printf("c2\n\r");
	taskFinishRtai(calctask);
	printf("c3\n\r");
	free(sample);
	printf("c4\n\r");
	return NULL;
}
