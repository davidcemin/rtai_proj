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
 * \return void
 */
static inline void robotCalcV(st_robotControl *local)
{
	double ymx = local->control_t.ym[XM_POSITION];
	double ymy = local->control_t.ym[YM_POSITION];
	double dymx = local->control_t.dym[XM_POSITION];
	double dymy = local->control_t.dym[YM_POSITION];
	unsigned char alpha1 = local->alpha[ALPHA_1];
	unsigned char alpha2 = local->alpha[ALPHA_2];

	//printf("%f %f %f %f %f %f\n\r",dymx, dymy, ymx, ymy, local->lin_t.y[0], local->lin_t.y[1] );

	local->lin_t.v[0] = dymx + alpha1*(ymx - local->lin_t.y[0]);
	local->lin_t.v[1] = dymy + alpha2*(ymy - local->lin_t.y[1]);
}

/*****************************************************************************/

void *robotControl(void *ptr)
{	
	st_robotControlStack *stack = ptr;
	st_robotControl *local;
	st_robotSample *sample;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	double tInit = 0;
	
	/* Allocates memory to robot structure */
	if ( (sample = (st_robotSample*)malloc(sizeof(sample)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to sample!\n\r");
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
	
	RT_TASK *ctrltsk = NULL;
	/*registering realtime task*/
	if((ctrltsk = rt_thread_init(nam2num(CONTROLTSK), CTRLPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"gentask init error!\n\r");
		return NULL;
	}
	
	/*sync*/
	printf("G: Waiting signal..\n\r");
	rt_sem_wait(stack->sem.sm_control);
	
	/*make it hard real time*/
	rt_make_hard_real_time();
	
	/*and finally make it periodic*/
	rt_task_make_periodic(ctrltsk, nano2count(stack->time), TIMECTRL*stack->tick);

	printf("CTRL\n\r");
	tInit = stack->time;
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
		monitorControlMain(local, MONITOR_GET_YMX);
		monitorControlMain(local, MONITOR_GET_YMY);

		/* get y */
		monitorControlMain(local, MONITOR_GET_Y);
		//printf("%d %f %f\n\r", sample->kIndex, local->lin_t.y[0], local->lin_t.y[1]);
		
		//TODO: Get alpha
		local->alpha[ALPHA_1] = 1;
		local->alpha[ALPHA_2] = 1;
		
		/*then calculate v */
		robotCalcV(local);
		
		/*send v*/
		monitorControlMain(local, MONITOR_SET_V);
		
		sample->kIndex++;

		lastT = currentT;
		total = currentT / SEC2NANO(1); 	
		rt_task_wait_period(); 
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	//	/*log data*/
	//if(	robotLogData(sample) < 0) 
	//	fprintf(stderr, "Error! It was not possible to log data!\n\r");
	/*send u*/

	/*sync*/
	printf("G: Waiting signal..\n\r");
	rt_sem_wait(stack->sem.sm_control);

	/*Not real time anymore*/
	rt_make_soft_real_time();
	rt_task_delete(ctrltsk);
	return 0;
}

