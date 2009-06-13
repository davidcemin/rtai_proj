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
	double y1 = local->lin_t.y[0];
	double y2 = local->lin_t.y[1];
	unsigned char alpha1 = local->alpha[ALPHA_1];
	unsigned char alpha2 = local->alpha[ALPHA_2];

	//printf("%f %f %f %f %f %f\n\r",dymx, dymy, ymx, ymy, local->lin_t.y[0], local->lin_t.y[1] );

	local->lin_t.v[0] = dymx + alpha1*(ymx - y1);
	local->lin_t.v[1] = dymy + alpha2*(ymy - y2);
}

/*****************************************************************************/

void *robotControl(void *ptr)
{	
	st_robotControlStack *stack = ptr;
	st_robotControl local;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	int i = 0;
	//double tInit = 0;

	mlockall(MCL_CURRENT | MCL_FUTURE);	
	
	/* Pointers init*/
	memset(&local, 0, sizeof(local));
	
	RT_TASK *ctrltsk = NULL;
	/*registering realtime task*/
	if((ctrltsk = rt_thread_init(nam2num(CONTROLTSK), CTRLPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"gentask init error!\n\r");
		return NULL;
	}
	
	/*sync*/
	printf("C: Waiting signal.A.\n\r");
	rt_sem_wait(stack->sem.sm_control);
	
	rtaiMakeHard(ctrltsk, CONTROLTSK, STEPTIMECALCNANO);

	printf("CTRL\n\r");
	//tInit = stack->time;
	double diff = 0;
	lastT = rt_get_time_ns();
	do {
		currentT = rt_get_time_ns();
		diff = currentT - lastT;
		i++;

		/*Algorithm:
		 * 1) get ymx and y'mx;
		 * 2) get ymy and y'my;
		 * 3) get y vector;
		 * 4) calculate v vector;
		 * 5) send v vector;
		*/
		
		/* get references */
		monitorControlMain(&local, MONITOR_GET_YMX);
		monitorControlMain(&local, MONITOR_GET_YMY);

		/* get y */
		monitorControlMain(&local, MONITOR_GET_Y);
		
		/*Get alpha*/
		monitorControlMain(&local, MONITOR_GET_ALPHA);
		
		/*then calculate v */
		robotCalcV(&local);
		
		/*send v*/
		monitorControlMain(&local, MONITOR_SET_V);
		
		total += diff / SEC2NANO(1); 	
		lastT = currentT;
		local.control_t.k = i;
		local.control_t.time[i] = total;
		local.control_t.tc[i] = rt_get_time_ns() - currentT;
		//local.control_t.period[i] = diff;
		rt_task_wait_period(); 
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	

	/*sync*/
	printf("C: Waiting signal..\n\r");
	rt_sem_wait(stack->sem.sm_control);

	/*Not real time anymore*/
	rt_make_soft_real_time();
	rt_task_delete(ctrltsk);
	
#ifdef CALC_DATA
	robotCalcData(local.control_t.tc, local.control_t.k, "results_ctrl_te.dat");
	//robotCalcData(local.control_t.period, local.control_t.k, "results_ctrl_period.dat");
	saveToFileGeneric(FILE_CTRL, &local.control_t);
#endif /*CALC_DATA*/
	
	return 0;
}

