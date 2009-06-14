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
#include "simulCalcsUtils.h"

/*rtai includes*/
#include <rtai_lxrt.h>
#include <rtai_netrpc.h>

/*****************************************************************************/

void *robotLin(void *ptr)
{
	st_robotControlStack *stack = ptr;
	st_robotControl local;
	st_robotLinPacket linPacket;
	RT_TASK *task = NULL;
	st_rtnetRobot rtnet;
	
	double currentT = 0;
	double lastT = 0;
	double total = 0;
	
	double u[U_DIMENSION];
	double xy[XY_DIMENSION];
	long len;
	int i =0;

	mlockall(MCL_CURRENT | MCL_FUTURE);	

	memset(&linPacket, 0, sizeof(linPacket));
	memset(&local, 0, sizeof(local));
	
	/*registering real time task*/
	if((task = rt_thread_init(nam2num(LINEARTSK), LINPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0){
		fprintf(stderr,"refY task init error!\n\r");
		return NULL;
	}

	unsigned long plantnode=0;
	int plantport=0;
	rtnet.ip = stack->ip;
	
	/*init rtnet*/
	rtnetPacketInit(&rtnet);
	plantport = rtnet.port;
	plantnode = rtnet.node;
	
	RT_TASK *planttsk=NULL;

	while((planttsk=(RT_TASK *)RT_get_adr(plantnode,plantport,SIMTSK))==NULL) {
		usleep(100000);
		printf("Waiting %s to be up...\n\r", SIMTSK);
	}

	rtaiMakeHard(task, LINEARTSK, STEPTIMELINNANO);

	/*sync*/
	printf("release gen\n\r");
	rt_sem_signal(stack->sem.sm_gen);
	printf("release x\n\r");
	rt_sem_signal(stack->sem.sm_refx);
	printf("release y\n\r");
	rt_sem_signal(stack->sem.sm_refy);
	printf("release ctrl\n\r");
	rt_sem_signal(stack->sem.sm_control);
	printf("release displ\n\r");
	sem_post(&stack->sem.sm_disp);

	double diff = 0;
	lastT = rt_get_time_ns();
	printf("LIN\n\r");
	do {
		currentT = rt_get_time_ns();
		diff = currentT - lastT;
		i++;
		/*Algorithm:
		 * 1) Get x;
		 * 2) Get v;
		 * 3) Generate u;
		 * 4) Send u;
		 */
		
		/* get  x and y*/
		while (RT_receivex_if(plantnode,plantport,0/*planttsk*/,xy,sizeof(xy),&len) != NULL) {
			if (xy[0] == DUMMYPACK || xy[1] == DUMMYPACK || xy[2] == DUMMYPACK) {
				printf("Dummy pack received. Breaking now..\n\r");
				break;
			}
			else if (len == sizeof(xy) ) {
				linPacket.x[0] = xy[0];
				linPacket.x[1] = xy[1];
				linPacket.x[2] = xy[2];
				local.lin_t.x[0] = xy[0];
				local.lin_t.x[1] = xy[1];
				local.lin_t.x[2] = xy[2];
				local.lin_t.y[0] = xy[3];
				local.lin_t.y[1] = xy[4];
			}
			else if (len == 2*sizeof(double)) {
				local.alpha[ALPHA_1] = xy[0];
				local.alpha[ALPHA_2] = xy[1];
				monitorControlMain(&local, MONITOR_SET_ALPHA);
			}
		}

		monitorControlMain(&local, MONITOR_GET_ALPHA);
		local.lin_t.alpha[ALPHA_1][i] = local.alpha[ALPHA_1];
		local.lin_t.alpha[ALPHA_2][i] = local.alpha[ALPHA_2];
		
		/* set y*/
		monitorControlMain(&local, MONITOR_SET_Y);

		/* Get v */
		monitorControlMain(&local, MONITOR_GET_V);
		linPacket.v[0] = local.lin_t.v[0];
		linPacket.v[1] = local.lin_t.v[1];

		/* Generate u*/
		robotGenU(&linPacket);
		
		/* send u */
		u[0] = linPacket.u[0];
		u[1] = linPacket.u[1];
		RT_sendx(plantnode,-plantport,planttsk,u,sizeof(u));
						
		total += diff / SEC2NANO(1); 
		lastT = currentT;
		
		local.lin_t.k = i;
		local.lin_t.time[i] = total;
		local.lin_t.tc[i] = rt_get_time_ns() - currentT;
		local.lin_t.period[i] = diff;
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
	printf("finishing ctrl\n\r");
	rt_sem_signal(stack->sem.sm_control);

	munlockall();
	rtnetPacketFinish(&rtnet);
	
	rt_make_soft_real_time();
	rt_task_delete(task);
	
#ifdef CALC_DATA
	robotCalcData(local.lin_t.tc, local.lin_t.k, "results_lin_te.dat");
	robotCalcData(local.lin_t.period, local.lin_t.k, "results_lin_pd.dat");
	saveToFileGeneric(FILE_LIN, &local.lin_t);
#endif /*CALC_DATA*/
	return NULL;
}

