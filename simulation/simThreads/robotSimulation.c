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
#include "simulCalcsUtils.h"

/*rtai includes*/
#include <rtai_netrpc.h>
#include <rtai_lxrt.h>


/*****************************************************************************/

void *robotSimulation(void *ptr)
{	
	st_robotSimulStack *stack = ptr;
	st_robotSimulPacket pack;
	st_robotMain data;

	double currentT = 0;
	double lastT = 0;
	double total = 0;
	long len;
	double u[U_DIMENSION];
	double xy[XY_DIMENSION];
		
	mlockall(MCL_CURRENT | MCL_FUTURE);	

	/*pointers init*/
	memset(&pack, 0, sizeof(pack));

	RT_TASK *planttsk = NULL;
	
	/*registering realtime task*/
	if((planttsk = rt_thread_init(nam2num(SIMTSK), SIMPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0) {
		printf("sim task init error\n\r");
		return NULL;
	}

	st_rtnetRobot rtnet;
	unsigned long ctrlnode = 0;
	int ctrlport=0;
	rtnet.ip = stack->ip;
	
	/*init rtnet*/
	rtnetPacketInit(&rtnet);
	ctrlport = rtnet.port;
	ctrlnode = rtnet.node;
	
	RT_TASK *ctrltsk=NULL;
	
	while((ctrltsk=(RT_TASK *)RT_get_adr(ctrlnode,ctrlport,LINEARTSK))==NULL) {
		usleep(100000);
		printf("Waiting %s to be up...\n\r", LINEARTSK);
	}

	rtaiMakeHard(planttsk, LINEARTSK, STEPTIMESIMNANO);

	pack.x[0] = 0;
	pack.x[1] = 1;
	pack.x[2] = 0;
	data.x[0][0] = 0;
	data.x[1][0] = 1;
	data.x[2][0] = 0;
	data.u[0][0] = 0;
	data.u[1][0] = 0;
	data.k = 0;
	sem_post(&stack->sm_disp);

	int i = 0;
	double diff = 0;
	//double tInit = stack->time;
	lastT = rt_get_time_ns();
	printf("SIMULATION\n\r");
	do {
		currentT = rt_get_time_ns();
		diff = currentT - lastT;
		i++;
		/* Algorithm:
		 * 1) get u from lin thread;
		 * 2) calculate new x value;
		 * 3) calculate x' from x and u;
		 * 4) calculate y from x and u
		 * 5) send x and y to control
		 */
		
		/* send xy to control thread*/
		xy[0] = pack.x[0];
		xy[1] = pack.x[1];
		xy[2] = pack.x[2];
		xy[3] = pack.y[0];
		xy[4] = pack.y[1];
		RT_sendx(ctrlnode,-ctrlport,ctrltsk,xy,sizeof(xy));

		/* get u from lin thread*/
		RT_receivex_if(ctrlnode,ctrlport,ctrltsk,u,sizeof(u),&len);

		if (u[0] == DUMMYPACK || u[1] == DUMMYPACK) {
			printf("Dummy packet received. Breaking now..\n\r");
			break;
		}
		else if (len == sizeof(u)) {
			/*Copy u into robot structure*/
			pack.u[0] = u[0];
			pack.u[1] = u[1];
		}
		/* New X value */
		robot_sim_calc_x(&pack, i, (currentT - lastT)/SEC2NANO(1));

		memcpy(&pack.xl, &pack.x, sizeof(pack.x));

		/* Calculates x' from x and u*/
		robot_sim_calc_dx(&pack, i);

		/* Calculates y from x*/
		robot_sim_calc_y(&pack, i);
					
		/*monitor set shared to display*/
		monitorSimMain(&pack, MONITOR_SET_SIM_SHARED);
				
		/*Timers procedure*/
		total += diff / SEC2NANO(1);
		lastT = currentT;
		
		data.k = i;
		data.time[i] = total;
		data.period[i] = diff;
		data.tc[i] = (rt_get_time_ns() - currentT);
		data.x[0][i] = pack.x[0];
		data.x[1][i] = pack.x[1];
		data.x[2][i] = pack.x[2];
		data.y[0][i] = pack.y[0];
		data.y[1][i] = pack.y[1];
		data.u[0][i] = pack.u[0];
		data.u[1][i] = pack.u[1];
		rt_task_wait_period(); 
	} while ( (fabs(total) <= (double)TOTAL_TIME) );
	
	xy[0] = DUMMYPACK;
	xy[1] = DUMMYPACK;
	xy[2] = DUMMYPACK;
	xy[3] = DUMMYPACK;
	xy[4] = DUMMYPACK;
	printf("Sending dummy\n\r");
	RT_sendx(ctrlnode,-ctrlport,ctrltsk,xy,sizeof(xy));

	rtnetPacketFinish(&rtnet);
	rt_make_soft_real_time();
	munlockall();
	rt_task_delete(planttsk);

#ifdef CALC_DATA
	robotCalcData(data.tc, data.k, "results_sim_texec.dat");
	robotCalcData(data.period, data.k, "results_sim_period.dat");
	saveToFileGeneric(FILE_SIM, &data);
#endif /*CALC_DATA*/

	return 0;
}
