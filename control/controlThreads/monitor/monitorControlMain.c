/******************************************************************************/
/**
 * \file monitorMain.c
 * \brief This file has the Main monitor functions
 *
 */
/******************************************************************************/

/* libc */
#include <stdio.h>
#include <rtai_sem.h>

/* robot */
#include "monitorControlMain.h"
#include "monitorGen.h"
#include "monitorLin.h"
#include "monitorRefModels.h"
#include "robotDefines.h"
#include "robotStructs.h"

/******************************************************************************/

/**
 * \brief  Control Shared structure
 */
static st_robotControlShared Shared;

/******************************************************************************/

void robotControlSharedInit(void)
{
	memset(&Shared.control, 0, sizeof(Shared.control));

	Shared.control.alpha[ALPHA_1] = 1;
	Shared.control.alpha[ALPHA_2] = 1;
	
	pthread_mutex_init(&Shared.mutex.mutexControl, NULL);
	pthread_mutex_init(&Shared.mutex.mutexGen, NULL);
	pthread_mutex_init(&Shared.mutex.mutexLin, NULL);

	Shared.sem.sm_control = rt_sem_init(nam2num("sma"), 0);
	Shared.sem.sm_gen = rt_sem_init(nam2num("smb"), 0);
	Shared.sem.sm_lin = rt_sem_init(nam2num("smc"), 0);
	Shared.sem.sm_refx = rt_sem_init(nam2num("smd"), 0);
	Shared.sem.sm_refy = rt_sem_init(nam2num("sme"), 0);
}

/******************************************************************************/

void robotControlSharedFinish(void)
{
	pthread_mutex_destroy(&Shared.mutex.mutexControl);
	pthread_mutex_destroy(&Shared.mutex.mutexGen);
	pthread_mutex_destroy(&Shared.mutex.mutexLin);
	
	rt_sem_delete(Shared.sem.sm_control);
	rt_sem_delete(Shared.sem.sm_gen);
	rt_sem_delete(Shared.sem.sm_lin);
	rt_sem_delete(Shared.sem.sm_refx);
	rt_sem_delete(Shared.sem.sm_refy);
}

/******************************************************************************/

inline void monitorControlMain(st_robotControl *local, int type)
{

	switch (type) {
		case MONITOR_GET_REFERENCE_X:
			monitorGenGetX(local, &Shared);
			break;

		case MONITOR_SET_REFERENCE_X:
			monitorGenSetX(&Shared, local);
			break;	
		
		case MONITOR_GET_REFERENCE_Y:
			monitorGenGetY(local, &Shared);
			break;

		case MONITOR_SET_REFERENCE_Y:
			monitorGenSetY(&Shared, local);
			break;

		case MONITOR_GET_YMX:
			monitorRefXGet(local, &Shared);
			break;

		case MONITOR_SET_YMX:
			monitorRefXSet(&Shared, local);
			break;

		case MONITOR_GET_YMY:
			monitorRefYGet(local, &Shared);
			break;

		case MONITOR_SET_YMY:
			monitorRefYSet(&Shared, local);
			break;

		case MONITOR_GET_V:
			monitorLinGetV(local, &Shared);
			break;	
		
		case MONITOR_SET_V:
			monitorLinSetV(&Shared, local);
			break;
	
		case MONITOR_GET_Y:
			monitorLinGetY(local, &Shared);
			break;	
		
		case MONITOR_SET_Y:
			monitorLinSetY(&Shared, local);
			break;	
		
		default:
			fprintf(stderr, "Error in monitor Main. I should not be here!\n\r");
			break;
	}
}

