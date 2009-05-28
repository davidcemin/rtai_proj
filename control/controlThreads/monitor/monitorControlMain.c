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

inline void monitorControlMain(st_robotControlShared *shared, st_robotControl *local, int type)
{

	switch (type) {
		case MONITOR_GET_REFERENCE_X:
			if( !(rt_sem_wait(shared->sem.sm_refx) ) )
				break;
			monitorGenGetX(local, shared);
			break;

		case MONITOR_SET_REFERENCE_X:
			monitorGenSetX(shared, local);
			rt_sem_signal(shared->sem.sm_refx);
			break;	
		
		case MONITOR_GET_REFERENCE_Y:
			if( !(rt_sem_wait(shared->sem.sm_refy) ) )
				break;
			monitorGenGetY(local, shared);
			break;

		case MONITOR_SET_REFERENCE_Y:
			monitorGenSetY(shared, local);
			rt_sem_signal(shared->sem.sm_refy);
			break;

		case MONITOR_GET_YMX:
			if( !(rt_sem_wait(shared->sem.sm_control)) )
				break;
			monitorRefXGet(local, shared);
			break;

		case MONITOR_SET_YMX:
			monitorRefXSet(shared, local);
			rt_sem_signal(shared->sem.sm_control);
			break;

		case MONITOR_GET_YMY:
			if( !(rt_sem_wait(shared->sem.sm_control) ) )
			monitorRefYGet(local, shared);
			break;

		case MONITOR_SET_YMY:
			monitorRefYSet(shared, local);
			rt_sem_signal(shared->sem.sm_control);
			break;

		case MONITOR_GET_V:
			if( !(rt_sem_wait(shared->sem.sm_lin) ) )
				break;
			monitorLinGet(local, shared);
			break;	
		
		case MONITOR_SET_V:
			monitorLinSet(shared, local);
			rt_sem_signal(shared->sem.sm_lin);
			break;
	
		case MONITOR_GET_U:
			if( !(rt_sem_wait(shared->sem.sm_lin) ) )
				break;
			monitorLinGetU(local, shared);
			break;	
		
		case MONITOR_SET_U:
			monitorLinSet(shared, local);
			rt_sem_signal(shared->sem.sm_lin);
			break;	
		
		case MONITOR_GET_X:
			if( !(rt_sem_wait(shared->sem.sm_lin) ) )
				break;
			monitorLinGetX(local, shared);
			break;	
		
		case MONITOR_SET_X:
			monitorLinSetX(shared, local);
			rt_sem_signal(shared->sem.sm_lin);
			break;

		default:
			fprintf(stderr, "Error in monitor Main. I should not be here!\n\r");
			break;
	}

}

