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
#include "monitorCtrlAlpha.h"
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
	int i;
	struct {
		pthread_mutex_t mutex;
	} tableInit [] = {
		{Shared.mutex.mutexRefX  },
		{Shared.mutex.mutexRefY  },
		{Shared.mutex.mutexYmx   },
		{Shared.mutex.mutexYmy   },
		{Shared.mutex.mutexV     },
		{Shared.mutex.mutexY     },
		{Shared.mutex.mutexAlpha },
	};

	memset(&Shared.control, 0, sizeof(Shared.control));

	Shared.control.alpha[ALPHA_1] = ALPHA_1_INIT;
	Shared.control.alpha[ALPHA_2] = ALPHA_2_INIT;

	for (i = 0; i < NMEMB(tableInit); i++)
		pthread_mutex_init(&tableInit[i].mutex, NULL);
}

/******************************************************************************/

void robotControlSharedFinish(void)
{	
	int i;
	struct {
		pthread_mutex_t mutex;
	} tableInit [] = {
		{Shared.mutex.mutexRefX  },
		{Shared.mutex.mutexRefY  },
		{Shared.mutex.mutexYmx   },
		{Shared.mutex.mutexYmy   },
		{Shared.mutex.mutexV     },
		{Shared.mutex.mutexY     },
		{Shared.mutex.mutexAlpha },
	};

	for (i = 0; i < NMEMB(tableInit); i++)
		pthread_mutex_destroy(&tableInit[i].mutex);
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
		
		case MONITOR_GET_ALPHA:
			monitorGetAlpha(&Shared, local);
			break;	

		case MONITOR_SET_ALPHA:
			monitorSetAlpha(local, &Shared);
			break;	
		
		default:
			fprintf(stderr, "Error in monitor Main. I should not be here!\n\r");
			break;
	}
}

