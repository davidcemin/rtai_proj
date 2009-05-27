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
			printf("Get ref x\n\r");
			monitorGenGetX(local, shared);
			break;

		case MONITOR_SET_REFERENCE_X:
			printf("Set ref x\n\r");
			monitorGenSetX(shared, local);
			break;	
		
		case MONITOR_GET_REFERENCE_Y:
			printf("Get ref y\n\r");
			monitorGenGetY(local, shared);
			break;

		case MONITOR_SET_REFERENCE_Y:
			printf("Set ref y\n\r");
			monitorGenSetY(shared, local);
			break;

		case MONITOR_GET_YMX:
			printf("Get ymx\n\r");
			monitorRefXGet(local, shared);
			break;

		case MONITOR_SET_YMX:
			printf("Set ymx\n\r");
			monitorRefXSet(shared, local);
			break;

		case MONITOR_GET_YMY:
			printf("Get ymy\n\r");
			monitorRefYGet(local, shared);
			break;

		case MONITOR_SET_YMY:
			printf("Set ymy\n\r");
			monitorRefYSet(shared, local);
			break;

		case MONITOR_GET_V:
			monitorLitGet(local, shared);
			break;	
		
		case MONITOR_SET_V:
			monitorLinSet(shared, local);
			break;

		default:
			fprintf(stderr, "Error in monitor Main. I should not be here!\n\r");
			break;
	}

}

