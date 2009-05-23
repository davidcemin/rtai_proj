/******************************************************************************/
/**
 * \file monitorMain.c
 * \brief This file has the Main monitor functions
 *
 */
/******************************************************************************/

/* libc */
#include <stdio.h>

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
		case MONITOR_GET_REFERENCE:
			monitorGenGet(local, shared);
			break;

		case MONITOR_SET_REFERENCE:
			monitorGenSet(shared, local);
			break;

		case MONITOR_GET_YMX:
			monitorRefXGet(local, shared);
			break;

		case MONITOR_SET_YMX:
			monitorRefXSet(shared, local);
			break;

		case MONITOR_GET_YMY:
			monitorRefYGet(local, shared);
			break;

		case MONITOR_SET_YMY:
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


