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
#include "monitorSim.h"
#include "monitorSimMain.h"
#include "robotStructs.h"

/******************************************************************************/

inline void monitorSimMain(st_robotSimulShared *shared, st_robotSimulPacket *local, int type)
{
	
	switch (type) {
		case MONITOR_GET_SIM_SHARED:
			monitorSimGet(local, shared);
			break;

		case MONITOR_SET_SIM_SHARED:
			monitorSimSet(shared, local);
			break;

		default:
			fprintf(stderr, "Error in monitor Main. I should not be here!\n\r");
			break;
	}

}


