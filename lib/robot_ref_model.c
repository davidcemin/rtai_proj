/******************************************************************************/
/**
 * \file robotRefModel.c
 * \brief Functions to calculate the reference models.
 */
/******************************************************************************/

/* robot */
#include "robotDefines.h"
#include "robotStructs.h"

/******************************************************************************/

inline void robotSetYm(st_robotControl *local, double ts, int type)
{
	double tsA = ts * local->alpha[type];
	
	local->control_t.ym[type] = (tsA / (tsA + 1)) * local->generation_t.ref[type] + (1/(tsA+1))*local->refmod_t.ymLast;

	//printf("%s %f %f\n\r", type == XREF_POSITION ? "xref" : "yref", tsA, local->control_t.ym[type]);
}

/******************************************************************************/

inline void robotDxYm(st_robotControl *local, int type)
{
	int alpha = local->alpha[type];
	double ref = local->generation_t.ref[type];
	double ym = local->control_t.ym[type];

	local->control_t.dym[type] = alpha*(ref - ym);
}

/******************************************************************************/

