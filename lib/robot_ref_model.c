/******************************************************************************/
/**
 * \file robotRefModel.c
 * \brief Functions to calculate the reference models.
 */
/******************************************************************************/

/* robot */
#include "robotDefines.h"
#include "libRobot.h"

/******************************************************************************/

void robotNewYm(st_robotRefMod *refmod)
{
	int k = refmod->kIndex;
	double ts;

	/* ymx(k) = y'(k) * T*/
	ts = (refmod->timeInstant[k] - refmod->timeInstant[k-1]);

	refmod->ym[k] = refmod->dRef[k-1] * ts;
}

/******************************************************************************/

void robotDxYm(st_robotRefMod *refmod)
{
	int k = refmod->kIndex;
	int alpha = refmod->alpha;

	refmod->dRef[k] = alpha * (refmod->ref[k] - refmod->ym[k]);
}

/******************************************************************************/

