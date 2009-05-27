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

void robotNewYm(st_robotRefMod *refmod)
{
	int k = refmod->kIndex;
	double ts = (refmod->timeInstant[k] - refmod->timeInstant[k-1]);
	
	refmod->ym[k] = refmod->dRef[k] * ts;
}

/******************************************************************************/

void robotDxYm(st_robotRefMod *refmod, st_robotControl *local, int type)
{
	int k = refmod->kIndex;
	int alpha = local->alpha[type];

	refmod->dRef[k] = alpha * (refmod->ref[k] - refmod->ym[k]);
}

/******************************************************************************/

