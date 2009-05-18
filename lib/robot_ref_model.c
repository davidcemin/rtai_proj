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
	int alpha = refmod->alpha;
	double ref = refmod->ref[k];
	double ts;

	/* dx/dt = (x[n] - x[n-1])/Ts*/
	ts = (refmod->timeInstant[k] - refmod->timeInstant[k-1]);
	
	refmod->ym[k] = ( (alpha * ts * ref - refmod->ym[k-1]) / (1 + alpha * ts) );
}

/******************************************************************************/

void robotDxYm(st_robotRefMod *refmod)
{
	int k = refmod->kIndex;
	int alpha = refmod->alpha;

	refmod->dRef[k] = alpha * (refmod->ref[k] - refmod->ym[k]);
}

/******************************************************************************/

