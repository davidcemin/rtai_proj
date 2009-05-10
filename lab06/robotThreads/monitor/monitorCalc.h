/******************************************************************************/
/**
 * \file monitorCalc.h
 * \brief This file has the Control monitor prototypes, structs and defines
 *
 */
/******************************************************************************/

#ifndef _MONITORCALC_H
#define _MONITORCALC_H

/*robot includes*/
#include "libRobot.h"

/******************************************************************************/

/**
 * \brief This function calculates the inputs u and copy it into shared
 * \param shared Pointer to shared structure
 * \param t current time
 * \return 0
 */
extern int monitorCalcSet(st_robotShared *shared, double t);

/******************************************************************************/

/**
 * \brief  This function copies the shared into a local memory.
 * \param  sample Pointer to st_robotSample structure.
 * \param  shared pointer to shared memory
 * \param  t current time.
 */
int monitorCalcGet(st_robotSample *sample, st_robotShared *shared, double t);

/******************************************************************************/

#endif
