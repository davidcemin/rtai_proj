/******************************************************************************/
/**
 * \file monitorCalc.h
 * \brief This file has the Control monitor prototypes, structs and defines
 *
 */
/******************************************************************************/

#ifndef _MONITORCALC_H
#define _MONITORCALC_H

/*rtai includes*/
#include <rtai_sem.h>

/*robot includes*/
#include "libRobot.h"

/******************************************************************************/

/**
 * \brief  
 */
extern int monitorCalcSet(st_robotShared *shared, double t);

/******************************************************************************/

/**
 * \brief  
 */
int monitorCalcGet(st_robotSample *sample, st_robotShared *shared, double t);

/******************************************************************************/

#endif
