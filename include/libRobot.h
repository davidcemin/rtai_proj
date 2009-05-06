/******************************************************************************/
/**
* \file libRobot.h
* \brief  Library file. Functions used in several programs.
*/
/******************************************************************************/

#ifndef _LIBROBOT_H
#define _LIBROBOT_H

/*linux includes*/
#include <pthread.h>

/*robot includes*/
#include "robotDefines.h"

//! Create a single type to the variables. Easy to change it later.
typedef double varType;

//! Main structure with arrays
typedef struct {
	int kIndex;										//! k index
	varType timeInstant[MAX_DATA_VALUE];			//! Time
	varType dxVal[XY_DIMENSION][MAX_DATA_VALUE];	//! dx array(x1', x2', x3')
	varType xVal[XY_DIMENSION][MAX_DATA_VALUE];		//! x array(x1, x2, x3)
	varType yVal[XY_DIMENSION][MAX_DATA_VALUE];		//! y array(y1, y2, y3)
	varType uVal[U_DIMENSION][MAX_DATA_VALUE];		//! u array(v, w)
} st_robotMainArrays;

//! Struct used to sample y
typedef struct {
	int kIndex;										//! k index
	varType timeInstant[MAX_DATA_VALUE];			//! Time
	varType yVal[XY_DIMENSION][MAX_DATA_VALUE];		//! y array(y1, y2, y3)
} st_robotSample;

//! Robot thread structure used inside shared
typedef struct {
	ptrhead_mutex_t	mutexSim;
	ptrhead_mutex_t mutexCalc;
	pthread_cond_t	simEmpty;
	pthread_cond_t	calcEmpty;
} st_robotPthread;

//! Shared structure
typedef	struct {
	varType yf[3];	//! Output array
	varType u[2];	//! Input array 
	st_robotPthread	robotThread; //! Robot shared thread structure
} st_robotShared;

/******************************************************************************/
/**
 * \brief  It calculates the output y from x
 * \param  *robotMain Pointer to st_robotMainArrays structure
 * \return void
 */
extern void robotCalcYFromX(st_robotMainArrays *robotMain);

/******************************************************************************/
/**
 * \brief  It calculates x' from x
 * \param  *robotMain Pointer to st_robotMainArrays structure
 * \return void
 */
extern void robotDxSim(st_robotMainArrays *robotMain);

/******************************************************************************/
/**
 * \brief  Calculates x from current x and x'
 * \param  *robotMain Pointer to st_robotMainArrays structure
 * \return void
 */
extern void robotNewX(st_robotMainArrays *robotMain);

/******************************************************************************/

#endif //! _LIBROBOT_H

