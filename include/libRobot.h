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
#include <semaphore.h>

/*rtai includes*/
#include <rtai_sem.h>

/*robot includes*/
#include "robotDefines.h"

/******************************************************************************/

//!{@ Structures

//! Structure used in reference generation
typedef struct {
	double xref;
	double yref;
	double ym;
	double dym;
} st_robotGeneration;

//! Structure with reference models
typedef struct {
	double timeInstant[MAX_DATA_VALUE];	//! Current time
	double ref[MAX_DATA_VALUE];			//! x and y reference
	double ym[MAX_DATA_VALUE];			//! ymx and ymy
	double dRef[MAX_DATA_VALUE];		//! ref'
	int alpha;							//! alpha used
	int kIndex;							//! current index
} st_robotRefMod;

//! Main structure with arrays
typedef struct {
	int kIndex;										//! k index
	double timeInstant[MAX_DATA_VALUE];			//! Time
	double dxVal[XY_DIMENSION][MAX_DATA_VALUE];	//! dx array(x1', x2', x3')
	double xVal[X_DIMENSION][MAX_DATA_VALUE];		//! x array(x1, x2, x3)
	double yVal[Y_DIMENSION][MAX_DATA_VALUE];		//! y array(y1, y2, y3)
	double uVal[U_DIMENSION][MAX_DATA_VALUE];		//! u array(v, w)
} st_robotMainArrays;

//! Struct used to sample y
typedef struct {
	int kIndex;									//! k index
	double timeInstant[MAX_DATA_VALUE];			//! Time
	double yVal[XY_DIMENSION][MAX_DATA_VALUE];	//! y array(y1, y2, y3)
} st_robotSample;

//! Struct with mutexes
typedef struct{
	pthread_mutex_t	mutexSim;
	pthread_mutex_t	mutexCalc;
} st_robotMutex;

//! Struct with semaphores
typedef struct{
	SEM *rt_sem;
	sem_t disp_sem;
} st_robotSem;

//! Shared structure
typedef	struct {
	double yf[3];	//! Output array
	double u[2];	//! Input array 
	st_robotMutex mutex;
	st_robotSem	sem;
} st_robotShared;

//! Robot generation shared strucuture
typedef struct {
	st_robotGeneration genShared;
	pthread_mutex_t	mutexGen;
} st_robotGenerationShared;

/******************************************************************************/

/**
 * \brief  It calculates the output y from x
 * \param  robotMain Pointer to st_robotMainArrays structure
 * \return void
 */
extern void robotCalcYFromX(st_robotMainArrays *robotMain);

/******************************************************************************/

/**
 * \brief  It calculates x' from x
 * \param  robotMain Pointer to st_robotMainArrays structure
 * \return void
 */
extern void robotDxSim(st_robotMainArrays *robotMain);

/******************************************************************************/

/**
 * \brief  Calculates x from current x and x'
 * \param  robotMain Pointer to st_robotMainArrays structure
 * \return void
 */
extern void robotNewX(st_robotMainArrays *robotMain);

/******************************************************************************/

/**
 * \brief It generates the refereces xref and yref according to the project's specification
 * \param robot Pointer to st_robotGeneration structure
 * \param t current time
 * \return void 
 */
extern void robotRefGen(st_robotGeneration *robot, double t);

/******************************************************************************/

/**
 * \brief  It calculates new y reference model
 * \param  refmod Pointer to st_robotRefMod structure
 * \return void
 */
extern void robotNewYm(st_robotRefMod *refmod);

/******************************************************************************/

/**
 * \brief  It calculates the first derivative from ym
 * \param  refmod Pointer to st_robotRefMod structure
 * \return void
 */
extern void robotDxYm(st_robotRefMod *refmod);

/******************************************************************************/

#endif //! _LIBROBOT_H

