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
#include <arpa/inet.h>
#include <semaphore.h>

/*rtai includes*/
#include <rtai_sem.h>

/*robot includes*/
#include "robotDefines.h"
#include "robotStructs.h"

/******************************************************************************/

//!{@ Structures


//! Struct used to sample y
typedef struct {
	int kIndex;									//! k index
	double timeInstant[MAX_DATA_VALUE];			//! Time
	double yVal[X_DIMENSION][MAX_DATA_VALUE];	//! y array(y1, y2, y3)
} st_robotSample;

//! Struct with mutexes
typedef struct{
	pthread_mutex_t	mutexSim;
	pthread_mutex_t	mutexControl;
} st_robotMutex;

//! Struct with semaphores
typedef struct{
	SEM *rt_sem;
	sem_t disp_sem;
} st_robotSem;

//! Shared structure
//typedef	struct {
//	double yf[Y_DIMENSION];	
//	double u[U_DIMENSION];
//	st_robotMutex mutex;
//	st_robotSem	sem;
//} st_robotShared;

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
extern void robotRefGen(st_robotControl *robot, double t);

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

/**
 * \brief  Generates u vector from x and v
 * \param  linPacket Pointer to st_robotLinPacket structure
 * \return void
 */
extern void robotGenU(st_robotLinPacket *linPacket);

/******************************************************************************/

#endif //! _LIBROBOT_H

