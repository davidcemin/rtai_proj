/******************************************************************************/
/**
 * \file simulCalcsUtils.h
 * \brief Functions used in the simulation 
 */
/******************************************************************************/

#ifndef _SIMULCALCUTILS_H
#define _SIMULCALCUTILS_H

#include "libRobot.h"

/******************************************************************************/

/**
 * \brief  It returns the current time in miliseconds
 * \return void
 */
extern double getTimeMilisec(void);

/******************************************************************************/

/**
 * \brief  Init the robot structure
 * \param robotInit Pointer to st_robotMainArrays structure
 * \return void
 */
extern void robotInit(st_robotMainArrays *robotInit);

/******************************************************************************/

/**
 * \brief  It calculates the inputs
 * \param  robot Pointer to shared memory
 * \param  t current time
 * \return st_robotInput structure
 */
extern void robotInputCalc(st_robotShared *robot, double t);

/******************************************************************************/

/**
 * \brief Calculate mean, variance and jitter
 * \param robot pointer to robot structure
 * \return 0 ok, -1 error
 */
extern int robotCalcData(st_robotMainArrays *robot);

/******************************************************************************/

/**
 * \brief  Gets u array from shared memory
 * \param  robot Pointer to st_robotMainArrays memory
 * \param  shared Pointer to st_robotShared memory
 * \return void
 */
extern inline void getUFromShared(st_robotMainArrays *robot, st_robotShared *shared);

/******************************************************************************/

/**
 * \brief  Copy yf array into shared memory
 * \param  robot Pointer to st_robotMainArrays memory
 * \param  shared Pointer to st_robotShared memory
 * \return void
 */
extern inline void cpYIntoShared(st_robotMainArrays *robot, st_robotShared *shared);

/******************************************************************************/

/**
 * \brief  It samples yf array to later save it in a file
 * \param  shared Pointer to st_robotShared memory
 * \param  sample Pointer to st_robotSample memory
 * \param  t Current simulation time
 * \return void
 */
extern inline void robotSampleYf(st_robotShared *shared, st_robotSample *sample, double t);

/******************************************************************************/

#endif
