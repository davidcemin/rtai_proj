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

#endif
