/******************************************************************************/
/**
 * \file simulCalcsUtils.h
 * \brief Functions used in the simulation 
 */
/******************************************************************************/

#ifndef _SIMULCALCUTILS_H
#define _SIMULCALCUTILS_H

/* robot include */
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
 * \brief Calculate mean, variance and jitter
 * \param tc Pointer to execution time
 * \param nmemb Arrays number of members
 * \param filename File name to save the file
 * \return 0 ok, -1 error
 */
extern int robotCalcData(double *tc, int nmemb, char *filename);

/******************************************************************************/

/**
 * \brief  Save data into files. Used inside all threads
 * \param  type Which thread we want to save data
 * \param  ptr Pointer to data
 * \return void
 */
extern inline void saveToFileGeneric(int type, void *ptr);

#endif
